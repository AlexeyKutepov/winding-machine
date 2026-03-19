#include <AccelStepper.h>
#include <LiquidCrystal_I2C.h>

// пин подключения контакта VRX
#define PIN_VRX A0
// пин подключения контакта VRY
#define PIN_VRY A1
// пин подключения кнопки
#define PIN_START_STOP_BUTTON 7
const int DIR_PIN = 2;
const int ENABLE_PIN = 3;
const int STEP_PIN = 5;

// Используем пины с аппаратными прерываниями для кнопки
#define PIN_BUTTON_INTERRUPT 2

// ограничение по максимальному числу оборотов в минуту
const long MAX_RPM = 340;
// ограничение по минимальному числу оборотов в минуту
const long MIN_RPM = 60;
// ограничение по максимальному числу витков
const long MAX_TURNS = 20000;
// ограничение по минимальному числу витков
const long MIN_TURNS = 100;
// количество шагов за одну итерацию (200 шагов = 1 оборот)
const long STEPS_PER_REV = 200;

// ========== ИЗМЕНЕНО: параметры очень плавного разгона ==========
// Количество шагов для разгона 
const unsigned long ACCELERATION_STEPS = 2000; 
// Количество шагов для торможения
const unsigned long DECELERATION_STEPS = 2000;   
// Минимальная скорость при старте (% от целевой) - значительно уменьшена
const float START_SPEED_FACTOR = 0.1; 
// Максимальная скорость при торможении (% от целевой)
const float STOP_SPEED_FACTOR = 0.1;  
// Шаг изменения скорости при разгоне/торможении
const float SPEED_STEP_FACTOR = 0.01;  

// ========== НОВОЕ: дополнительные параметры для сверхплавного старта ==========
// Количество шагов для начального "ползучего" режима
const unsigned long CREEP_STEPS = 1;   // Первые N шагов на минимальной скорости
// Скорость в ползучем режиме (% от целевой)
const float CREEP_SPEED_FACTOR = 0.02;   // 2% от целевой скорости
// Использовать S-образную кривую разгона
const boolean USE_S_CURVE = true;        // S-кривая дает самый плавный старт

// Максимальная частота шагов в Гц
const long MAX_STEP_FREQUENCY = (MAX_RPM * STEPS_PER_REV) / 60;
// Минимальный интервал между шагами в микросекундах
const unsigned long MIN_STEP_INTERVAL_US = 1000000 / MAX_STEP_FREQUENCY;

// Debounce параметры
const unsigned long DEBOUNCE_DELAY = 50;
const unsigned long BUTTON_HOLD_TIME = 1000;

// пункты меню
const int MENU_WINDING_STATUS = 0;
const int MENU_SET_SPEED = 1;
const int MENU_SET_TURNS = 2;

// пункт меню
int menu = MENU_SET_SPEED;
// число оборотов в минуту
long speedRpm = MAX_RPM;
// число витков для намотки
long targetNumberOfTurns = 100;
// фактически намотанное количество витков
volatile long currentNumberOfTurns = 0;
// состояние намотки
volatile boolean winding = false;
// флаг для обновления дисплея
boolean lcdNeedsUpdate = true;
// целевая позиция в шагах
volatile long targetPosition = 0;

// переменная для безопасного чтения из прерывания
volatile long safeCurrentNumberOfTurns = 0;
// флаг, что значение изменилось
volatile boolean turnsUpdated = false;

// переменные для антидребезга кнопки
int lastButtonState = HIGH;
unsigned long lastDebounceTime = 0;
unsigned long buttonPressTime = 0;
boolean buttonHeld = false;

// Определение тип интерфейса двигателя
#define motorInterfaceType 1
// Создаем экземпляр
AccelStepper stepper(motorInterfaceType, STEP_PIN, DIR_PIN);

LiquidCrystal_I2C lcd(0x27,16,2);

// Переменные для управления джойстиком
unsigned long lastJoystickCheck = 0;
const unsigned long JOYSTICK_CHECK_INTERVAL = 50;

// Переменные для джойстика
int lastVrxValue = 512;
int lastVryValue = 512;
unsigned long lastVrxTriggerTime = 0;
unsigned long lastVryTriggerTime = 0;
const unsigned long JOYSTICK_COOLDOWN = 200;

// Переменные для аппаратного таймера
volatile unsigned long timerCounter = 0;
volatile boolean timerTick = false;

// Структура для хранения параметров движения
struct MotionParams {
  volatile long currentPosition;
  volatile long targetPosition;
  volatile unsigned long currentStepInterval;  // текущий интервал между шагами
  volatile unsigned long targetStepInterval;    // целевой интервал между шагами
  volatile unsigned long creepStepInterval;     // интервал в ползучем режиме
  volatile unsigned long startStepInterval;     // стартовый интервал
  volatile unsigned long stopStepInterval;      // интервал при торможении
  volatile boolean direction;
  volatile boolean accelerating;
  volatile boolean decelerating;
  volatile boolean creeping;                    // ползучий режим
} motion;

// для отслеживания предыдущего значения витков
long lastDisplayedTurns = -1;

// Флаг для отслеживания состояния драйвера
boolean driverEnabled = false;

// ========== НОВЫЕ: функции для очень плавного разгона ==========

// Функция для S-образной кривой (самый плавный старт)
float sCurve(float x) {
  // S-образная кривая: 3x^2 - 2x^3 (плавный старт и плавное нарастание)
  return 3 * x * x - 2 * x * x * x;
}

// Функция для расчета интервала по фактору скорости
unsigned long calculateStepIntervalByFactor(long rpm, float factor) {
  long stepsPerSecond = (rpm * STEPS_PER_REV) / 60;
  float effectiveSpeed = stepsPerSecond * factor;
  
  // Защита от деления на ноль
  if (effectiveSpeed < 1.0) effectiveSpeed = 1.0;
  
  unsigned long stepInterval = 1000000 / effectiveSpeed;
  
  // Ограничиваем минимальный интервал
  if (stepInterval < MIN_STEP_INTERVAL_US) {
    stepInterval = MIN_STEP_INTERVAL_US;
  }
  
  // Ограничиваем максимальный интервал (для очень медленных скоростей)
  if (stepInterval > 50000) {  // Максимум 50ms между шагами
    stepInterval = 50000;
  }
  
  return stepInterval * 2;  // конвертируем в такты таймера
}

// Функция для расчета целевого интервала
unsigned long calculateTargetStepInterval(long rpm) {
  return calculateStepIntervalByFactor(rpm, 1.0);
}

// Функция для расчета стартового интервала (очень медленно)
unsigned long calculateStartStepInterval(long rpm) {
  return calculateStepIntervalByFactor(rpm, START_SPEED_FACTOR);
}

// Функция для расчета ползучего интервала (экстремально медленно)
unsigned long calculateCreepStepInterval(long rpm) {
  return calculateStepIntervalByFactor(rpm, CREEP_SPEED_FACTOR);
}

// Функция для расчета интервала при торможении
unsigned long calculateStopStepInterval(long rpm) {
  return calculateStepIntervalByFactor(rpm, STOP_SPEED_FACTOR);
}

// Функция для обновления скорости разгона/торможения в прерывании
void updateRampingInISR() {
  if (!winding) return;
  
  long distanceToGo = motion.targetPosition - motion.currentPosition;
  long distanceFromStart = motion.currentPosition;
  
  // Ползучий режим (самые первые шаги)
  if (distanceFromStart < CREEP_STEPS) {
    motion.creeping = true;
    motion.accelerating = false;
    motion.decelerating = false;
    
    // В ползучем режиме используем постоянную минимальную скорость
    motion.currentStepInterval = motion.creepStepInterval;
  }
  // Плавный разгон
  else if (distanceFromStart < ACCELERATION_STEPS + CREEP_STEPS) {
    motion.creeping = false;
    motion.accelerating = true;
    motion.decelerating = false;
    
    // Прогресс разгона (0.0 до 1.0) с учетом ползучего режима
    float progress = (float)(distanceFromStart - CREEP_STEPS) / ACCELERATION_STEPS;
    if (progress > 1.0) progress = 1.0;
    
    float speedFactor;
    if (USE_S_CURVE) {
      // S-образная кривая - самый плавный старт
      speedFactor = START_SPEED_FACTOR + (1.0 - START_SPEED_FACTOR) * sCurve(progress);
    } else {
      // Квадратичная кривая
      speedFactor = START_SPEED_FACTOR + (1.0 - START_SPEED_FACTOR) * progress * progress;
    }
    
    unsigned long targetInterval = motion.targetStepInterval;
    unsigned long startInterval = motion.startStepInterval;
    
    // Интерполируем между стартовым и целевым интервалом
    motion.currentStepInterval = startInterval - 
      (unsigned long)((startInterval - targetInterval) * progress);
  }
  // Плавное торможение
  else if (distanceToGo < DECELERATION_STEPS) {
    motion.creeping = false;
    motion.accelerating = false;
    motion.decelerating = true;
    
    // Прогресс торможения (0.0 до 1.0)
    float progress = 1.0 - (float)distanceToGo / DECELERATION_STEPS;
    if (progress < 0.0) progress = 0.0;
    if (progress > 1.0) progress = 1.0;
    
    float speedFactor;
    if (USE_S_CURVE) {
      // S-образная кривая для торможения
      speedFactor = STOP_SPEED_FACTOR + (1.0 - STOP_SPEED_FACTOR) * (1.0 - sCurve(progress));
    } else {
      // Квадратичная кривая
      speedFactor = STOP_SPEED_FACTOR + (1.0 - STOP_SPEED_FACTOR) * (1.0 - progress * progress);
    }
    
    unsigned long targetInterval = motion.targetStepInterval;
    unsigned long stopInterval = motion.stopStepInterval;
    
    // Интерполируем между целевым и стоповым интервалом
    motion.currentStepInterval = targetInterval + 
      (unsigned long)((stopInterval - targetInterval) * progress);
  }
  // Постоянная скорость
  else {
    motion.creeping = false;
    motion.accelerating = false;
    motion.decelerating = false;
    motion.currentStepInterval = motion.targetStepInterval;
  }
  
  // Применяем новый интервал к таймеру
  OCR1A = motion.currentStepInterval;
}

// Функция для прямого отключения драйвера
void directDisableDriver() {
  noInterrupts();
  
  OCR1A = 65535;
  TCNT1 = 0;
  digitalWrite(STEP_PIN, LOW);
  digitalWrite(ENABLE_PIN, HIGH);
  stepper.disableOutputs();
  
  driverEnabled = false;
  winding = false;
  
  interrupts();
  
  Serial.println("Driver DISABLED");
}

// Функция для прямого включения драйвера
void directEnableDriver() {
  noInterrupts();
  
  digitalWrite(ENABLE_PIN, LOW);
  stepper.enableOutputs();
  
  driverEnabled = true;
  
  interrupts();
  
  Serial.println("Driver ENABLED");
}

// функция для проверки допустимой скорости
boolean isValidSpeed(long rpm) {
  return (rpm >= MIN_RPM && rpm <= MAX_RPM);
}

void setup() {
  Serial.begin(9600);
  
  pinMode(PIN_START_STOP_BUTTON, INPUT_PULLUP);
  pinMode(ENABLE_PIN, OUTPUT);
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  
  digitalWrite(ENABLE_PIN, HIGH);
  digitalWrite(STEP_PIN, LOW);
  
  lcd.init();
  lcd.backlight();

  // Настройка AccelStepper
  stepper.setPinsInverted(true, false, false);
  stepper.setEnablePin(ENABLE_PIN);
  stepper.setPinsInverted(false, false, true);
  
  stepper.disableOutputs();
  driverEnabled = false;
  
  // Настройка аппаратного таймера 1
  noInterrupts();
  
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = 0;
  
  OCR1A = 65535;  // Начинаем с максимального интервала (фактически останов)
  
  TCCR1B |= (1 << WGM12);
  TCCR1B |= (1 << CS11);
  TIMSK1 |= (1 << OCIE1A);
  
  EICRA |= (1 << ISC00);
  EIMSK |= (1 << INT0);
  
  interrupts();
  
  // Инициализация структуры движения
  motion.currentPosition = 0;
  motion.targetPosition = 0;
  motion.targetStepInterval = calculateTargetStepInterval(speedRpm);
  motion.creepStepInterval = calculateCreepStepInterval(speedRpm);
  motion.startStepInterval = calculateStartStepInterval(speedRpm);
  motion.stopStepInterval = calculateStopStepInterval(speedRpm);
  motion.currentStepInterval = motion.creepStepInterval;  // Начинаем с ползучей скорости
  motion.direction = true;
  motion.accelerating = false;
  motion.decelerating = false;
  motion.creeping = false;
  
  Serial.println("=== Motor Configuration ===");
  Serial.print("MAX RPM: ");
  Serial.println(MAX_RPM);
  Serial.print("ACCELERATION STEPS: ");
  Serial.println(ACCELERATION_STEPS);
  Serial.print("CREEP STEPS: ");
  Serial.println(CREEP_STEPS);
  Serial.print("START SPEED FACTOR: ");
  Serial.println(START_SPEED_FACTOR);
  Serial.print("CREEP SPEED FACTOR: ");
  Serial.println(CREEP_SPEED_FACTOR);
  Serial.print("USE S-CURVE: ");
  Serial.println(USE_S_CURVE ? "YES" : "NO");
  Serial.println("==========================");
  
  updateDisplay();
}

// Прерывание таймера 1 - генерирует шаги двигателя
ISR(TIMER1_COMPA_vect) {
  if (winding) {
    if (motion.currentPosition < motion.targetPosition) {
      digitalWrite(STEP_PIN, HIGH);
      delayMicroseconds(2);
      digitalWrite(STEP_PIN, LOW);
      
      motion.currentPosition++;
      
      // Обновляем счетчик витков
      if (motion.currentPosition % STEPS_PER_REV == 0) {
        currentNumberOfTurns = motion.currentPosition / STEPS_PER_REV;
        turnsUpdated = true;
      }
      
      // Обновляем разгон/торможение КАЖДЫЙ шаг для максимальной плавности
      updateRampingInISR();
      
    } else {
      stopWindingFromISR();
    }
  }
}

// Прерывание для кнопки
ISR(INT0_vect) {
  static unsigned long lastInterruptTime = 0;
  unsigned long currentTime = millis();
  
  if (currentTime - lastInterruptTime > DEBOUNCE_DELAY) {
    lastInterruptTime = currentTime;
  }
}

void stopWindingFromISR() {
  winding = false;
  digitalWrite(STEP_PIN, LOW);
  OCR1A = 65535;
}

void loop() {
  unsigned long currentMillis = millis();
  
  if (turnsUpdated) {
    noInterrupts();
    safeCurrentNumberOfTurns = currentNumberOfTurns;
    turnsUpdated = false;
    interrupts();
    
    lcdNeedsUpdate = true;
  }
  
  checkAndHandleStartStopButton(currentMillis);
  
  if (currentMillis - lastJoystickCheck >= JOYSTICK_CHECK_INTERVAL) {
    checkAndHandleVRX(currentMillis);
    checkAndHandleVRY(currentMillis);
    lastJoystickCheck = currentMillis;
  }
  
  if (lcdNeedsUpdate) {
    updateDisplay();
    lcdNeedsUpdate = false;
  }
  
  // Проверка завершения
  if (winding) {
    noInterrupts();
    long currentPos = motion.currentPosition;
    long targetPos = motion.targetPosition;
    interrupts();
    
    if (currentPos >= targetPos) {
      Serial.println("Target reached, stopping");
      directDisableDriver();
      menu = MENU_WINDING_STATUS;
      lcdNeedsUpdate = true;
    }
  }
  
  // Периодическая проверка состояния
  static unsigned long lastDriverCheck = 0;
  if (currentMillis - lastDriverCheck > 5000) {
    lastDriverCheck = currentMillis;
    
    if (!winding && driverEnabled) {
      Serial.println("WARNING: Driver enabled while not winding! Forcing disable...");
      directDisableDriver();
    }
  }
  
  delay(1);
}

void stopWinding() {
  Serial.println("stopWinding() called");
  directDisableDriver();
  
  noInterrupts();
  currentNumberOfTurns = motion.currentPosition / STEPS_PER_REV;
  safeCurrentNumberOfTurns = currentNumberOfTurns;
  interrupts();
  
  menu = MENU_WINDING_STATUS;
  lcdNeedsUpdate = true;
}

void checkAndHandleStartStopButton(unsigned long currentMillis) {
  int reading = digitalRead(PIN_START_STOP_BUTTON);
  
  if (reading != lastButtonState) {
    lastDebounceTime = currentMillis;
  }
  
  if ((currentMillis - lastDebounceTime) > DEBOUNCE_DELAY) {
    if (reading == LOW) {
      if (!buttonHeld) {
        if (buttonPressTime == 0) {
          buttonPressTime = currentMillis;
        }
        
        if (currentMillis - buttonPressTime >= BUTTON_HOLD_TIME) {
          if (winding) {
            emergencyStop();
          }
          buttonHeld = true;
        }
      }
    } else {
      if (buttonPressTime != 0 && !buttonHeld) {
        toggleWinding();
      }
      buttonPressTime = 0;
      buttonHeld = false;
    }
  }
  
  lastButtonState = reading;
}

void emergencyStop() {
  Serial.println("EMERGENCY STOP!");
  directDisableDriver();
  
  noInterrupts();
  currentNumberOfTurns = motion.currentPosition / STEPS_PER_REV;
  safeCurrentNumberOfTurns = currentNumberOfTurns;
  interrupts();
  
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("EMERGENCY STOP!");
  delay(1000);
  lcdNeedsUpdate = true;
}

void toggleWinding() {
  if (!winding) {
    startWinding();
  } else {
    pauseWinding();
  }
}

void startWinding() {
  Serial.println("startWinding() called");
  
  directDisableDriver();
  delay(10);
  directEnableDriver();
  delay(10);
  
  noInterrupts();
  motion.currentPosition = 0;
  motion.targetPosition = targetNumberOfTurns * STEPS_PER_REV;
  
  // Пересчитываем интервалы для текущей скорости
  motion.targetStepInterval = calculateTargetStepInterval(speedRpm);
  motion.creepStepInterval = calculateCreepStepInterval(speedRpm);
  motion.startStepInterval = calculateStartStepInterval(speedRpm);
  motion.stopStepInterval = calculateStopStepInterval(speedRpm);
  
  // Начинаем с ПОЛЗУЧЕЙ скорости (очень медленно)
  motion.currentStepInterval = motion.creepStepInterval;
  
  currentNumberOfTurns = 0;
  safeCurrentNumberOfTurns = 0;
  turnsUpdated = true;
  
  OCR1A = motion.creepStepInterval;  // Стартуем экстремально медленно
  TCNT1 = 0;
  winding = true;
  interrupts();
  
  menu = MENU_WINDING_STATUS;
  lcdNeedsUpdate = true;
  
  Serial.println("Winding started with CREEP SPEED");
  Serial.print("Creep interval: ");
  Serial.println(motion.creepStepInterval);
  Serial.print("Target interval: ");
  Serial.println(motion.targetStepInterval);
  Serial.print("Target turns: ");
  Serial.println(targetNumberOfTurns);
}

void pauseWinding() {
  Serial.println("pauseWinding() called");
  directDisableDriver();
  
  noInterrupts();
  currentNumberOfTurns = motion.currentPosition / STEPS_PER_REV;
  safeCurrentNumberOfTurns = currentNumberOfTurns;
  turnsUpdated = true;
  interrupts();
  
  lcdNeedsUpdate = true;
}

void checkAndHandleVRX(unsigned long currentMillis) {
  if (currentMillis - lastVrxTriggerTime < JOYSTICK_COOLDOWN) {
    return;
  }
  
  int x = analogRead(PIN_VRX);
  lastVrxValue = x;
  
  if (winding) {
    if (x > 700) {
      if (menu != MENU_WINDING_STATUS) {
        menu = MENU_WINDING_STATUS;
        lcdNeedsUpdate = true;
        lastVrxTriggerTime = currentMillis;
      }
    } else if (x < 300) {
      if (menu == MENU_WINDING_STATUS) {
        menu = MENU_SET_SPEED;
        lcdNeedsUpdate = true;
        lastVrxTriggerTime = currentMillis;
      }
    }
    return;
  }
  
  if (x > 700) {
    int newMenu = menu + 1;
    if (newMenu > MENU_SET_TURNS) {
      newMenu = MENU_SET_SPEED;
    }
    if (newMenu != menu) {
      menu = newMenu;
      lcdNeedsUpdate = true;
      lastVrxTriggerTime = currentMillis;
    }
  } else if (x < 300) {
    int newMenu = menu - 1;
    if (newMenu < MENU_SET_SPEED) {
      newMenu = MENU_SET_TURNS;
    }
    if (newMenu != menu) {
      menu = newMenu;
      lcdNeedsUpdate = true;
      lastVrxTriggerTime = currentMillis;
    }
  }
}

void checkAndHandleVRY(unsigned long currentMillis) {
  if (currentMillis - lastVryTriggerTime < JOYSTICK_COOLDOWN || winding) {
    return;
  }
  
  int y = analogRead(PIN_VRY);
  lastVryValue = y;
  
  boolean valueChanged = false;
  
  if (y < 200) {
    switch(menu) {
      case MENU_SET_SPEED:
        if (speedRpm + 10 <= MAX_RPM) {
          speedRpm = speedRpm + 10;
          valueChanged = true;
          Serial.print("Speed increased to: ");
          Serial.println(speedRpm);
        } else {
          Serial.println("Maximum speed reached!");
        }
        break;
      case MENU_SET_TURNS:
        if (targetNumberOfTurns + 100 <= MAX_TURNS) {
          targetNumberOfTurns = targetNumberOfTurns + 100;
          valueChanged = true;
        }
        break;
    }
    if (valueChanged) {
      lastVryTriggerTime = currentMillis;
    }
  } else if (y > 800) {
    switch(menu) {
      case MENU_SET_SPEED:
        if (speedRpm - 10 >= MIN_RPM) {
          speedRpm = speedRpm - 10;
          valueChanged = true;
          Serial.print("Speed decreased to: ");
          Serial.println(speedRpm);
        } else {
          Serial.println("Minimum speed reached!");
        }
        break;
      case MENU_SET_TURNS:
        if (targetNumberOfTurns - 100 >= MIN_TURNS) {
          targetNumberOfTurns = targetNumberOfTurns - 100;
          valueChanged = true;
        }
        break;
    }
    if (valueChanged) {
      lastVryTriggerTime = currentMillis;
    }
  }
  
  if (valueChanged) {
    lcdNeedsUpdate = true;
  }
}

void updateDisplay() {
  lcd.clear();
  
  switch (menu) {
    case MENU_WINDING_STATUS:
      lcd.setCursor(0, 0);
      if (winding) {
        // Показываем детальный статус разгона
        if (motion.creeping) {
          lcd.print("CREEPING...");
        } else if (motion.accelerating) {
          lcd.print("ACCELERATING...");
        } else if (motion.decelerating) {
          lcd.print("DECELERATING...");
        } else {
          lcd.print("WINDING...");
        }
      } else {
        lcd.print("STOPPED");
      }

      lcd.setCursor(0, 1);
      lcd.print(String(safeCurrentNumberOfTurns));
      lcd.print("/");
      lcd.print(String(targetNumberOfTurns));
      break;
      
    case MENU_SET_SPEED:
      lcd.setCursor(0, 0);
      lcd.print("SPEED (MAX 340):");
  
      lcd.setCursor(0, 1);
      lcd.print(String(speedRpm));
      lcd.print(" RPM");
      break;
      
    case MENU_SET_TURNS:
      lcd.setCursor(0, 0);
      lcd.print("TARGET TURNS:");

      lcd.setCursor(0, 1);
      lcd.print(String(targetNumberOfTurns));
      break;
  }
}

void disableStepper() {
  directDisableDriver();
}

void enableStepper() {
  directEnableDriver();
}
