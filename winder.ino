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
#define PIN_BUTTON_INTERRUPT 2  // Используем пин 2 для прерывания (INT0)

// ограничение по максимальному числу оборотов в минуту
const long MAX_RPM = 300;
// ограничение по минимальному числу оборотов в минуту
const long MIN_RPM = 60;
// ограничение по максимальному числу витков
const long MAX_TURNS = 20000;
// ограничение по минимальному числу витков
const long MIN_TURNS = 100;
// количество шагов за одну итерацию (200 шагов = 1 оборот)
const long STEPS_PER_REV = 200;

// Буфер для накопления шагов
volatile long stepBuffer = 0;
volatile boolean stepRequested = false;
volatile unsigned long lastStepTime = 0;

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

// НОВОЕ: переменная для безопасного чтения из прерывания
volatile long safeCurrentNumberOfTurns = 0;
// НОВОЕ: флаг, что значение изменилось
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
  volatile unsigned long stepInterval;
  volatile boolean direction;
} motion;

// НОВОЕ: для отслеживания предыдущего значения витков
long lastDisplayedTurns = -1;

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
  
  // Настройка аппаратного таймера 1
  noInterrupts();
  
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = 0;
  
  OCR1A = 1000;
  
  TCCR1B |= (1 << WGM12);
  TCCR1B |= (1 << CS11);
  TIMSK1 |= (1 << OCIE1A);
  
  EICRA |= (1 << ISC00);
  EIMSK |= (1 << INT0);
  
  interrupts();
  
  motion.currentPosition = 0;
  motion.targetPosition = 0;
  motion.stepInterval = 1000;
  motion.direction = true;
  
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
      
      // Обновляем счетчик витков каждые 200 шагов
      if (motion.currentPosition % STEPS_PER_REV == 0) {
        currentNumberOfTurns = motion.currentPosition / STEPS_PER_REV;
        // НОВОЕ: устанавливаем флаг для основного цикла
        turnsUpdated = true;
      }
      
      // Плавный разгон и торможение
      static unsigned long rampCounter = 0;
      rampCounter++;
      
      if (motion.currentPosition < 1000) {
        if (rampCounter % 10 == 0 && OCR1A > 200) {
          OCR1A -= 2;
        }
      } else if (motion.targetPosition - motion.currentPosition < 1000) {
        if (rampCounter % 10 == 0) {
          OCR1A += 2;
        }
      }
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
  
  // НОВОЕ: безопасно копируем значение из прерывания
  if (turnsUpdated) {
    noInterrupts();  // Отключаем прерывания на время копирования
    safeCurrentNumberOfTurns = currentNumberOfTurns;
    turnsUpdated = false;
    interrupts();    // Включаем обратно
    
    lcdNeedsUpdate = true;
    
    // Отладочный вывод
    Serial.print("Turns updated: ");
    Serial.println(safeCurrentNumberOfTurns);
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
  
  // НОВОЕ: дополнительная проверка завершения
  if (winding) {
    // Используем безопасное копирование для проверки
    noInterrupts();
    long currentPos = motion.currentPosition;
    long targetPos = motion.targetPosition;
    interrupts();
    
    if (currentPos >= targetPos) {
      stopWinding();
    }
  }
  
  delay(1);
}

void stopWinding() {
  noInterrupts();
  winding = false;
  OCR1A = 65535;
  digitalWrite(STEP_PIN, LOW);
  
  // НОВОЕ: финальное обновление счетчика
  currentNumberOfTurns = motion.currentPosition / STEPS_PER_REV;
  safeCurrentNumberOfTurns = currentNumberOfTurns;
  interrupts();
  
  stepper.disableOutputs();
  menu = MENU_WINDING_STATUS;
  lcdNeedsUpdate = true;
  Serial.println("Winding completed");
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
  noInterrupts();
  winding = false;
  OCR1A = 65535;
  digitalWrite(STEP_PIN, LOW);
  
  // НОВОЕ: обновляем счетчик при экстренной остановке
  currentNumberOfTurns = motion.currentPosition / STEPS_PER_REV;
  safeCurrentNumberOfTurns = currentNumberOfTurns;
  interrupts();
  
  stepper.stop();
  stepper.disableOutputs();
  
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("EMERGENCY STOP!");
  delay(1000);
  lcdNeedsUpdate = true;
  Serial.println("EMERGENCY STOP!");
}

void toggleWinding() {
  if (!winding) {
    startWinding();
  } else {
    pauseWinding();
  }
}

void startWinding() {
  // НОВОЕ: сбрасываем все счетчики
  noInterrupts();
  motion.currentPosition = 0;
  motion.targetPosition = targetNumberOfTurns * STEPS_PER_REV;
  currentNumberOfTurns = 0;
  safeCurrentNumberOfTurns = 0;
  turnsUpdated = true;  // Чтобы обновился дисплей
  interrupts();
  
  // Рассчитываем интервал шагов
  long stepsPerSecond = (speedRpm * STEPS_PER_REV) / 60;
  unsigned long stepInterval = 1000000 / stepsPerSecond;
  unsigned int timerTicks = stepInterval * 2;
  
  noInterrupts();
  motion.stepInterval = timerTicks;
  OCR1A = timerTicks;
  TCNT1 = 0;
  winding = true;
  interrupts();
  
  stepper.enableOutputs();
  menu = MENU_WINDING_STATUS;
  lcdNeedsUpdate = true;
  
  Serial.print("Start winding. Timer ticks: ");
  Serial.println(timerTicks);
}

void pauseWinding() {
  noInterrupts();
  winding = false;
  OCR1A = 65535;
  digitalWrite(STEP_PIN, LOW);
  
  // НОВОЕ: сохраняем текущее значение при паузе
  currentNumberOfTurns = motion.currentPosition / STEPS_PER_REV;
  safeCurrentNumberOfTurns = currentNumberOfTurns;
  turnsUpdated = true;
  interrupts();
  
  stepper.disableOutputs();
  Serial.println("Pause winding");
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
        if (speedRpm < MAX_RPM) {
          speedRpm = min(speedRpm + 10, MAX_RPM);
          valueChanged = true;
        }
        break;
      case MENU_SET_TURNS:
        if (targetNumberOfTurns < MAX_TURNS) {
          targetNumberOfTurns = min(targetNumberOfTurns + 100, MAX_TURNS);
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
        if (speedRpm > MIN_RPM) {
          speedRpm = max(speedRpm - 10, MIN_RPM);
          valueChanged = true;
        }
        break;
      case MENU_SET_TURNS:
        if (targetNumberOfTurns > MIN_TURNS) {
          targetNumberOfTurns = max(targetNumberOfTurns - 100, MIN_TURNS);
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
        lcd.print("WINDING...");
      } else {
        lcd.print("STOPPED");
      }

      lcd.setCursor(0, 1);
      // НОВОЕ: используем безопасное значение для отображения
      lcd.print(String(safeCurrentNumberOfTurns));
      lcd.print("/");
      lcd.print(String(targetNumberOfTurns));
      
      // Отладка - показываем, что значение меняется
      if (safeCurrentNumberOfTurns != lastDisplayedTurns) {
        lastDisplayedTurns = safeCurrentNumberOfTurns;
        Serial.print("Displaying: ");
        Serial.println(safeCurrentNumberOfTurns);
      }
      break;
      
    case MENU_SET_SPEED:
      lcd.setCursor(0, 0);
      lcd.print("SPEED:");
  
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
  stepper.disableOutputs();
}

void enableStepper() {
  stepper.enableOutputs();
}
