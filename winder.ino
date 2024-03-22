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

// ограничение по максимальному числу оборотов в минуту
const long MAX_RPM = 600;
// ограничение по минимальному числу оборотов в минуту
const long MIN_RPM = 60;
// ограничение по максимальному числу витков
const long MAX_TURNS = 20000;
// ограничение по максимальному числу витков
const long MIN_TURNS = 100;
// количество шагов за одну итерацию (200 шагов = 1 оборот)
const long STEPS_PER_REV = 200;

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
long currerntNumberOfTurns = 0;
// состояние намотки (true - намотка идёт, false - намотка остановлена)
boolean winding = false;

// Определение тип интерфейса двигателя
#define motorInterfaceType 1
// Создаем экземпляр
AccelStepper stepper(motorInterfaceType, STEP_PIN, DIR_PIN);

LiquidCrystal_I2C lcd(0x27,16,2);

void setup() {
  Serial.begin(9600);

  lcd.init();
  lcd.backlight(); // включение подсветки дисплея

  stepper.setPinsInverted(true, false, false);
  disableStepper();
}

void loop() {  
  if (winding) {  
    stepper.runToPosition();
    currerntNumberOfTurns = stepper.currentPosition()/200;

    Serial.println("currerntNumberOfTurns = "  + String(currerntNumberOfTurns));
    Serial.println("targetNumberOfTurns = "  + String(targetNumberOfTurns));

    if (stepper.distanceToGo() == 0) {
      winding = false;
      disableStepper();
    }
  } 
  
  checkAndHandleStartStopButton();
  checkAndHandleVRX();
  checkAndHandleVRY();

  switch (menu) {
    case MENU_WINDING_STATUS:
      lcd.setCursor(0,0);
      if (winding) {
        lcdPrint("WINDING...");
      } else {
        lcdPrint("COMPLETED!");
      }

      lcd.setCursor(0, 1);
      lcdPrint(String(currerntNumberOfTurns));
      break;
    case MENU_SET_SPEED:
      lcd.setCursor(0, 0);
      lcdPrint("SPEED:");
  
      lcd.setCursor(0, 1);
      lcdPrint(String(speedRpm) + " RPM");
      break;
    case MENU_SET_TURNS:
      lcd.setCursor(0,0);
      lcdPrint("NUMBER OF TURNS:");

      lcd.setCursor(0, 1);
      lcdPrint(String(targetNumberOfTurns));
      break;
  }
  lcd.display();
}

/**
 * Обработка нажатия кнопки старт/стоп
 */
void checkAndHandleStartStopButton() {
  boolean startStopButtonState = digitalRead(PIN_START_STOP_BUTTON);
  startStopButtonState = debounce(startStopButtonState, PIN_START_STOP_BUTTON);
  if (startStopButtonState == HIGH) {
    return;
  }
  if (!winding) {
    enableStepper();
    
    winding = true;
    menu = 0;
    currerntNumberOfTurns = 0;
    long stepperSpeed = (speedRpm * STEPS_PER_REV) / 60;
    
    stepper.setMaxSpeed(stepperSpeed);
    stepper.setAcceleration(100);
    stepper.setSpeed(stepperSpeed);
    stepper.move(targetNumberOfTurns * STEPS_PER_REV); 
    
    Serial.println("Start winding");
    delay(1000);
  } else {
    winding = false;
    disableStepper();
    Serial.println("Stop winding");
    delay(1000);
  }
}

/**
 * Управление пунктами меню с помощью джойстика
 */
void checkAndHandleVRX() {   
  if (winding) {
    return;
  }
  int x = analogRead(PIN_VRX);
  Serial.print("X = ");
  Serial.println(x);
  if (x > 700) {
    if (menu > 1) {
      menu--;
    } else {
      menu = MENU_SET_TURNS;
    }
    delay(100);
  } else if (x < 300) {
    if (menu < 2) {
      menu++;
    } else {
      menu = MENU_SET_SPEED;
    }
    delay(100);
  }
}

/**
 * Настройка параметров станка с помощью джойстика
 */
void checkAndHandleVRY() {   
  if (winding) {
    return;
  }
  int y = analogRead(PIN_VRY);
  Serial.print("Y = ");
  Serial.println(y);
  if (y < 200) {
    switch(menu) {
      case MENU_SET_SPEED:
        speedRpm = speedRpm + 10;
        if (speedRpm > MAX_RPM) {
          speedRpm = MAX_RPM;
        }
        break;
      case MENU_SET_TURNS:
        targetNumberOfTurns = targetNumberOfTurns + 100;
        if (targetNumberOfTurns > MAX_TURNS) {
          targetNumberOfTurns = MAX_TURNS;
        }
        break;
    }
  } else if (y > 800) {
    switch(menu) {
      case MENU_SET_SPEED:
        speedRpm = speedRpm - 10;
        if (speedRpm < MIN_RPM) {
          speedRpm = MIN_RPM;
        }
        break;
      case MENU_SET_TURNS:
        targetNumberOfTurns = targetNumberOfTurns - 100;
        if (targetNumberOfTurns < MIN_TURNS) {
          targetNumberOfTurns = MIN_TURNS;
        }
        break;
    }
  }
}

/**
 * Считывание значения кнопки с поправкой на дребезг контактов
 */
boolean debounce(boolean last, int pin) {
  boolean current = digitalRead(pin);
  if (last != current) {
    delay(100);
    current = digitalRead(pin);
  }
  return current;
}

/**
 * Вывод информации на дисплей
 */
void lcdPrint(String str) {
  while (str.length() < 16) {
    str = str + " ";
  }
  lcd.print(str);
}

/**
 * Отключение питания на управляющих контактах двигателя
 */
void disableStepper() {
    digitalWrite(ENABLE_PIN, LOW);
}

void enableStepper() {
    digitalWrite(ENABLE_PIN, HIGH);
}
