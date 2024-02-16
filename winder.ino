#include <Stepper.h>
#include <LiquidCrystal_I2C.h>

// пин подключения контакта VRX
#define PIN_VRX A0
// пин подключения контакта VRY
#define PIN_VRY A1
// пин подключения кнопки
#define PIN_START_STOP_BUTTON 7
// пины управления двигателем
#define COIL1_MC1 2
#define COIL1_MC2 3
#define COIL2_MC1 5
#define COIL2_MC2 6

// ограничение по максимальному числу оборотов в минуту
const int MAX_RPM = 300;
// ограничение по максимальному числу витков
const int MAX_TURNS = 20000;
// количество шагов за одну итерацию (200 шагов = 1 оборот)
const int STEPS_PER_REV = 200*10;

// пункт меню
int menu = 1;
// число оборотов в минуту
int speed_rpm = 300;
// число витков для намотки
int max_number_of_turns = 8000;
// фактически намотанное количество витков
int number_of_turns = 0;
// состояние намотки (true - намотка идёт, false - намотка остановлена)
boolean winding = false;

Stepper stepper(200, COIL1_MC1, COIL1_MC2, COIL2_MC1, COIL2_MC2);
LiquidCrystal_I2C lcd(0x27,16,2);

void setup() {
  Serial.begin(9600);
  stepper.setSpeed(speed_rpm);

  lcd.init();
  lcd.backlight(); // включение подсветки дисплея

  digitalWrite(COIL1_MC1, LOW);
  digitalWrite(COIL1_MC2, LOW);
  digitalWrite(COIL2_MC1, LOW);
  digitalWrite(COIL2_MC2, LOW);
}

void loop() {  
  checkAndHandleStartStopButton();
  checkAndHandleVRX();
  checkAndHandleVRY();
  
  if (menu == 0) {
    lcd.setCursor(0,0);
    if (winding) {
      lcdPrint("WINDING...");
    } else {
      lcdPrint("COMPLETED!");
    }

    lcd.setCursor(0, 1);
    lcdPrint(String(number_of_turns));
  } else if (menu == 1) {
    lcd.setCursor(0, 0);
    lcdPrint("SPEED:");
  
    lcd.setCursor(0, 1);
    lcdPrint(String(speed_rpm) + " RPM");
  } else if (menu == 2) {
    lcd.setCursor(0,0);
    lcdPrint("NUMBER OF TURNS:");

    lcd.setCursor(0, 1);
    lcdPrint(String(max_number_of_turns));
  }
  lcd.display();

  if (winding) {  
    stepper.step(STEPS_PER_REV);
    number_of_turns = number_of_turns + 10;

    Serial.println("number_of_turns = "  + String(number_of_turns));
    Serial.println("number_of_turns = "  + String(max_number_of_turns));

    if (number_of_turns >= max_number_of_turns) {
      winding = false;
      disableStepper();
    }
  } 
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
    winding = true;
    menu = 0;
    number_of_turns = 0;
    stepper.setSpeed(speed_rpm);
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
      menu = menu - 1;
    }
  } else if (x < 300) {
    if (menu < 2) {
      menu = menu + 1;
    }
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
      case 1:
        speed_rpm = speed_rpm + 10;
        if (speed_rpm > MAX_RPM) {
          speed_rpm = MAX_RPM;
        }
        break;
      case 2:
        max_number_of_turns = max_number_of_turns + 100;
        if (max_number_of_turns > MAX_TURNS) {
          max_number_of_turns = MAX_TURNS;
        }
        break;
    }
  } else if (y > 800) {
    switch(menu) {
      case 1:
        speed_rpm = speed_rpm - 10;
        if (speed_rpm < 60) {
          speed_rpm = 60;
        }
        break;
      case 2:
        max_number_of_turns = max_number_of_turns - 100;
        if (max_number_of_turns < 100) {
          max_number_of_turns = 100;
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
    digitalWrite(COIL1_MC1, LOW);
    digitalWrite(COIL1_MC2, LOW);
    digitalWrite(COIL2_MC1, LOW);
    digitalWrite(COIL2_MC2, LOW);
}
