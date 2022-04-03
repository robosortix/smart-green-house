#include <Servo.h>
#include <LiquidCrystal_I2C.h>
#include <iarduino_RTC.h>
#include <iarduino_OLED.h>          // Подключаем библиотеку iarduino_OLED.
iarduino_OLED myOLED(0x3C);         // Объявляем объект myOLED, указывая адрес дисплея на шине I2C: 0x3C.
extern uint8_t MediumFont[];        // Подключаем шрифт MediumFontRus.

#include <iarduino_Pressure_BMP.h>  //  Подключаем библиотеку для работы с датчиками BMP180 или BMP280
// Создаём объект SensorP для работы с датчиком, адрес которого на шине I2C установлен по умолчанию.
iarduino_Pressure_BMP SensorP;

#include <iarduino_I2C_SHT.h>       // Подключаем библиотеку iarduino_SHT
// Объявляем объект SensorTH для работы с датчиком Датчик температуры и влажности, FLASH-I2C
iarduino_I2C_SHT SensorTH(0x10);
iarduino_RTC time(RTC_DS1302,12,11,10);  // rst, clk, dat
Servo windowServo;
Servo wateringServo;

// Commands, cmd and param:
// Get illum, Get th, Get watertime, Get waterdur
// Set watertime, Set waterdur
// Window open, Window close
// Watering on

// Вывод отладочных сообщений в монитор порта
#define DEBUG  1

#define WATERTIME1_VALUE_ADDR  0x00
#define WATERTIME2_VALUE_ADDR  0x01
#define WATERDUR_VALUE_ADDR    0x02
#define WINDOW_STATE_ADDR      0x03
#define WATERING_STATE_ADDR    0x04
#define ILLUM_STATE_ADDR       0x05
#define WINDOW_STATE_ADDR      0x06

#define DAY_NIGHT_ILLUM_VAL    500
#define HOT_COLD_TEMP_VAL      30

#define PIN_LIGHT_ON_OFF       6
#define PIN_VENT_ON_OFF        5
#define PIN_WATER_MOTOR        2
#define PIN_WINDOW_SERVO       3

const uint8_t Temperature[] PROGMEM = { 24, 24,             // Массив для картинки знака температуры.
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFC, 0x02, //
0x01, 0x01, 0x01, 0x02, 0xFC, 0x00, 0xA0, 0xA0, 0x20, 0x00, //
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, //
0x00, 0x00, 0xFF, 0x00, 0xFE, 0xFF, 0xFE, 0x00, 0xFF, 0x00, //
0x2A, 0x2A, 0x22, 0x22, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, //
0x00, 0x00, 0x00, 0x00, 0x1E, 0x21, 0x40, 0x8E, 0x9F, 0x9F, //
0x9F, 0x8E, 0x40, 0x21, 0x1E, 0x00, 0x00, 0x00, 0x00, 0x00, //
0x00, 0x00};

const uint8_t Humidity[] PROGMEM = { 24, 24,                // Массив для картинки знака влажности.
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0xC0, 0x30, 0x18, //
0x0C, 0x06, 0x0C, 0x18, 0x30, 0xC0, 0x80, 0x00, 0x00, 0x00, //
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xF0, 0x1C, 0x07, //
0x01, 0x30, 0x78, 0x4C, 0xF8, 0xE0, 0xF0, 0x58, 0xC0, 0x80, //
0x01, 0x06, 0x1C, 0xF0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, //
0x00, 0x03, 0x06, 0x08, 0x10, 0x20, 0x20, 0x63, 0x41, 0x40, //
0x43, 0x66, 0x23, 0x21, 0x10, 0x08, 0x06, 0x03, 0x00, 0x00, 
0x00, 0x00};

const uint8_t Pressure[] PROGMEM = { 24, 24,                // Массив для картинки знака давления.
0x00, 0xC0, 0xE0, 0x30, 0x18, 0x8C, 0x46, 0x26, 0x26, 0x13, //
0x13, 0x13, 0x13, 0x13, 0x13, 0x26, 0x22, 0x46, 0x8C, 0x18, //
0x38, 0xE0, 0xC0, 0x00, 0xFE, 0xFF, 0x01, 0x10, 0x7E, 0x81, //
0x00, 0x00, 0x00, 0x00, 0x20, 0x18, 0x18, 0x04, 0x02, 0x01, //
0x00, 0x00, 0x81, 0x7E, 0x10, 0x01, 0xFF, 0xFF, 0x00, 0x03, //
0x07, 0x1C, 0x18, 0x31, 0x62, 0x60, 0xE0, 0xC0, 0xC0, 0xC0, //
0xC0, 0xC0, 0xC0, 0xE0, 0x60, 0x62, 0x31, 0x18, 0x1C, 0x07, //
0x03, 0x00};

uint16_t  LUM_result;           // объявляем переменную для хранения результата опроса датчика освещённости
uint8_t PIN_illumination = A3;  // номер аналогового вывода к которому подключён датчик освещенности
String recStr = "", command, param, value;
char symbol;
char cmd[10], par[10], val[10], buf[50];
int watering_time;

void setup() {
  delay(1000);
  SensorP.begin();               // Инициируем работу с датчиком (начальная высота по умолчанию = 0 метров)
  SensorTH.begin();              // Инициируем работу с датчиком AM2320.
  myOLED.begin();                // Инициируем работу с дисплеем.
  time.begin();
  //time.settime(00,22,22,01,04,22,5);  // 00  сек, 22 мин, 22 час, 1 апреля, 2022 года, пятница
  Serial.begin(9600);
  myOLED.setFont(MediumFont); // Указываем шрифт который требуется использовать для вывода цифр и текста.
  myOLED.print(F("Smart-G-H"), OLED_C, OLED_C);  // Выводим надпись по центру. 
  delay(3000);
  myOLED.clrScr();
  eeprom_write_byte(ILLUM_STATE_ADDR, 0); // Освещение выключено
  eeprom_write_byte(WINDOW_STATE_ADDR, 0); // Форточка закрыта
  eeprom_write_byte(WATERING_STATE_ADDR, 0); // Полив выключен
  if (eeprom_read_byte(WATERDUR_VALUE_ADDR) == 0xFF) {
    eeprom_write_byte(WATERDUR_VALUE_ADDR, 3);  // 3 sec default
  }
  pinMode(PIN_LIGHT_ON_OFF, OUTPUT);
  pinMode(PIN_WATER_MOTOR, OUTPUT);
  pinMode(PIN_VENT_ON_OFF, OUTPUT);
  digitalWrite(PIN_WATER_MOTOR, LOW);
  digitalWrite(PIN_LIGHT_ON_OFF, LOW);
}

void loop() {

   if (Serial.available() > 0) {  //если есть доступные данные     
     do {
       symbol = Serial.read();
       if (symbol != '\xFF') {
         if (symbol == '\x0A') {
           break;
         } else {
           recStr += symbol;
         }       
       }
     } while (1);

     recStr.toCharArray(buf, 50);
     recStr = "";
     sscanf(buf, "%s%s%s", &cmd, &par, &val);
     command = String(cmd);
     param = String(par);
     value = String(val);
     commandProcessing(command, param, value);
     cmd[0] = 0;
     par[0] = 0;
     val[0] = 0;
     buf[0] = 0;
   }
   
   LUM_result = 1024 - analogRead(PIN_illumination);   // опрашиваем датчик освещённости
   if (DEBUG) {
     Serial.print(LUM_result);
     Serial.println(" lum");
   }
   myOLED.print(LUM_result, 25, 40);
   myOLED.print(" lum");
   if (eeprom_read_byte(ILLUM_STATE_ADDR) == 0) {
     if (LUM_result > DAY_NIGHT_ILLUM_VAL) {
       eeprom_write_byte(ILLUM_STATE_ADDR, 1); // Освещение включено
       digitalWrite(PIN_LIGHT_ON_OFF, HIGH);
       if (DEBUG) {
         Serial.println("Light ON!");
       }
     }
   } else {
     if (LUM_result < DAY_NIGHT_ILLUM_VAL) {
       eeprom_write_byte(ILLUM_STATE_ADDR, 0); // Освещение выключено
       digitalWrite(PIN_LIGHT_ON_OFF, LOW);
       if (DEBUG) {
         Serial.println("Light OFF!");
       }
     }
   }
   delay(1000);
   myOLED.clrScr();
   if (DEBUG) {
     Serial.println(time.gettime("d-m-Y, H:i:s")); 
   }
   checkTimeForWatering();
   myOLED.print(time.gettime("d-m-Y"), 7, 40); // выводим дату
   delay(1000);
   myOLED.clrScr();
   myOLED.print(time.gettime("H:i:s"), 15, 40);  // выводим время
   delay(1000);
   myOLED.clrScr();
  
    // Читаем показания датчика температуры и влажности.
    myOLED.drawImage(Temperature,   5, 45, IMG_ROM);    // Выводим на дисплей знак температуры.
    float temp = SensorTH.getTem();
    myOLED.print(temp, 43, 40, 1);  // Выводим на дисплей показания температуры.
    if (DEBUG) {
      Serial.print(temp);
      Serial.println(" C");
    }
    if (eeprom_read_byte(WINDOW_STATE_ADDR) == 0) {
      if (temp >= HOT_COLD_TEMP_VAL) {
       eeprom_write_byte(WINDOW_STATE_ADDR, 1); // Открываем форточку
       windowServo.attach(PIN_WINDOW_SERVO);
       servoSmoothRotationM(windowServo, 70, 140);
       delay(100);
       windowServo.detach();
       digitalWrite(PIN_VENT_ON_OFF, HIGH);
       if (DEBUG) {
         Serial.println("Window OPEN!");
       }
      }
    } else {
      if (temp < HOT_COLD_TEMP_VAL) {
        eeprom_write_byte(WINDOW_STATE_ADDR, 0); // Закрываем форточку
        windowServo.attach(PIN_WINDOW_SERVO);
        servoSmoothRotationM(windowServo, 140, 70);
        delay(100);
        windowServo.detach();
        digitalWrite(PIN_VENT_ON_OFF, LOW);
        if (DEBUG) {
          Serial.println("Window CLOSE!");
        }
      }
    }
    myOLED.drawCircle (100,  30, 3, false, 1); // Выводим на дисплей обозначение температуры в виде фигуры круга с радиусом 3.
    myOLED.drawCircle (100,  30, 2, false, 1); // Выводим на дисплей обозначение температуры в виде фигуры круга с радиусом 2.
    delay(1000);
    myOLED.clrScr();
    myOLED.drawImage(Humidity,   5, 45, IMG_ROM);  // Выводим на дисплей знак влажности.
    myOLED.print(SensorTH.getHum(), 45, 40, 1);    // Выводим на дисплей показания влажности.
    myOLED.print(F("%"), 100, 40);                 // Выводим на дисплей обозначение влажности.
    if (DEBUG) {
      Serial.print(SensorTH.getHum());
      Serial.println("%");
    }
    delay(1000);
    myOLED.clrScr();

  // Читаем показания датчика давления.
  /*if (SensorP.read(1))  {
    myOLED.drawImage(Pressure,   5, 45, IMG_ROM);  // Выводим на дисплей знак давления.
    myOLED.print(SensorP.pressure, 35, 40, 1);     // Выводим на дисплей показания давления.
    myOLED.print(F("MM"), 100, 40);                // Выводим на дисплей обозначение давления. 
    Serial.print(SensorP.pressure);
    Serial.println("mm");
    delay(1000);
    myOLED.clrScr();
  }*/
}

void checkTimeForWatering(void) {
  String curr_time;
  char buf[16];
  int hour, mins, sec, watering_hour;
  curr_time = time.gettime("H:i:s");
  curr_time.toCharArray(buf, 16);
  sscanf(buf, "%d:%d:%d", &hour, &mins, &sec);
  if (eeprom_read_byte(WATERING_STATE_ADDR) == 0) { // Полива не было в этом часе
    watering_hour = eeprom_read_byte(WATERTIME1_VALUE_ADDR);
    if (watering_hour == hour) { // Пора включать утренний полив
      eeprom_write_byte(WATERING_STATE_ADDR, 1); // Полив был в этом утреннем часе
      digitalWrite(PIN_WATER_MOTOR, HIGH); // Включаем полив
      if (DEBUG) {
        Serial.println("Watering ON!");
      }
      delay(1000*eeprom_read_byte(WATERDUR_VALUE_ADDR)); // Поливаем
      digitalWrite(PIN_WATER_MOTOR, LOW); // Выключаем полив
      if (DEBUG) {
        Serial.println("Watering OFF!");
      }
      watering_time = hour;
    } else {
      watering_hour = eeprom_read_byte(WATERTIME2_VALUE_ADDR);
      if (watering_hour == hour) { // Пора включать вечерний полив
        eeprom_write_byte(WATERING_STATE_ADDR, 2); // Полив был в этом вечернем часе
        digitalWrite(PIN_WATER_MOTOR, HIGH); // Включаем полив
        if (DEBUG) {
          Serial.println("Watering ON!");
        }
        delay(1000*eeprom_read_byte(WATERDUR_VALUE_ADDR)); // Поливаем
        digitalWrite(PIN_WATER_MOTOR, LOW); // Выключаем полив
        if (DEBUG) {
          Serial.println("Watering OFF!");
        }
        watering_time = hour;
      }
   }
  
  } else { // Полив был в этом часе
    // Как только настанет следующий час, нужно обнулить WATERING_STATE_ADDR
    if (hour == (watering_time+1)) {
      eeprom_write_byte(WATERING_STATE_ADDR, 0);
    }
  }
}

void commandProcessing(String cmd, String par, String val) {
/* Get illum, Get th, Get watertime, Get waterdur
   Set watertime, Set waterdur
   Window open, Window close
   Watering on
*/  
   String text = cmd + " " + par;
   if (DEBUG) {
     Serial.println(text);
   }
   if (text  == "Get illum") {
     LUM_result = 1024 - analogRead(PIN_illumination);   // опрашиваем датчик освещённости
     Serial.println(LUM_result);
     
   } else if (text == "Get th") {
      Serial.print(SensorTH.getTem());
      Serial.print(", ");
      Serial.print(SensorTH.getHum());
      Serial.println("%");
   
   } else if (text == "Get watertime") {
       String answer = "";
       int wt = eeprom_read_byte(WATERTIME1_VALUE_ADDR);
       if (wt == 0xFF) {
         answer = "wt1: not set, ";         
       } else {
         answer = "wt1: " + String(wt) + ", ";
       }
       wt = eeprom_read_byte(WATERTIME2_VALUE_ADDR);
       if (wt == 0xFF) {
         answer = answer + "wt2: not set";         
       } else {
         answer = answer + "wt2: " + String(wt);
       }
       Serial.println(answer);
   
   } else if (text == "Get waterdur") {
     String answer = "";
     int wd = eeprom_read_byte(WATERDUR_VALUE_ADDR);
     if (wd == 0xFF) {
       answer = "wd: not set";         
     } else {
       answer = "wd: " + String(wd);
     }
     Serial.println(answer);
  
   } else if (text == " watertime1") {
      eeprom_write_byte(WATERTIME1_VALUE_ADDR, val.toInt());
      Serial.println("ok");

   } else if (text == " watertime2") {
      eeprom_write_byte(WATERTIME2_VALUE_ADDR, val.toInt());
      Serial.println("ok");

   } else if (text == "Set waterdur") {
      eeprom_write_byte(WATERDUR_VALUE_ADDR, val.toInt());
      Serial.println("ok");

   } else if (text == "Watering on") {
      digitalWrite(PIN_WATER_MOTOR, HIGH); // Включаем полив
      Serial.println("ok");
      if (DEBUG) {
        Serial.println("Watering ON!");
      }
      delay(1000*eeprom_read_byte(WATERDUR_VALUE_ADDR)); // Поливаем
      digitalWrite(PIN_WATER_MOTOR, LOW); // Выключаем полив
      if (DEBUG) {
        Serial.println("Watering OFF!");
      }

   } else if (text == "Window open") {
      int ws = eeprom_read_byte(WINDOW_STATE_ADDR);
      if ((ws == 0xFF) || (ws == 0)) {
        eeprom_write_byte(WINDOW_STATE_ADDR, 1);
        Serial.println("ok");
        windowServo.attach(PIN_WINDOW_SERVO);
        servoSmoothRotationM(windowServo, 70, 140);
        delay(100);
        windowServo.detach();
      } else {
        Serial.println("Window already opened");
      }
   
   } else if (text == "Window close") {
      int ws = eeprom_read_byte(WINDOW_STATE_ADDR);
      if ((ws == 0xFF) || (ws == 1)) {
        eeprom_write_byte(WINDOW_STATE_ADDR, 0);
        windowServo.attach(PIN_WINDOW_SERVO);
        servoSmoothRotationM(windowServo, 140, 70);
        delay(100);
        windowServo.detach();
        Serial.println("ok");
      } else {
        Serial.println("Window already closed");
      }
  }
}

void servoSmoothRotationM(Servo &s, int angle1, int angle2) {
  // входные параметры функции:
  // s - ссылка на объект класса Servo (передаем в функцию сервопривод, который
  // нужно повернуть на определенный угол) angle1 - начальный угол поворота
  // сервопривода angle2 - конечный угол поворота сервопривода
  int step;
  int i = 0;

  if (angle1 > angle2) {
    step = -1; // шаг поворота = -1 град
    i = angle1;
    do {
      i += step;
      s.write(i); // плавно поворачиваем сервопривод с шагом -1 град.
      delay(25);
    } while (i >= angle2);
  } else {
    step = 1; // шаг поворота = +1 град
    i = angle1;
    do {
      i += step;
      s.write(i); // плавно поворачиваем сервопривод с шагом +1 град.
      delay(25);
    } while (i <= angle2);
  }
}
