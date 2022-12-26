#include "TinyMPU6050Reset.h" //Собственно ручно переписанная библиотека
#include "TinyWireM.h"        //Библиотека управления I2C
#include "TinyOzOLED.h"       //Библиотека управление дисплеем

MPU6050 MPU;
int vectorprevious, vector, totalvector;
int Steps = 0;

void setup() {
  TinyWireM.begin();
  OzOled.init();                              //Инициализация Дисплея
  OzOled.clearDisplay();                      //Очистка дисплея
  OzOled.setNormalDisplay();
  OzOled.sendCommand(0xA1);                   // установки ориентации дисплея
  OzOled.sendCommand(0xC8);
  MPU.BaseInititalize();                      //Инициализация MPU
  OzOled.printString("Starting", 0, 0);       //Индикация что пошла калибровка MPU
  OzOled.printString("calibration...", 0, 2);
  MPU.Calibrate();                            //Калибровка MPU
  OzOled.clearDisplay();
  OzOled.printString("Calibration", 0, 0);    //Индикация что прошла калибровка MPU
  OzOled.printString("complete!", 0, 2);
  OzOled.clearDisplay();
}

void loop() {
  MPU.Execute();  //Получение данных с MPU
  vector = sqrt( (MPU.GetGyroX() * MPU.GetGyroX()) + (MPU.GetGyroY() * MPU.GetGyroY()) + (MPU.GetGyroZ() * MPU.GetGyroZ()) ); //Устанавливаем новый вектор
  totalvector = vector - vectorprevious;  //Сравниваем со прошлым вектором
  if (totalvector > 15){  //Калибровка шага
    Steps++;  //Прибавление шагов
  }
  OzOled.printString("Steps: ", 0, 4);  //Выводим строчку на экран
  OzOled.printNumber(Steps, 0, 8, 4); //Выводим сами шаги
  OzOled.printString("Temp: ", 0, 6);  //Выводим строчку на экран
  OzOled.printNumber((float)MPU.readTemperature(), 0, 6, 6); //Выводим температуру
  vectorprevious = vector;//Приравниваем новый вектор к старому
  delay(250); //Пауза в 250 миллисекунд
}
