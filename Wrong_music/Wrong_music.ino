/*********
  Shchuchkin Evgeny
  
  First version of "Wrong Music" rocket controller program
*********/
/*
#include "WM_RF.h"

#include "WM_Gyro.h"
#include "WM_Servo.h"

#include "WM_BMP280.h"
#include "WM_GPS.h"
#include "WM_SD.h"
*/




#include <Wire.h>
#include <SPI.h>
#include <Adafruit_BMP280.h>

#include <TinyGPS++.h>                  // Подключение библиотеки TinyGPS
#include <SoftwareSerial.h>             // Подключение библиотеки SoftwareSerial


#include <SD.h>
const int PIN_CHIP_SELECT = 4;  //SD card

Adafruit_BMP280 bmp; // I2C



//GPS
int RXPin = 2;                          // Указываем вывод подключения RX  
int TXPin = 3;                          // Указываем вывод подключения TX
int GPSBaud = 9600;                     // Указываем скорость передачи с NEO-6M
TinyGPSPlus gps;                        // Создание объекта 
SoftwareSerial gpsSerial(RXPin, TXPin); // Создайте последовательную связь под названием "gpsSerial"




void setup() {
  Serial.begin(9600);
  Serial.println(F("BMP280 init"));
  unsigned status;
  status = bmp.begin(BMP280_ADDRESS_ALT, BMP280_CHIPID);
 
  /* Default settings from datasheet. */
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */
          

          
gpsSerial.begin(GPSBaud);             // Запустите последовательный порт с NEO-6M на скорости 9600          


//SD card
 // Этот пин обязательно должен быть определен как OUTPUT ????????????????????
  //pinMode(10, OUTPUT);

  // Пытаемся проинициализировать модуль
  if (!SD.begin(PIN_CHIP_SELECT)) {
    //Serial.println("Card failed, or not present");
    // Если что-то пошло не так, завершаем работу:
    return;
  }
  //Serial.println("card initialized.");

}




/*
#include <SPI.h>  // Подключаем библиотеку для работы с SPI-интерфейсом
#include <nRF24L01.h> // Подключаем файл конфигурации из библиотеки RF24
#include <RF24.h> // Подключаем библиотеку для работа для работы с модулем NRF24L01

#define PIN_CE  7  // Номер пина Arduino, к которому подключен вывод CE радиомодуля
#define PIN_CSN 8 // Номер пина Arduino, к которому подключен вывод CSN радиомодуля
RF24 radio(PIN_CE, PIN_CSN); // Создаём объект radio с указанием выводов CE и CSN

int potValue[1]; // Создаём массив для приёма значений потенциометра

void RF_reciever_setup() {

  radio.begin();  // Инициализация модуля NRF24L01
  radio.setChannel(5); // Обмен данными будет вестись на пятом канале (2,405 ГГц)
  radio.setDataRate (RF24_1MBPS); // Скорость обмена данными 1 Мбит/сек
  radio.setPALevel(RF24_PA_HIGH); // Выбираем высокую мощность передатчика (-6dBm)
  radio.openReadingPipe (1, 0x7878787878LL); // Открываем трубу ID передатчика
  radio.startListening(); // Начинаем прослушивать открываемую трубу
}

int BuzzerPin = 5;  //Need to check for PWM
int RF_MachineState = 0; // Rocket state from RF transmitter. Default = 0 - Not Ready for start;
*/
//void init(){
  //RF_init();
  
  //Gyro_init();
  //Servo_init();
  
  //BMP280_init();
  //GPS_init();
  //SD_init(); //https://arduinomaster.ru/datchiki-arduino/podklyuchenie-sd-karty-k-arduino/
  //pinMode(BuzzerPin, OUTPUT); //Buzzer init
//}


void loop() {
  //Stabilization system code section
  
  
  
  //RF control system code section
  /*if(radio.available()){
    radio.read(&RF_MachineState, sizeof(RF_MachineState)); // Read rocket state from RF transmitter
  }
  
  //Rocket rescue system code section
    if (RF_MachineState == 1) {   //Rocket is ready for start
      //turn on Buzzer
      //digitalWrite(13, HIGH)
      analogWrite(BuzzerPin,50);
      delay(1000);
      analogWrite(BuzzerPin,0);
      delay(1000);
    }*/
    
  //Data logging system code section
    //get GPS data
    /*while (gpsSerial.available() > 0)
    if (gps.encode(gpsSerial.read()))

    //Если нет данных в течении 5000 миллисекунд, пишем сообщение об ошибки
    if (millis() > 5000 && gps.charsProcessed() < 10) {
      Serial.println("No GPS detected");
    }
    
    /*double GPS_latitude = 0;
    double GPS_longitude = 0;
    double GPS_altitude = 0;*/
    /*uint8_t GPS_time_hour = 0;
    uint8_t GPS_time_minute = 0;
    uint8_t GPS_time_second = 0;
    uint8_t GPS_time_centisecond = 0;*/
    
    /*if (gps.location.isValid()){
      GPS_latitude = gps.location.lat();
      GPS_longitude = gps.location.lng();
      GPS_altitude = gps.altitude.meters();
    }
    else {
      Serial.println("Location: Not Available");
    }*/
    
    /*if (gps.time.isValid()) {
      GPS_time_hour = gps.time.hour();
      GPS_time_minute = gps.time.minute();
      GPS_time_second = gps.time.second();
      GPS_time_centisecond = gps.time.centisecond();
      
      //For Debug without microSD

      Serial.print("Time: ");
      Serial.print(String(GPS_time_hour)); 
      Serial.print(":");
      Serial.print(String(GPS_time_minute)); 
      Serial.print(":");
      Serial.println(String(GPS_time_second)); 
            
      
    }
    else {
    Serial.println("Time: Not Available");
    }*/
  /*  
  Serial.print(F("Temperature = "));
    Serial.print(bmp.readTemperature());
    Serial.println(" *C");

    Serial.print(F("Pressure = "));
    Serial.print(bmp.readPressure());
    Serial.println(" Pa");

    Serial.print(F("Approx altitude = "));
    Serial.print(bmp.readAltitude(1013.25)); /* Adjusted to local forecast! */
    /*Serial.println(" m");
  

    Serial.println();*/
    delay(2000);
    //get BMP280 data
    /*float BMP280_temp = bmp.readTemperature();  // todo: Add SMA filter to this function in extenal file
    float BMP280_pressure = bmp.readPressure();
    float BMP280_altitude = bmp.readAltitude(1013.25); /* Adjusted to local forecast! */




    //write data to SD card 
    String logStringData = "";  //String for SD card data logging
    
    //Write data to the string
    //BMP280 data
    logStringData += "Temperature = ";
    logStringData += String(bmp.readTemperature());
    logStringData += " *C; ";
    
    logStringData += "Pressure = ";
    logStringData += String(bmp.readPressure());
    logStringData += " Pa; ";
    
    logStringData += "BMP280 altitude = ";
    logStringData += String(bmp.readAltitude(1013.25));
    logStringData += " m; ";    
    //GPS data
    /*logStringData += "GPS latitude = ";
    logStringData += String(GPS_latitude);
    logStringData += " ; "; 
    
    logStringData += "GPS longitude = ";
    logStringData += String(GPS_longitude);
    logStringData += " ; ";     
    
    logStringData += "GPS_altitude = ";
    logStringData += String(GPS_altitude);
    logStringData += " ; "; */

      
    if (gps.time.isValid()) {
      logStringData += "Time: ";
      logStringData += String(gps.time.hour());
      logStringData += ":"; 
      logStringData += String(gps.time.minute());
      logStringData += ":"; 
      logStringData += String(gps.time.second());
      logStringData += ":"; 
      logStringData += String(gps.time.centisecond());
      logStringData += "/n";          
    }
    else {
    logStringData += "Time: Not Available";
    }
    
    



    File dataFile = SD.open("DataLog.txt", FILE_WRITE); //Arduino will create file. Only one file can be opened.

    if (dataFile) {   //If everything is OK, then write data
      dataFile.println(logStringData);
      dataFile.close();
    }
    /*else {
      Serial.println("Error opening DataLog.txt");  //todo: Add RF message
    }*/

  delay(5000);
}

//Перейти на библиотеку с отслеживанием активных спутников
//https://github.com/tremaru/iarduino_GPS_NMEA
//https://3d-diy.ru/wiki/arduino-moduli/gps-modul-neo-6m/

/*
float SMA_readTemperature() {
 
  uint8_t n = 4;  //Window size
  float average = 0;

  float data1Value = bmp.readTemperature();
  float data_array[n-1] = [data1Value data1Value data1Value data1Value];  //Window size = 4
  uint8_t data_array_index = 1;

  data_array[data_array_index] = bmp.readTemperature();
     
  if (data_array_index < (n-1))
    data_array_index = data_array_index + 1;
  else
    data_array_index = 0;

  average = (data_array[0] + data_array[1] + data_array[2] + data_array[3]) / n;  //Window size = 4
  return (average)
}
*/
