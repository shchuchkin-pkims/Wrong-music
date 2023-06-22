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

//Pinout define
#define BMP_SCK  (13)
#define BMP_MISO (12)
#define BMP_MOSI (11)
#define BMP_CS   (10)


#include <Wire.h>
#include <SPI.h>
#include <Adafruit_BMP280.h>

Adafruit_BMP280 bmp; // I2C

void BMP280_setup() {
  Serial.begin(9600);
  Serial.println(F("BMP280 init"));
  unsigned status;
  status = bmp.begin(BMP280_ADDRESS_ALT, BMP280_CHIPID);
  if (!status) {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring or "
                      "try a different address!"));
    Serial.print("SensorID was: 0x"); 
	Serial.println(bmp.sensorID(),16);
    while (1) delay(10);
  }
  /* Default settings from datasheet. */
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */
}

#include <TinyGPS++.h>                  // Подключение библиотеки TinyGPS
#include <SoftwareSerial.h>             // Подключение библиотеки SoftwareSerial

//GPS
int RXPin = 2;                          // Указываем вывод подключения RX  
int TXPin = 3;                          // Указываем вывод подключения TX
int GPSBaud = 9600;                     // Указываем скорость передачи с NEO-6M
TinyGPSPlus gps;                        // Создание объекта 
SoftwareSerial gpsSerial(RXPin, TXPin); // Создайте последовательную связь под названием "gpsSerial"

void GPS_setup(){
	gpsSerial.begin(GPSBaud);             // Запустите последовательный порт с NEO-6M на скорости 9600
}

#include <SPI.h>
#include <SD.h>
const int PIN_CHIP_SELECT = 4;

void SD_setup() {
 // Этот пин обязательно должен быть определен как OUTPUT ????????????????????
  pinMode(10, OUTPUT);

  // Пытаемся проинициализировать модуль
  if (!SD.begin(PIN_CHIP_SELECT)) {
    Serial.println("Card failed, or not present");
    // Если что-то пошло не так, завершаем работу:
    return;
  }
  Serial.println("card initialized.");
}

#include <SPI.h>  // Подключаем библиотеку для работы с SPI-интерфейсом
#include <nRF24L01.h> // Подключаем файл конфигурации из библиотеки RF24
#include <RF24.h> // Подключаем библиотеку для работа для работы с модулем NRF24L01

#define PIN_CE  7  // Номер пина Arduino, к которому подключен вывод CE радиомодуля
#define PIN_CSN 8 // Номер пина Arduino, к которому подключен вывод CSN радиомодуля
RF24 radio(PIN_CE, PIN_CSN); // Создаём объект radio с указанием выводов CE и CSN

int potValue[1]; // Создаём массив для приёма значений потенциометра

void RF_reciever_setup() {
  pinMode(PIN_LED, OUTPUT); // Настраиваем на выход пин светодиода
  radio.begin();  // Инициализация модуля NRF24L01
  radio.setChannel(5); // Обмен данными будет вестись на пятом канале (2,405 ГГц)
  radio.setDataRate (RF24_1MBPS); // Скорость обмена данными 1 Мбит/сек
  radio.setPALevel(RF24_PA_HIGH); // Выбираем высокую мощность передатчика (-6dBm)
  radio.openReadingPipe (1, 0x7878787878LL); // Открываем трубу ID передатчика
  radio.startListening(); // Начинаем прослушивать открываемую трубу
}

int BuzzerPin = 5;	//Need to check for PWM
int RF_MachineState = 0; // Rocket state from RF transmitter. Default = 0 - Not Ready for start;

void init(){
	//RF_init();
	
	//Gyro_init();
	//Servo_init();
	
	//BMP280_init();
	//GPS_init();
	//SD_init(); //https://arduinomaster.ru/datchiki-arduino/podklyuchenie-sd-karty-k-arduino/
	pinMode(BuzzerPin, OUTPUT);	//Buzzer init
}


void loop() {
	//Stabilization system code section
	
	
	
	//RF control system code section
	if(radio.available()){
		radio.read(&RF_MachineState, sizeof(RF_MachineState)); // Read rocket state from RF transmitter
	}
	
	//Rocket rescue system code section
		if (RF_MachineState == 1) {		//Rocket is ready for start
			//turn on Buzzer
			//digitalWrite(13, HIGH)
			analogWrite(BuzzerPin,50);
			delay(1000);
			analogWrite(BuzzerPin,0);
			delay(1000);
		}
		
	//Data logging system code section
		//get GPS data
		if (gpsSerial.available() > 0)
		if (gps.encode(gpsSerial.read()))

		//Если нет данных в течении 5000 миллисекунд, пишем сообщение об ошибки
		if (millis() > 5000 && gps.charsProcessed() < 10) {
			Serial.println("No GPS detected");
		}
		
		{
		if (gps.location.isValid()){
		double GPS_latitude = gps.location.lat();
		double GPS_longitude = gps.location.lng();
		double GPS_altitude = gps.altitude.meters();
		}
		else {
			Serial.println("Location: Not Available");
		}
		
		if (gps.time.isValid()) {
		uint8_t GPS_time_hour = gps.time.hour();
		uint8_t GPS_time_minute = gps.time.minute();
		uint8_t GPS_time_second = gps.time.second();
		uint8_t GPS_time_centisecond = gps.time.centisecond();
		}
		else {
		Serial.println("Not Available");
		}
		
	
		//get BMP280 data
		float BMP280_temp = bmp.readTemperature();	// todo: Add SMA filter to this function in extenal file
		float BMP280_pressure = bmp.readPressure();
		float BMP280_altitude = bmp.readAltitude(1013.25); /* Adjusted to local forecast! */

		//For Debug without microSD
/*********
Serial.print("Temperature = ");
Serial.println(string(BMP280_temp)); 

Serial.print("Time: ");
Serial.print(string(GPS_time_hour)); 
Serial.print(":");
Serial.print(string(GPS_time_minute)); 
Serial.print(":");
Serial.println(string(GPS_time_second)); 
*********/

		//write data to SD card	
		String logStringData = "";	//String for SD card data logging
		
		//Write data to the string
		//BMP280 data
		logStringData += "Temperature = ";
		logStringData += string(BMP280_temp);
		logStringData += " *C; ";
		
		logStringData += "Pressure = ";
		logStringData += string(BMP280_pressure);
		logStringData += " Pa; ";
		
		logStringData += "BMP280 altitude = ";
		logStringData += string(BMP280_altitude);
		logStringData += " m; ";		
		//GPS data
		logStringData += "GPS latitude = ";
		logStringData += string(GPS_latitude);
		logStringData += " ; ";	
		
		logStringData += "GPS longitude = ";
		logStringData += string(GPS_longitude);
		logStringData += " ; ";			
		
		logStringData += "GPS_altitude = ";
		logStringData += string(GPS_altitude);
		logStringData += " ; ";	
		
		logStringData += "Time: ";
		logStringData += string(GPS_time_hour);
		logStringData += ":";	
		logStringData += string(GPS_time_minute);
		logStringData += ":";	
		logStringData += string(GPS_time_second);
		logStringData += ":";	
		logStringData += string(GPS_time_centisecond);
		logStringData += "/n";	



		File dataFile = SD.open("DataLog.csv", FILE_WRITE); //Arduino will create file. Only one file can be opened.

		if (dataFile) {		//If everything is OK, then write data
			dataFile.println(logStringData);
			dataFile.close();
		}
		else {
			Serial.println("Error opening DataLog.csv");	//todo: Add RF message
		}

	delay(5000);
}

//Перейти на библиотеку с отслеживанием активных спутников
//https://github.com/tremaru/iarduino_GPS_NMEA
//https://3d-diy.ru/wiki/arduino-moduli/gps-modul-neo-6m/

/*
float SMA_readTemperature() {
 
	uint8_t n = 4;	//Window size
	float average = 0;

	float data1Value = bmp.readTemperature();
	float data_array[n-1] = [data1Value data1Value data1Value data1Value];	//Window size = 4
	uint8_t data_array_index = 1;

	data_array[data_array_index] = bmp.readTemperature();
	   
	if (data_array_index < (n-1))
		data_array_index = data_array_index + 1;
	else
		data_array_index = 0;

	average = (data_array[0] + data_array[1] + data_array[2] + data_array[3]) / n;	//Window size = 4
	return (average)
}
*/