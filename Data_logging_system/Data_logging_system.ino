/*********
  Shchuchkin Evgeny
  
  First version of "Wrong Music" rocket data logging controller program
*********/
#include <Wire.h>						// BMP280 lib section
#include <SPI.h>						//
#include <Adafruit_BMP280.h>			//
#include <TinyGPS++.h>                  // GPS lib section
#include <SoftwareSerial.h>             // 
#include <SD.h>							// SD card lib section

#define SD_nCS  4						// SD card nCS pin

#define RXPin 2							//GPS
#define TXPin 3							//
#define GPSBaud 9600					//
// BMP280
Adafruit_BMP280 bmp;
// GPS
TinyGPSPlus gps;                       
SoftwareSerial gpsSerial(RXPin, TXPin);

void setup() {
	Serial.begin(9600);
	// BMP280 init
	unsigned status;
	status = bmp.begin(BMP280_ADDRESS_ALT, BMP280_CHIPID);
 
	bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,	// Operating Mode
                  Adafruit_BMP280::SAMPLING_X2,     // Temp. oversampling
                  Adafruit_BMP280::SAMPLING_X16,    // Pressure oversampling
                  Adafruit_BMP280::FILTER_X16,      // Filtering
                  Adafruit_BMP280::STANDBY_MS_500); // Standby time
	// GPS init     
	gpsSerial.begin(GPSBaud);             // GPS port for NEO-6M at 9600 baud rate

	//SD card init
	if (!SD.begin(SD_nCS)) {
		//Serial.println("Card failed, or not present");
		return;
	}
	Serial.println("Data logging system initialized.");
}

void loop() {
	//Data logging system code section
    String logStringData = ""; 								// String for SD card data logging
    File dataFile = SD.open("DataLog.txt", FILE_WRITE);		// Arduino will create file. Only one file can be opened.

    if (dataFile) {   // If everything is OK, then write data to string
		//BMP280 data
		logStringData = "Temperature = ";
		logStringData += String(bmp.readTemperature());
		logStringData += " *C; ";
		dataFile.print(logStringData);
		
		logStringData = "Pressure = ";
		logStringData += String(bmp.readPressure());
		logStringData += " Pa; ";
		dataFile.print(logStringData);
		
		logStringData = "BMP280 altitude = ";
		logStringData += String(bmp.readAltitude(1013.25));	// Adjusted to local forecast
		logStringData += " m; ";   	
		dataFile.print(logStringData);
		
		//GPS data
		if (gps.encode(gpsSerial.read())) {
			if (gps.location.isValid()){
				logStringData = "GPS latitude = ";
				logStringData += String(gps.location.lat());
				logStringData += " ; ";
				dataFile.print(logStringData);
				
				logStringData = "GPS longitude = ";
				logStringData += String(gps.location.lng());
				logStringData += " ; ";
				dataFile.print(logStringData);
				
				logStringData = "GPS GPS_altitude = ";
				logStringData += String(gps.altitude.meters());
				logStringData += " ; ";
				dataFile.print(logStringData);
			}
			else {
				logStringData = "Location: Not Available ";
				dataFile.print(logStringData);
			}
			
			if (gps.time.isValid()) {
				logStringData = "Time: ";
				logStringData += String(gps.time.hour());
				logStringData += ":"; 
				logStringData += String(gps.time.minute());
				logStringData += ":"; 
				logStringData += String(gps.time.second());
				logStringData += ":"; 
				logStringData += String(gps.time.centisecond());        
			}
			else {
				logStringData = "Time: Not Available";
			}
		}
		dataFile.println(logStringData);
    }
	dataFile.close();
	delay(5000);
}
// todo:
// Change TinyGPS++ lib for GPS NMEA for active satellites number logging
// https://github.com/tremaru/iarduino_GPS_NMEA
// https://3d-diy.ru/wiki/arduino-moduli/gps-modul-neo-6m/
