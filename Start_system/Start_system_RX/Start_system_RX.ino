/*   Обрабатываем посылки receiv_data
0 байт - ключ шифрования crypt_key
1 байт - ключ запуска start_key
*/

#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"

float my_vcc_const = 1.1;	// начальное значение константы должно быть 1.1

uint8_t LED_bat = 5;  			// пин, к которому подключен LED battery D5
uint8_t MOSFET = 4;  			// пин, к которому подключен мосфет D4
uint8_t protector = 7;			// pin for safety start D7
uint8_t battery_pin = 6;    	// пин, подключен аккумулятор для измерения напряжения A6

boolean FLAG = 0;			// массив, хранящий состояние мосфетов
unsigned long TIME; 		// массив, хранящий время работы мосфетов

boolean RXstate = 0;

byte receiv_data[2];

int fuse_time = 1500; 		// время в миллисекундах, которое ток будет подаваться  на спираль
int battery_check = 2800;	// нижняя граница заряда аккумулятора для защиты, в милливольтах!
int voltage = 0;			// измеренное значение напряжения

byte crypt_key = 123;   	// уникальный ключ для защиты связи
int start_key = 321;        // условный код, для пуска

RF24 radio(9, 10); 			// "создать" модуль на пинах 9 и 10 Для Уно


byte address[][6] = {"1Node", "2Node", "3Node", "4Node", "5Node", "6Node"}; //возможные номера труб

void setup() {
  Serial.begin(9600); 			//открываем порт для связи с ПК

  pinMode(MOSFET, OUTPUT);		// настраиваем пин мосфет как выход
  digitalWrite(MOSFET, LOW); 	// turn off MOSFET
  pinMode(LED_bat, OUTPUT);		// настраиваем пин LED как выход
  digitalWrite(LED_bat, LOW); 	// turn off LED
  pinMode(protector, INPUT);	// настраиваем пин protector как вход
  
  radio.begin(); 				//активировать модуль
  radio.setAutoAck(1);          //режим подтверждения приёма, 1 вкл 0 выкл
  radio.setRetries(0, 15);		//(время между попыткой достучаться, число попыток)
  radio.enableAckPayload();		//разрешить отсылку данных в ответ на входящий сигнал
  radio.setPayloadSize(32);     //размер пакета, в байтах

  radio.openReadingPipe(1, address[0]);     //хотим слушать трубу 0
  radio.setChannel(0x60);  					//выбираем канал (в котором нет шумов!)

  radio.setPALevel (RF24_PA_MAX); 			//уровень мощности передатчика. На выбор RF24_PA_MIN, RF24_PA_LOW, RF24_PA_HIGH, RF24_PA_MAX
  radio.setDataRate (RF24_250KBPS);			//скорость обмена. На выбор RF24_2MBPS, RF24_1MBPS, RF24_250KBPS
  
  radio.powerUp(); 							//начать работу
  radio.startListening();  					//начинаем слушать эфир, мы приёмный модуль
}

void loop() {
	byte pipeNo;
	if (digitalRead(protector) == HIGH) {
		voltage = analogRead(battery_pin) * readVcc() / 1024;
		if (voltage > battery_check) {
			RXstate = 1;					// Battery charge is OK
			digitalWrite(LED_bat, HIGH); 	// turn on LED
			}
		else {
			RXstate = 0;					// Battery charge is low
			digitalWrite(LED_bat, LOW); 	// turn off LED
			}
		while (radio.available(&pipeNo)) {             			// слушаем эфир со всех труб
			radio.read( &receiv_data, sizeof(receiv_data) );		// читаем входящий сигнал
			if (receiv_data[0] == crypt_key) {
				radio.writeAckPayload(pipeNo, &RXstate, sizeof(RXstate) );
				Serial.println("RX battery state sent");
				if (RXstate == 1 && receiv_data[1] == start_key)
					if (FLAG == 0) {
						FLAG = 1;                           // поднять флаг для мосфета, по входящему сигналу
						TIME = millis();                    // запомнить время прихода сигнала
						digitalWrite(MOSFET, HIGH);         // подать питание на мосфет (на запал)
						Serial.print("Fuse ON");
					}
			}
		}
	 
		if (millis() - TIME > fuse_time && FLAG == 1) {  				// если время с момента открытия мосфета > заданного
			FLAG = 0;                                            		// опустить флаг
			digitalWrite(MOSFET, LOW);                                  // закрыть мосфет, погасить запал
			Serial.print("Fuse OFF");
		} 
	}
}

long readVcc() { //функция чтения внутреннего опорного напряжения, универсальная (для всех ардуин)
#if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
  ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
#elif defined (__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
  ADMUX = _BV(MUX5) | _BV(MUX0);
#elif defined (__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
  ADMUX = _BV(MUX3) | _BV(MUX2);
#else
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
#endif
  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Start conversion
  while (bit_is_set(ADCSRA, ADSC)); // measuring
  uint8_t low  = ADCL; // must read ADCL first - it then locks ADCH
  uint8_t high = ADCH; // unlocks both
  long result = (high << 8) | low;

  result = my_vcc_const * 1023 * 1000 / result; // расчёт реального VCC
  return result; // возвращает VCC
}
