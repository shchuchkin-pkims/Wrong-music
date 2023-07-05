/*
  Пульт передатчика. Считываем статус-заряд батарейки приемника. Если все ОК и нажата кнопка ПУСК, то посылаем команду на зажигание.
*/
#include <SPI.h>         	// библиотека для работы с шиной SPI
#include "nRF24L01.h"   	// библиотека радиомодуля
#include "RF24.h"      		// ещё библиотека радиомодуля

RF24 radio(9, 10); 			// "создать" модуль на пинах 9 и 10 Для Уно
byte address[][6] = {"1Node", "2Node", "3Node", "4Node", "5Node", "6Node"}; //возможные номера труб

uint8_t LED_RF = 4;			// LED for RF connection - pin D4
uint8_t START_BTN = 5;		// Button for ignition start - pin D5


byte crypt_key = 123;   				// уникальный ключ для защиты связи
int start_key = 321;       				// условный код, для пуска
byte transm_data[2];    				// массив отправляемых данных (уникальный ключ + условный код, для пуска)
boolean check_answer;


boolean flag = 0;

void setup() {
  Serial.begin(9600);

  transm_data[0] = crypt_key;     		// задать уникальный ключ для защиты связи

  pinMode(LED_RF, OUTPUT);
  digitalWrite(LED_RF, LOW);    		// turn off LED_RF

  radio.begin(); 						//активировать модуль
  radio.setAutoAck(1);        			//режим подтверждения приёма, 1 вкл 0 выкл
  radio.setRetries(0, 15);				//(время между попыткой достучаться, число попыток)
  radio.enableAckPayload();				//разрешить отсылку данных в ответ на входящий сигнал
  radio.setPayloadSize(32);				//размер пакета, в байтах

  radio.openWritingPipe(address[0]);  	//мы - труба 0, открываем канал для передачи данных
  radio.setChannel(0x60);  				//выбираем канал (в котором нет шумов!)

  radio.setPALevel (RF24_PA_MAX); 		//уровень мощности передатчика. На выбор RF24_PA_MIN, RF24_PA_LOW, RF24_PA_HIGH, RF24_PA_MAX
  radio.setDataRate (RF24_250KBPS); 	//скорость обмена. На выбор RF24_2MBPS, RF24_1MBPS, RF24_250KBPS

  radio.powerUp(); 						//начать работу
  radio.stopListening();  				//не слушаем радиоэфир, мы передатчик
}

void loop() {
	transm_data[1] = 000;										// Just testing connection. No start ignition.
    if (radio.write(&transm_data, sizeof(transm_data)) ) {  	// отправляем 
		if ( !radio.available() ) {                           	// если не получаем ответ
			Serial.println("No answer");
			digitalWrite(LED_RF, LOW);                     		// выключить зелёный светодиод
		} 
		else {
			while ( radio.available() ) {                      	// если в ответе что-то есть
				radio.read( &check_answer, 1);                  // читаем
				if (check_answer == 1) {                        // если статус = 1 (готов к работе)
					flag = 1;                                	// поднять флаг готовности к работе
					digitalWrite(LED_RF, HIGH);              	// зажечь красный светодиод
					Serial.println("Status OK");
				} 
				else {                                        	// если статус = 0 (акум разряжен)
					flag = 0;
					Serial.println("Status BAD");
					for (int i = 0; i < 5; i++) {            	// моргнуть 5 раз зелёным светодиодом
						digitalWrite(LED_RF, HIGH);
						delay(200);
						digitalWrite(LED_RF, LOW);
						delay(200);
					}
				}
			}
		}
    } 
	else {
      Serial.println("Sending failed");              			// ошибка отправки
      digitalWrite(LED_RF, LOW);                      			// выключить зелёный светодиод
    }

	
	if (digitalRead(START_BTN) == HIGH && flag == 1) {		 	// если нажали START_BTN и флаг поднят
		transm_data[1] = start_key;                         	// на второе место массива ставим start_key
		radio.write(&transm_data, sizeof(transm_data));     	// отправить массив по радио
		delay(100);
	}
	
}
