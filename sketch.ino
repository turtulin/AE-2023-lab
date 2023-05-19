#define DHT22_PIN 2

#include "Arduino.h"

float temperature = 0;
float humidity = 0;

void setup() {
  Serial.begin(9600);
}

void loop() {
  delay(2000);
  readDHT22(DHT22_PIN);
  delay(2000);
  readDHT22(DHT22_PIN);
}

void readDHT22(uint8_t pin) {
  uint8_t data[5] = {0, 0, 0, 0, 0};
  uint8_t bitIndex = 7;
  uint8_t byteIndex = 0;
  
  DDRD |= (1 << pin);
  PORTD &= ~(1 << pin);
  delayMicroseconds(1000);
  PORTD |= (1 << pin);
  DDRD &= ~(1 << pin);
  delayMicroseconds(40);
  
  if (PIND & (1 << pin)) {
    Serial.println("No response from sensor");
    return;
  }

  delayMicroseconds(80);
  
  if (!(PIND & (1 << pin))) {
    Serial.println("No response from sensor");
    return;
  }
  
  for(int i = 0; i < (40); i++) {
    while (!(PIND & (1 << pin)));
    uint32_t startMicros = micros();
    while (PIND & (1 << pin));
    uint16_t durationHigh = micros() - startMicros;

    if(durationHigh > 48) {
      data[byteIndex] |= (1 << bitIndex);
    }

    if(bitIndex == 0) {
      bitIndex = 7;
      byteIndex++;
    }
    else {
      bitIndex--;
    }
  }
   
   uint8_t checksum = data[0] + data[1] + data[2] + data[3];
   
   if (checksum != data[4]) {
     Serial.println("Checksum error");
     return;
   }
   
   humidity = ((data[0] << 8) | data[1]) / 10.0;
   temperature = (((data[2] & 0x7F) << 8) | data[3]) / 10.0;
   
   if(data[2] & 0x80)
   temperature = -temperature;

   
   Serial.print("Humidity: ");
   Serial.print(humidity);
   Serial.print("%\t");
   Serial.print("Temperature: ");
   Serial.print(temperature);
   Serial.println("°C");
}
