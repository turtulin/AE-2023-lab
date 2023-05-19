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
  
  DDRD |= (1 << pin);     //output
  PORTD &= ~(1 << pin);   //low
  _delay_ms(1);
  PORTD |= (1 << pin);
  DDRD &= ~(1 << pin);
  _delay_us(40);
  
  if (PIND & (1 << pin)) {
    Serial.println("No response from sensor");
    return;
  }

  _delay_us(80);
  
  if (!(PIND & (1 << pin))) {
    Serial.println("No response from sensor");
    return;
  }
  
  TCCR1A = 0;
  TCCR1B = (1 << CS11);
  
  for (int i = 0; i < (80); i +=2) {
    while (!(PIND & (1 << pin)));
    TCNT1 = 0;
    while (PIND & (1 << pin));
    uint16_t pulseLength = TCNT1;
    
    if (pulseLength > (F_CPU / (1000000 *8) *80)) {
      data[byteIndex] |= (1 << bitIndex);
    }
    
    if (bitIndex == 0) {
      bitIndex =7;
      byteIndex++;
    } else {
      bitIndex--;
    }
   }
   
   TCCR1B =0;
   
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


/*void myPinMode(uint8_t pin, uint8_t mode) {
  if (mode == INPUT) {
    *ddrReg &= ~(1 << bitNum);
  } else if (mode == OUTPUT) {
    *ddrReg |= (1 << bitNum);
  }
}

uint8_t myDigitalRead(uint8_t pin) {
  pinState = (*pinReg) & (1 << bitNum);
  return (pinState > 0) ? HIGH : LOW;
}

void myDigitalWrite(uint8_t pin, uint8_t value) {
  if (value == LOW) {
    *portReg &= ~(1 << bitNum);
  } else if (value == HIGH) {
    *portReg |= (1 << bitNum);
  }


  #define DHTPIN 2 // Pin connected to the DHT22 sensor
#define DHTTYPE DHT22 // DHT22 sensor
*/