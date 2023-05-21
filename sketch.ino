#include "wiring_private.h"

#define DHT22_PIN 2
#define myDigitalWrite(n,level) (level == HIGH? (PORTD |= myBit) : (PORTD &= ~myBit))
#define myPinMode(n,mode) (mode == OUTPUT? (DDRD |= myBit) : (DDRD &= ~myBit)); \ 
(mode == INPUT_PULLUP? (PORTD |= (1<<n)) : (NULL))

float temperature = 0;
float humidity = 0;
uint8_t myBit = (1 << DHT22_PIN);

int myDigitalRead(uint8_t pin) {
	if (*portInputRegister(PORTD) & myBit) return HIGH;
	return LOW;
}

void setup() {
  Serial.begin(9600);
}

void loop() {
  delay(3000);
  readDHT22(DHT22_PIN);
}


void readDHT22(uint8_t pin) {
  uint8_t data[5] = {0, 0, 0, 0, 0};

  myPinMode(pin, OUTPUT);             
  myDigitalWrite(pin, LOW);           
  delayMicroseconds(1000);           
  myDigitalWrite(pin, HIGH);          
  delayMicroseconds(40);             

  myPinMode(pin, INPUT);              
  while(myDigitalRead(pin));          
  while(!myDigitalRead(pin));         
  while(myDigitalRead(pin));      

  for(uint8_t i=0; i<5; i++) {
    for(uint8_t j=0; j<8; j++) {
      while(!myDigitalRead(DHT22_PIN));      
      delayMicroseconds(50);

      if(myDigitalRead(DHT22_PIN)) data[i] = (data[i]<<1)|(0x01);
      else data[i] = (data[i]<<1);  

      while(myDigitalRead(DHT22_PIN));   
    }
  }

  humidity  = ((data[0] << 8) | data[1])/10.0;
  temperature = ((data[2] & 0x7F) << 8 | data[3])/10.0;
  if (data[2] & 0x80) {
    temperature = -temperature;
  }

  Serial.println(humidity);
  Serial.println(temperature);
}