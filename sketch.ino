/*
#define MICROSECONDS_PER_TIMER0_OVERFLOW (clockCyclesToMicroseconds(64 * 256))
#define MILLIS_INC (MICROSECONDS_PER_TIMER0_OVERFLOW / 1000)
#define FRACT_INC ((MICROSECONDS_PER_TIMER0_OVERFLOW % 1000) >> 3)
#define FRACT_MAX (1000 >> 3)

extern volatile unsigned long timer0_overflow_count;
extern volatile unsigned long timer0_millis;
unsigned char timer0_fract;
*/

#include "wiring_private.h"

#define DHT22_PIN 2
#define myDigitalWrite(n,level) (level == HIGH? (PORTD |= myBit) : (PORTD &= ~myBit))
#define myPinMode(n,mode) (mode == OUTPUT? (DDRD |= myBit) : (DDRD &= ~myBit)); \ 
(mode == INPUT_PULLUP? (PORTD |= (1<<n)) : (NULL))

float temperature = 0;
float humidity = 0;
//uint32_t myMillis = 0;
uint8_t myBit = (1 << DHT22_PIN);

int myDigitalRead(uint8_t pin) {
	if (*portInputRegister(PORTD) & myBit) return HIGH;
	return LOW;
}

void setup() {
  Serial.begin(9600);
  //TIMSK0 = 0x4;
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

  data[0] = readDHT22byte();       
  data[1] = readDHT22byte();        
  data[2] = readDHT22byte();
  data[3] = readDHT22byte();      
  data[4] = readDHT22byte();     


  humidity  = (data[0] << 8) | data[1];
  temperature = (data[2] & 0x7F) << 8 | data[3];
  if (data[2] & 0x80) {
    temperature = -temperature;
  }


  Serial.println(humidity/10.0);
  Serial.println(temperature/10.0);
}

uint8_t readDHT22byte() {
  uint8_t dataByte;
  for(uint8_t i=0; i<8; i++) {
    while(!myDigitalRead(DHT22_PIN));      
    delayMicroseconds(50);

    if(myDigitalRead(DHT22_PIN)) dataByte = (dataByte<<1)|(0x01);
    else dataByte = (dataByte<<1);  

    while(myDigitalRead(DHT22_PIN));   
  }
  return dataByte;
}

/*
ISR(TIMERO_COMPB_vect) {
  myMillis++;
	
  unsigned long m = timer0_millis;
	unsigned char f = timer0_fract;

	m += MILLIS_INC;
	f += FRACT_INC;
	if (f >= FRACT_MAX) {
		f -= FRACT_MAX;
		m += 1;
	}

	timer0_fract = f;
	timer0_millis = m;
	timer0_overflow_count++;
}
*/