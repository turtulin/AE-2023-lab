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
#define myPortInputRegister(port) ( (volatile uint8_t *)( __LPM_word_enhanced__( port_to_input_PGM + (port))) )

float temperature = 0;
float humidity = 0;
//uint32_t myMillis = 0;
uint8_t myBit = (1 << DHT22_PIN);

int myDigitalRead(uint8_t pin) {
	if (*myPortInputRegister(PORTD) & myBit) return HIGH;
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

  //alternativa al for
  cli();
  for(uint8_t i=0; i<5; i++) {
    for(uint8_t j=0; j<8; j++) {
      while(!myDigitalRead(DHT22_PIN));      
      delayMicroseconds(50);

      if(myDigitalRead(DHT22_PIN)) data[i] = (data[i]<<1)|(0x01);
      else data[i] = (data[i]<<1);  

      while(myDigitalRead(DHT22_PIN));   
    }
  }
  sei();

  if(data[4] == ((data[0] + data[1] + data[2] + data[3])&0xFF)) {
    humidity  = ((data[0] << 8) | data[1])/10.0;
    temperature = ((data[2] & 0x7F) << 8 | data[3])/10.0;
    if (data[2] & 0x80) {
      temperature = -temperature;
    }
  }

  Serial.print("Temperature: ");
  Serial.println(temperature);
  Serial.print("Humidity: ");
  Serial.println(humidity);
}
