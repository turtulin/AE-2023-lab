/*
#include "wiring_private.h"

#define MICROSECONDS_PER_TIMER0_OVERFLOW (clockCyclesToMicroseconds(64 * 256))
#define MILLIS_INC (MICROSECONDS_PER_TIMER0_OVERFLOW / 1000)
#define FRACT_INC ((MICROSECONDS_PER_TIMER0_OVERFLOW % 1000) >> 3)
#define FRACT_MAX (1000 >> 3)

extern volatile unsigned long timer0_overflow_count;
extern volatile unsigned long timer0_millis;
unsigned char timer0_fract;
*/

#define DHT22_PIN 2

//to obtain more generic functions, I could add controls to include PORTB and PORTC
//pass bitmask as argument
#define myDigitalWrite(n,level) (level == HIGH? (PORTD |= n) : (PORTD &= ~n))
#define myPinMode(n,mode) (mode == OUTPUT? (DDRD |= n) : (DDRD &= ~n)); \ 
(mode == INPUT_PULLUP? (PORTD |= (1 << n)) : (NULL))
#define myPortInputRegister(port) ((volatile uint8_t *)(__LPM_word_enhanced__(port_to_input_PGM + (port))))

int myDigitalRead(uint8_t n) { if (*myPortInputRegister(PORTD) & n) return HIGH; return LOW; }

float temperature;
float humidity;
//uint32_t myMillis = 0;

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
  uint8_t myDHTBit = (1 << pin);

  //start signal
  myPinMode(myDHTBit, OUTPUT);             
  myDigitalWrite(myDHTBit, LOW);           
  delayMicroseconds(1000);           
  myDigitalWrite(myDHTBit, HIGH);          
  delayMicroseconds(40);             

  //wait for response signal
  myPinMode(myDHTBit, INPUT);              
  while(myDigitalRead(myDHTBit));          
  while(!myDigitalRead(myDHTBit));         
  while(myDigitalRead(myDHTBit));      

  //clear interrupt to read data correctly
  cli();
  for(uint8_t i = 0; i < 5; i++) {
    for(uint8_t j = 0; j < 8; j++) {
      while(!myDigitalRead(myDHTBit));      
      delayMicroseconds(50);

      if(myDigitalRead(myDHTBit)) data[i] = (data[i] << 1) | (0x01);
      else data[i] = (data[i] << 1);  

      while(myDigitalRead(myDHTBit));   
    }
  }
  sei();

  //if checksum is correct -> assign data to global variables humidity and temperature
  if(data[4] == ((data[0] + data[1] + data[2] + data[3]) & 0xFF)) {
    humidity  = ((data[0] << 8) | data[1]) / 10.0;
    temperature = ((data[2] & 0x7F) << 8 | data[3]) / 10.0;
    if (data[2] & 0x80) {
      temperature = -temperature;
    }
  }

  Serial.print("Temperature: ");
  Serial.println(temperature);
  Serial.print("Humidity: ");
  Serial.println(humidity);
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