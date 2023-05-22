/*
#include "wiring_private.h"

#define MILLIS_INC (MICROSECONDS_PER_TIMER0_OVERFLOW / 1000)
#define FRACT_INC ((MICROSECONDS_PER_TIMER0_OVERFLOW % 1000) >> 3)
#define FRACT_MAX (1000 >> 3)

extern volatile unsigned long timer0_overflow_count;
extern volatile unsigned long timer0_millis;
unsigned char timer0_fract;
*/

#define MICROSECONDS_PER_TIMER0_OVERFLOW (clockCyclesToMicroseconds(64 * 256))

#define DHT22_PIN 2
#define DHT22_BIT (1 << DHT22_PIN)

#define POT_PIN 0

#define RELAY_PIN 3
#define RELAY_BIT (1 << RELAY_PIN)

//to obtain more generic functions, I could add controls to include PORTB and PORTC
//pass bitmask as argument
#define myDigitalWrite(n,level) (level == HIGH? (PORTD |= n) : (PORTD &= ~n))
#define myPinMode(n,mode) (mode == OUTPUT ? (DDRD |= n) : (DDRD &= ~n)); \ 
(mode == INPUT_PULLUP? (PORTD |= (1 << n)) : (NULL))
#define myPortInputRegister(port) ((volatile uint8_t *)(__LPM_word_enhanced__(port_to_input_PGM + (port))))

int myDigitalRead(uint8_t n) { if (*myPortInputRegister(PORTD) & n) return HIGH; return LOW; }

float temperature;
float humidity;
float soilHum;
//uint32_t myMillis = 0;

void setup() {
  Serial.begin(9600);
  //TIMSK0 = 0x4;
}

void loop() {
  delay(2000);
  myReadDHT22(DHT22_PIN);
  delay(2000);
  myAnalogRead(POT_PIN);
  delay(2000);
  myDigitalWrite(RELAY_BIT,HIGH);
  delay(4000);
  myDigitalWrite(RELAY_BIT,LOW);
}


void myReadDHT22(uint8_t pin) {
  uint8_t data[5] = {0, 0, 0, 0, 0};

  //start signal
  myPinMode(DHT22_BIT, OUTPUT);             
  myDigitalWrite(DHT22_BIT, LOW);           
  delayMicroseconds(1000);           
  myDigitalWrite(DHT22_BIT, HIGH);          
  delayMicroseconds(40);             

  //wait for response signal
  myPinMode(DHT22_BIT, INPUT);              
  while(myDigitalRead(DHT22_BIT));          
  while(!myDigitalRead(DHT22_BIT));         
  while(myDigitalRead(DHT22_BIT));      

  //clear interrupt to read data correctly
  cli();
  for(uint8_t i = 0; i < 5; i++) {
    for(uint8_t j = 0; j < 8; j++) {
      while(!myDigitalRead(DHT22_BIT));      
      delayMicroseconds(50);

      data[i] = (data[i] << 1);
      if(myDigitalRead(DHT22_BIT)) data[i] |= (0x01);

      while(myDigitalRead(DHT22_BIT));   
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

// Define the ADC channel to be used (0-7)

// Function to perform analog-to-digital conversion
void myAnalogRead(uint8_t pin)
{
  ADMUX = (ADMUX & 0xF8) | (pin & 0x07);
  ADCSRA |= (1 << ADSC);
  while (ADCSRA & (1 << ADSC));
  soilHum = ADC * (100.0 / 1023.0);
  Serial.print("Soil Humidity: ");
  Serial.println(soilHum);
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