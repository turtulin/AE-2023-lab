#include "wiring_private.h"

#define MICROSECONDS_PER_TIMER0_OVERFLOW (clockCyclesToMicroseconds(64 * 256))
#define MILLIS_INC (MICROSECONDS_PER_TIMER0_OVERFLOW / 1000)
#define FRACT_INC ((MICROSECONDS_PER_TIMER0_OVERFLOW % 1000) >> 3)
#define FRACT_MAX (1000 >> 3)

extern volatile unsigned long timer0_overflow_count;
extern volatile unsigned long timer0_millis;
unsigned char timer0_fract;

#define myPinMode(n,mode) (mode == OUTPUT? (DDRD |= (1<<n)) : (DDRD &= ~(1<<n))); \ 
(mode == INPUT_PULLUP? (PORTD |= (1<<n)) : (NULL));
#define myDigitalWrite(n,level) (level == HIGH? (PORTD |= (1<<n)) : (PORTD &= ~(1<<n)));
//could use _BV instead of 1<<n

#define DHT22_PIN 2

float temperature = 0;
float humidity = 0;

void setup() {
  Serial.begin(9600);
  Serial.println(MICROSECONDS_PER_TIMER0_OVERFLOW / 1000);
}

void loop() {
  delay(5000);
  readDHT22(DHT22_PIN);
}


void readDHT22(uint8_t pin) {

  myPinMode(pin, INPUT_PULLUP);
  delay(1);
  myPinMode(pin, OUTPUT);
  myDigitalWrite(pin, LOW);
  delayMicroseconds(1100);
  myPinMode(pin, INPUT_PULLUP);

  uint8_t data[5] = {0, 0, 0, 0, 0};

  cli();
  for (int i = 0; i < 5; i++) {
    for (int j = 7; j >= 0; j--) {
      while (!(PIND & (1 << pin)));
      uint64_t startHigh = micros();
      while (PIND & (1 << pin));
      uint32_t durationHigh = micros() - startHigh;
      if(durationHigh >= 68 && durationHigh <= 72) {
        data[i] |= (1 << j);
      }
    }
  }
  sei();

  uint8_t checksum = data[0] + data[1] + data[2] + data[3];
  if ((checksum & 0xFF) != data[4]) {
    Serial.println("Checksum error");
  }

  humidity = ((data[0] << 8) | data[1]) / 10.0;
  temperature = (((data[2] & 0x7F) << 8) | data[3]) / 10.0;
  if (data[2] & 0x80) {
    temperature = -temperature;
  }

  Serial.println(humidity * 4);
  Serial.println(temperature * 4);
  Serial.println(data[0]);
  Serial.println(data[1]);
  Serial.println(data[2]);
  Serial.println(data[3]);
  Serial.println(data[4]);
}

ISR(TIMERO_COMPB_vect) {
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