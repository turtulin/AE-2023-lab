#define myPinMode(n,mode) (mode == OUTPUT? (DDRD |= (1<<n)) : (DDRD &= ~(1<<n))); \
(mode == INPUT_PULLUP? (PORTD |= (1<<n)) : (NULL));
#define myDigitalWrite(n,level) (level == HIGH? (PORTD |= (1<<n)) : (PORTD &= ~(1<<n)));
//avrei potuto usare la macro _BV invece di 1<<n

#define DHT22_PIN 2

float temperature = 0;
float humidity = 0;

void setup() {
  Serial.begin(9600);
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



  // Read the DHT22 sensor data
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
      /*if (PIND & (1 << pin)) {
        data[i] |= (1 << j);
      }*/
    }
  }
  sei();

  uint8_t checksum = data[0] + data[1] + data[2] + data[3];
  if (checksum != data[4]) {
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