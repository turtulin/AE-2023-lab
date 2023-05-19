#define DHT22_PIN 2

float temperature = 0;
float humidity = 0;

void setup() {
  Serial.begin(9600);
  DDRD &= ~(1 << DHT22_PIN);
  PORTD |= (1 << DHT22_PIN); // Enable pull-up resistor
  delayMicroseconds(2000); // Delay to allow the sensor to stabilize
}

void loop() {
  delay(6000);
  readDHT22(DHT22_PIN);
}


void readDHT22(uint8_t pin) {

  pinMode(pin, INPUT_PULLUP);
  delay(1);
  pinMode(pin, OUTPUT);
  digitalWrite(pin, LOW);
  delayMicroseconds(1100);
  pinMode(pin, INPUT_PULLUP);



  // Read the DHT22 sensor data
  uint8_t data[5] = {0, 0, 0, 0, 0};

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
  Serial.println(temperature);
  Serial.println(data[0]);
  Serial.println(data[1]);
  Serial.println(data[2]);
  Serial.println(data[3]);
  Serial.println(data[4]);
}