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

  // Read the DHT22 sensor data
  uint8_t data[5] = {0, 0, 0, 0, 0};

  for (int i = 0; i < 5; i++) {
    for (int j = 7; j >= 0; j--) {
      while (!(PIND & (1 << pin)));
      delayMicroseconds(30);
      if (PIND & (1 << pin)) {
        data[i] |= (1 << j);
      }
      delayMicroseconds(48);
    }
  }

  uint8_t checksum = data[0] + data[1] + data[2] + data[3];
  if (checksum != data[4]) {
    Serial.println("Checksum error");
  }

  humidity = ((data[0] << 8) + data[1]) / 10.0;
  temperature = (((data[2] & 0x7F) << 8) + data[3]) / 10.0;
  if (data[2] & 0x80) {
    temperature = -temperature;
  }

  Serial.println(humidity);
  Serial.println(temperature);
  Serial.println(data[0]);
  Serial.println(data[1]);
  Serial.println(data[2]);
  Serial.println(data[3]);
  Serial.println(data[4]);
}
