#define DHT22_PIN 2
#define DHT22_BIT (1 << DHT22_PIN)

#define POT_PIN 0

#define RELAY_PIN 3
#define RELAY_BIT (1 << RELAY_PIN)

// Livello minimo e massimo di umidità del suolo (successivamente inseriti dall'utente)
#define livMin 30
#define livMax 70

// Intervalli in secondi tra le letture dei sensori (successivamente inseriti dall'utente, almeno una lettura al giorno)
#define intervalDHT22 7200
#define intervalPot 3600

// Passo bitmask come argomento n
#define myDigitalWrite(n,level) (level == HIGH? (PORTD |= n) : (PORTD &= ~n))
#define myPinMode(n,mode) (mode == OUTPUT ? (DDRD |= n) : (DDRD &= ~n)); \ 
(mode == INPUT_PULLUP? (PORTD |= (1 << n)) : (NULL))
#define myPortInputRegister(port) ((volatile uint8_t *)(__LPM_word_enhanced__(port_to_input_PGM + (port))))

bool myDigitalRead(uint8_t n) { if(*myPortInputRegister(PORTD)&n) return 1; return 0; }

volatile uint16_t counterDHT22 = 0;
volatile uint16_t counterPot = 0;
volatile uint16_t counterRel = 0;

float temperature;
float humidity;
float soilHum;

void setup() {
  cli(); 
  TCCR1A = 0; // Azzero il registro di controllo A del timer 1
  TCCR1B = 0; // Azzero il registro di controllo B del timer 1
  TCNT1 = 0; // Azzera il registro di conteggio del timer 1
  OCR1A = 15624; // Registro di confronto per interrupt ogni secondo
  TCCR1B |= (1 << WGM12); // Abilito la modalità di confronto CTC
  TCCR1B |= (1 << CS12) | (1 << CS10); // Imposto il prescaler a 1024
  TIMSK1 |= (1 << OCIE1A); // Abilito l'interrupt di confronto A del timer 1
  sei();
}

void loop() {
}

// Funzione per leggere il DHT22 che opera in background
void myReadDHT22(uint8_t pin) {
  uint8_t data[5] = {0, 0, 0, 0, 0};

  // Segnale start
  myPinMode(DHT22_BIT, OUTPUT);             
  myDigitalWrite(DHT22_BIT, LOW);           
  delayMicroseconds(1000);           
  myDigitalWrite(DHT22_BIT, HIGH);          
  delayMicroseconds(40);             

  // Segnale risposta
  myPinMode(DHT22_BIT, INPUT);              
  while(myDigitalRead(DHT22_BIT));          
  while(!myDigitalRead(DHT22_BIT));         
  while(myDigitalRead(DHT22_BIT));      

  // Leggo i 40 bit disabilitando interrupt
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

  // Se il checksum è corretto, salvo i valori nelle variabili globali
  if(data[4] == ((data[0] + data[1] + data[2] + data[3]) & 0xFF)) {
    humidity  = ((data[0] << 8) | data[1]) / 10.0;
    temperature = ((data[2] & 0x7F) << 8 | data[3]) / 10.0;
    if (data[2] & 0x80) {
      temperature = -temperature;
    }
  }
}

// Funzione per leggere il potenziometro che opera in background
void myAnalogRead(uint8_t pin)
{
  ADMUX = (pin & 0xF8) | (pin & 0x07);
  ADCSRA |= (1 << ADSC);
  while (ADCSRA & (1 << ADSC));
  soilHum = ADC * (100.0 / 1023.0);
}

ISR(TIMER1_COMPA_vect) {
  counterDHT22++;
  if(counterDHT22 >= intervalDHT22) {
    counterDHT22 = 0;
    myReadDHT22(DHT22_PIN);
  }
  
  counterPot++;
  if(counterPot >= intervalPot) {
    counterPot = 0;
    myAnalogRead(POT_PIN);
  }  
}