#include <DHT.h>
#include <DHT_U.h>

#define SENS_PIN 2
#define SENS_TYPE DHT22

#define TEMP 38
#define HUM_MIN 50
#define HUM_MAX 60

DHT dht(SENS_PIN, SENS_TYPE);

void setup() {
  Serial.begin(9600);
  dht.begin();
  pinMode(3, OUTPUT);   //heater
  pinMode(4, OUTPUT);   //hum to low
  pinMode(5, OUTPUT);   //hum to high
  pinMode(12, OUTPUT);  //ultrasonic vaporizer
  //digitalWrite(3, HIGH);
  PORTD |= 0x01 << 3;
  //digitalWrite(4, LOW);
  PORTD &= ~(0x01 << 4);
  //digitalWrite(5, LOW);
  PORTD &= ~(0x01 << 5);
  //digitalWrite(12, HIGH);
  PORTB |= 0x01 << 4;
}

void loop() {
  float temp, hum;
  
  hum = dht.readHumidity();
  temp = dht.readTemperature();
  Serial.println("\nTemp.:");
  Serial.print(temp);
  Serial.println("\nHum.:");
  Serial.print(hum);

  if(temp >= TEMP) {
    //heater off
    //digitalWrite(3, HIGH);
    PORTD |= 0x01 << 3;
  }
  else {
    //heater on
    //digitalWrite(3, LOW);
    PORTD &= ~(0x01 << 3);
  }

  if(hum < HUM_MIN) {
    //to dry
    //digitalWrite(4, HIGH);
    PORTD |= 0x01 << 4;
    //digitalWrite(5, LOW);
    PORTD &= ~(0x01 << 5);
    //vaporizer on
    //digitalWrite(12, LOW);
    PORTB &= ~(0x01 << 4);
  }
  else if(hum > HUM_MAX) {
    //to damp
    //digitalWrite(4, LOW);
    PORTD &= ~(0x01 << 4);
    //digitalWrite(5, HIGH);
    PORTD |= 0x01 << 5;
    //vaporizer off
    //digitalWrite(12, HIGH);
    PORTB |= 0x01 << 4;
  }
  else {
    //digitalWrite(4, LOW);
    PORTD &= ~(0x01 << 4);
    //digitalWrite(5, LOW);
    PORTD &= ~(0x01 << 5);
  }

  delay(2000);
}
