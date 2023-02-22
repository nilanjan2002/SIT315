const uint8_t echoPin = 2;
const uint8_t trigPin = 7;
const int LED = 13;
const int LED2 = 12;
const int PIR = 3;
const int threshold = 40;
long duration;
bool led_state = false;
bool led_state2 = false;
bool motion = false;
float distance;

void setup(){
  pinMode(trigPin,OUTPUT);
  pinMode(echoPin,INPUT);
  pinMode(LED, OUTPUT);
  Serial.begin(9600);
  attachInterrupt(digitalPinToInterrupt(echoPin), ISR1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIR), ISR2, CHANGE);
}

void loop(){
  digitalWrite(LED2, LOW);
  digitalWrite(LED,LOW);
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin,HIGH);
  distance = duration/58.2;
  Serial.print("Distance:");
  Serial.println(distance);
  delay(100);
}

void ISR1(){

  if(distance > threshold){
      Serial.println("Ultrasonic trigger trigerred");
    led_state = !led_state;
    digitalWrite(LED, led_state);
  }
}

void ISR2(){
    Serial.println("PIR sensor trigerred");
  led_state2 = !led_state2;
  digitalWrite(LED2, led_state2);
  delay(500);
}
