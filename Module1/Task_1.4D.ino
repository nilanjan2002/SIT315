const uint8_t echoPin = 2;
const uint8_t trigPin = 6;
const int LED = 13;
const int LED2 = 12;
const int PIR = 3;
const int threshold = 40;
long duration; // stores time taken for signal to echo back
bool led_state = false;
bool led_state2 = false;
bool motion = false;
float distance;
const int ledPin = 10; // led for timer-interrupt
volatile bool state = false;

void setup()
{
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(LED, OUTPUT);
  Serial.begin(9600);
  attachInterrupt(digitalPinToInterrupt(echoPin), ISR1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIR), ISR2, CHANGE);
  pinMode(ledPin, OUTPUT);
  noInterrupts();
  // Clear registers
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = 0;
  // enable PD7
  DDRB &= ~(1 << PD7);
  DDRB |= (1 << PD7);
  PORTB &= ~(1 << PD7);
  // Enable PinChangeControl Reigsters
  PCICR |= 0b00000001;
  // enable pin 9 : PB1 -> PIR Sensor
  PCMSK0 |= 0b00000010;
  // we want turn led 13 to blink every 4 seconds = 0.25 Hz
  // 0.25Hz = (16000000/((62499+1)*1024))
  // Set timer compare
  OCR1A = 62499;
  // Prescaler 1024
  TCCR1B |= (1 << CS12) | (1 << CS10);
  // Output Compare Match A Interrupt Enable
  TIMSK1 |= (1 << OCIE1A);
  // CTC
  TCCR1B |= (1 << WGM12);

  interrupts();
}

void loop()
{
  digitalWrite(LED2, LOW);
  digitalWrite(LED, LOW);
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  distance = duration / 58.2;

  delay(100);
}

void ISR1()
{

  if (distance > threshold) // threshold = 40cm
  {
    Serial.println("Ultrasonic trigger trigerred");
    led_state = !led_state;
    Serial.print("Distance:");
    Serial.println(distance);
    digitalWrite(LED, led_state);
  }
}

void ISR2()
{
  Serial.println("PIR sensor trigerred");
  led_state2 = !led_state2;
  digitalWrite(LED2, led_state2);
  delay(500);
}
ISR(TIMER1_COMPA_vect)
{                                                // timer compare interrupt
  digitalWrite(ledPin, digitalRead(ledPin) ^ 1); // toggles led
  Serial.println("A Timer interrupt has occured");
}
ISR(PCINT0_vect)
{
  //  PORTB ^= (1<<PD7);
  state = !state;
  digitalWrite(7, state);
  Serial.println("Pin Change Interrupt has occured!!");
  delayMicroseconds(6000);
}
