// =====================================
// MOTOR PINS (your pins)
// =====================================
#define ENA 11
#define IN1 1
#define IN2 2

#define ENB 3
#define IN3 12
#define IN4 13

// =====================================
// ULTRASONIC SENSOR PINS
// =====================================
#define TRIG_PIN A4
#define ECHO_PIN A3

// =====================================
// ULTRASONIC FUNCTION
// =====================================
long getDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH, 30000);  // timeout safety
  long distance = duration * 0.034 / 2;

  return distance;
}

// =====================================
// MOTOR CONTROL
// =====================================
void moveForward() {
  analogWrite(ENA, 200);  // motor speed 0–255
  analogWrite(ENB, 200);

  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);

  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void turnRight() {
  analogWrite(ENA, 200);
  analogWrite(ENB, 200);

  digitalWrite(IN1, HIGH);   // left motor forward
  digitalWrite(IN2, LOW);

  digitalWrite(IN3, LOW);    // right motor backward
  digitalWrite(IN4, HIGH);
}

// =====================================
// STOP MOTOR (NOT USED, BUT INCLUDED)
// =====================================
void stopMotor() {
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
}

// =====================================
// SETUP
// =====================================
void setup() {
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  Serial.begin(9600);
}

// =====================================
// MAIN LOOP
// =====================================
void loop() {
  long distance = getDistance();
  Serial.println(distance);

  if (distance > 0 && distance < 20) {
    
    turnRight();
    delay(350);   
  } 
  else {
    moveForward();
  }

  delay(20);
}
