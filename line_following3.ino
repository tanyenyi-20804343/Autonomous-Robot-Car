
//ENA: RIGHT  ENB:LEFT
#define ENA 11
#define IN1 1
#define IN2 2

#define ENB 3
#define IN3 12
#define IN4 13

//============================
// IR LINE SENSORS
//============================
#define LEFT_SENSOR A1
#define RIGHT_SENSOR A2

//============================
// ENCODERS
//============================
#define LEFT_ENCODER A4
#define RIGHT_ENCODER A3

volatile long leftTicks = 0;
volatile long rightTicks = 0;

//============================
// ENCODER PARAMETERS
//============================
const float PULSES_PER_REV = 20.0;
const float WHEEL_CIRCUMFERENCE = 20.73;

//============================
// LCD
//============================
#include <LiquidCrystal.h>
LiquidCrystal lcd(8, 9, 4, 5, 6, 7);

//============================
// LAST ENCODER STATE
//============================
volatile uint8_t lastA3 = 0;
volatile uint8_t lastA4 = 0;
int count;

unsigned long startTime;

// NEW VARIABLE: stop at 100 cm only once
bool stoppedAt100 = false;

//============================
// PIN CHANGE INTERRUPT SETUP
//============================
void setupEncoders() {
  pinMode(LEFT_ENCODER, INPUT);
  pinMode(RIGHT_ENCODER, INPUT);

  PCICR |= (1 << PCIE1);
  PCMSK1 |= (1 << PCINT11);
  PCMSK1 |= (1 << PCINT12);

  lastA3 = digitalRead(A3);
  lastA4 = digitalRead(A4);
}

//============================
// INTERRUPT
//============================
ISR(PCINT1_vect) {
  uint8_t newA3 = digitalRead(A3);
  uint8_t newA4 = digitalRead(A4);

  if (newA3 != lastA3 && newA3 == HIGH) {
    rightTicks++;
  }
  if (newA4 != lastA4 && newA4 == HIGH) {
    leftTicks++;
  }

  lastA3 = newA3;
  lastA4 = newA4;
}

//============================
// DISTANCE CALC
//============================
float getDistanceCM() {
  float avgTicks = (leftTicks + rightTicks) / 2.0;
  return (avgTicks * WHEEL_CIRCUMFERENCE) / PULSES_PER_REV;
}

//============================
// MOTOR CONTROL
//============================
void forward() {
  analogWrite(ENA, 100);
  analogWrite(ENB, 100);

  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void turnRight() {
  analogWrite(ENA, 250);
  analogWrite(ENB, 250);

  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void turnLeft() {
  analogWrite(ENA, 200);
  analogWrite(ENB, 200);

  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}

void stopCar() {
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);

  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

//============================
// SETUP
//============================
void setup() {
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);

  pinMode(LEFT_SENSOR, INPUT);
  pinMode(RIGHT_SENSOR, INPUT);

  setupEncoders();

  lcd.begin(16, 2);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Distance:");

  startTime = millis();
}

//============================
// LOOP
//============================
void loop() {

  int leftVal = digitalRead(LEFT_SENSOR);
  int rightVal = digitalRead(RIGHT_SENSOR);

  //=== CALCULATE DISTANCE ===
  float dist = getDistanceCM();

  if (!stoppedAt100 && dist >= 100.0) {
    stopCar();

    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Stopped @100cm");
    lcd.setCursor(0, 1);
    lcd.print("Wait 3 sec");

    delay(3000);  // STOP FOR 3 SECONDS

    stoppedAt100 = true;   // Prevent repeating stop
    startTime = millis();  // reset timer if you want (optional)
  }

  //=== LINE FOLLOWING NORMAL ===
  if (leftVal == LOW && rightVal == HIGH) {
    turnLeft();
  } else if (leftVal == HIGH && rightVal == LOW) {
    turnRight();
  } else if (leftVal == HIGH && rightVal == HIGH) {
    forward();
  }

  if (leftVal == LOW && rightVal == LOW) {
    count++;
  } else {
    count = 0;
  }

  if (count > 10) {
    stopCar();
    while (1) {}
  }

  //=== LCD TIME AND DIST ====
  unsigned long currentTime = millis();
  unsigned long elapsedSeconds = (currentTime - startTime) / 1000;

  lcd.setCursor(0, 0);
  lcd.print("Dist:");
  lcd.print(dist);
  lcd.print("cm   ");

  lcd.setCursor(0, 1);
  lcd.print("Time:");
  lcd.print(elapsedSeconds);
  lcd.print("s   ");
}
