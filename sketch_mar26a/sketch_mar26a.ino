#define IN1 25
#define IN2 26
#define IN3 33
#define IN4 32

void setup() {
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
}

void loop() {

  // 🚗 MOVE FORWARD
  digitalWrite(IN1, HIGH);   // Left motor forward
  digitalWrite(IN2, LOW);

  digitalWrite(IN3, HIGH);   // Right motor forward
  digitalWrite(IN4, LOW);

  delay(5000);  // run for 5 seconds


  // 🛑 STOP
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);

  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);

  delay(3000);  // stop for 3 seconds
}