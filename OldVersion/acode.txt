volatile int IN1;
volatile int IN2;
volatile int IN3;
volatile int IN4;
volatile int rightS;
volatile int centerS;
volatile int leftS;

void left_side_forward(int speed) {
  // Left
  analogWrite(IN1,(speed * 0.92));
  analogWrite(IN2,0);
}

void right_side_forward(int speed) {
  // Right
  analogWrite(IN3,speed);
  analogWrite(IN4,0);
}

void left_side_backward(int speed) {
  // Left
  analogWrite(IN1,0);
  analogWrite(IN2,(speed * 0.92));
}

void right_side_backward(int speed) {
  // Right
  analogWrite(IN3,0);
  analogWrite(IN4,speed);
}

boolean mixly_digitalRead(uint8_t pin) {
  pinMode(pin, INPUT);
  boolean _return =  digitalRead(pin);
  pinMode(pin, OUTPUT);
  return _return;
}

void forward(int speed) {
  left_side_forward(speed);
  right_side_forward(speed);
}

void stop() {
  left_side_forward(0);
  right_side_forward(0);
}

void left1(int speed) {
  left_side_forward(speed / 2);
  right_side_forward(speed);
}

void left2(int speed) {
  left_side_backward(speed);
  right_side_forward(speed);
}

void right1(int speed) {
  left_side_forward(speed);
  right_side_forward(speed / 2);
}

void right2(int speed) {
  left_side_forward(speed);
  right_side_backward(speed);
}

void setup(){
  IN1 = 11;
  IN2 = 6;
  IN3 = 5;
  IN4 = 3;
  rightS = 8;
  leftS = 10;
}

void loop(){
  if (mixly_digitalRead(leftS) == 1 && mixly_digitalRead(rightS) == 1) {
    forward(120);

  }
  if (mixly_digitalRead(leftS) == 1  && mixly_digitalRead(rightS) == 0) {
    left2(150);

  }
  if (mixly_digitalRead(leftS) == 0  && mixly_digitalRead(rightS) == 1) {
    right1(150);

  }
  if (mixly_digitalRead(leftS) == 0 && mixly_digitalRead(rightS) == 0) {
    stop();

  }

}
