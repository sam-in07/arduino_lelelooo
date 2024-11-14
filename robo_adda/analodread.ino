int x;

void setup() {
  pinMode(13, OUTPUT);
  Serial.begin(9600);
}

void loop() {
  x = analogRead(A0);
  Serial.println(x);
  if (x > 500) {
    digitalWrite(13, HIGH);
  } else {
    digitalWrite(13, LOW);
  }
}
/*
delay(500) mani delay diya blinking
This code reads an analog value from pin A0, 
prints it to the serial monitor, and turns an LED on pin 13 on or off based on whether the value is greater than 500. */
