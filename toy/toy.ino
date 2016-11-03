void setup() {
  pinMode(2, INPUT_PULLUP);
}

void loop() {
  if(digitalRead(2) == LOW)
    tone(8, 2048);
  else
    noTone(8);
}
