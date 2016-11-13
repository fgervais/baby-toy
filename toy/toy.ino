
#include <EnergySaving.h>

#define VBAT_PIN A7
#define YELLOW_BUTTON_PIN 6

#define time_after(a,b) \
  ((long)((b) - (a)) < 0)

EnergySaving nrgSave;
unsigned long last_idle_time;

void ext_int() {
  //sleep_disable();
  //detachInterrupt(digitalPinToInterrupt(YELLOW_BUTTON_PIN));
}

void setup() {
  //while (!Serial);
  pinMode(YELLOW_BUTTON_PIN, INPUT_PULLUP);
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);
  //tone(5, 330);
  nrgSave.begin(WAKE_EXT_INTERRUPT, digitalPinToInterrupt(YELLOW_BUTTON_PIN), ext_int);  //standby setup for external interrupts
  Serial.print("Hello from setup\n");
  last_idle_time = millis();
}

void loop() {

  float measuredvbat = analogRead(VBAT_PIN);
  measuredvbat *= 2;    // we divided by 2, so multiply back
  measuredvbat *= 3.3;  // Multiply by 3.3V, our reference voltage
  measuredvbat /= 1024; // convert to voltage
  Serial.print("VBat: " );
  Serial.println(measuredvbat);

  /*int freq = map(analogRead(A0), 0, 1023, 0, 4096);
    tone(5, freq);

    Serial.print(freq);
    Serial.print('\n');
    delay(1000);*/

  /*if(digitalRead(2) == LOW)
    tone(8, 2048);
    else
    noTone(8);*/

  if(time_after(millis(), last_idle_time+3000)) {
    nrgSave.standby();
    last_idle_time = millis();
  }

  //if (digitalRead(YELLOW_BUTTON_PIN) == LOW) {
    //nrgSave.standby();  //now mcu goes in standby mode
    //attachInterrupt(digitalPinToInterrupt(YELLOW_BUTTON_PIN), ext_int, FALLING);
    //set_sleep_mode(SLEEP_MODE_PWR_DOWN);
    //system_set_sleepmode(SYSTEM_SLEEPMODE_STANDBY);
    //noInterrupts();
    //sleep_bod_disable();
    //interrupts();
    //sleep_cpu();
    /* wake up here */
    //sleep_disable();
    /*SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;
    __DSB();
    __WFE();*/
  //}
  //else {
    digitalWrite(13, HIGH);   // turn the LED on (HIGH is the voltage level)
    delay(1000);              // wait for a second
    digitalWrite(13, LOW);    // turn the LED off by making the voltage LOW
    delay(1000);              // wait for a second
  //}
  //Serial.print("Loop\n");
}
