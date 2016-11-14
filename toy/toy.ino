
#include <EnergySaving.h>

#define VBAT_PIN        A7
#define TONE_PIN        5
#define YELLOW_BUTTON_PIN 6
#define ONBOARD_LED_PIN 13

#define TONE_FREQ 330

#define HEATBEAT_TOGGLE_SPEED_MS  1000

#define MAX_IDLE_TIME_MS  5*1000

#define time_after(a,b) \
  ((long)((b) - (a)) < 0)

EnergySaving nrgSave;
unsigned long last_idle_time;
unsigned long heartbeat_last_toggle_time;
boolean tone_playing = false;

void ext_int() {
  //sleep_disable();
  //detachInterrupt(digitalPinToInterrupt(YELLOW_BUTTON_PIN));
}

void setup() {
  //while (!Serial);
  pinMode(YELLOW_BUTTON_PIN, INPUT_PULLUP);
  //tone(TONE_PIN, TONE_FREQ);

  /* Onboard LED */
  pinMode(ONBOARD_LED_PIN, OUTPUT);
  digitalWrite(ONBOARD_LED_PIN, HIGH);
  heartbeat_last_toggle_time = millis();

  /* Standby */
  nrgSave.begin(WAKE_EXT_INTERRUPT, digitalPinToInterrupt(YELLOW_BUTTON_PIN), ext_int);
  last_idle_time = millis();
  Serial.print("setup done\n");
}

float getVBat() {
  float measuredvbat = analogRead(VBAT_PIN);
  measuredvbat *= 2;    // we divided by 2, so multiply back
  measuredvbat *= 3.3;  // Multiply by 3.3V, our reference voltage
  measuredvbat /= 1024; // convert to voltage
  //Serial.print("VBat: " );
  //Serial.println(measuredvbat);
  return measuredvbat;
}

void not_idle() {
  last_idle_time = millis();
}

void standby_if_idle() {
  if(time_after(millis(), last_idle_time + MAX_IDLE_TIME_MS)) {
    digitalWrite(ONBOARD_LED_PIN, LOW);
    nrgSave.standby();
    not_idle();
  }
}

void heartbeat() {
  if(time_after(millis(), heartbeat_last_toggle_time + HEATBEAT_TOGGLE_SPEED_MS)) {
    digitalWrite(ONBOARD_LED_PIN, !digitalRead(ONBOARD_LED_PIN));
    heartbeat_last_toggle_time = millis();
  }
}

void loop() {
  /*int freq = map(analogRead(A0), 0, 1023, 0, 4096);
    tone(5, freq);

    Serial.print(freq);
    Serial.print('\n');
    delay(1000);*/

  if(!tone_playing && digitalRead(YELLOW_BUTTON_PIN) == LOW) {
    tone(TONE_PIN, TONE_FREQ);
    not_idle();
    tone_playing = true;
  }
  else if(tone_playing && digitalRead(YELLOW_BUTTON_PIN) == HIGH){
    noTone(TONE_PIN);
    not_idle();
    tone_playing = false;
  }

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
    /*digitalWrite(13, HIGH);   // turn the LED on (HIGH is the voltage level)
    delay(1000);              // wait for a second
    digitalWrite(13, LOW);    // turn the LED off by making the voltage LOW
    delay(1000);              // wait for a second*/
  //}
  //Serial.print("Loop\n");
  
  /*if(time_after(millis(), last_idle_time + 3000)) {
    nrgSave.standby();
    last_idle_time = millis();
  }*/

  standby_if_idle();
  heartbeat();
}
