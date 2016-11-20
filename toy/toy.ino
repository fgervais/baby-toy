
#include <Wire.h>
#include <Adafruit_MPR121.h>
#include <EnergySaving.h>

//#define DEBUG
#define DEBUG_INFO      1
#define DEBUG_VERBOSE   2

#define VBAT_PIN        A7
#define TONE_PIN        5
#define YELLOW_BUTTON_PIN 6
#define ONBOARD_LED_PIN 13

#define TONE_FREQ 330

#define HEATBEAT_TOGGLE_SPEED_MS  1000

#define MAX_IDLE_TIME_MS  30*1000

#define time_after(a,b) \
  ((long)((b) - (a)) < 0)

EnergySaving energy_saving;
unsigned long last_idle_time;
uint8_t not_idle = 0;

bool tone_playing = false;

Adafruit_MPR121 touch_sensor = Adafruit_MPR121();
uint16_t last_touched = 0;
bool touching = false;

unsigned long heartbeat_last_toggle_time;

int debug_level = DEBUG_INFO;

void ext_int() {
  //sleep_disable();
  //detachInterrupt(digitalPinToInterrupt(YELLOW_BUTTON_PIN));
}

void setup() {
  //while (!Serial);
  pinMode(YELLOW_BUTTON_PIN, INPUT_PULLUP);

  /* Onboard LED */
  pinMode(ONBOARD_LED_PIN, OUTPUT);
  digitalWrite(ONBOARD_LED_PIN, HIGH);
  heartbeat_last_toggle_time = millis();

  /* Standby */
  energy_saving.begin(WAKE_EXT_INTERRUPT, digitalPinToInterrupt(YELLOW_BUTTON_PIN), ext_int);

  touch_sensor.begin(0x5A);

  Serial.print("setup done\n");
}

#ifdef DEBUG
void debug(const char *msg, int level) {
  if(level <= debug_level)
    Serial.print(msg);
}
#else
  void debug(const char *msg, int level) {
  }
#endif

float getVBat() {
  float measuredvbat = analogRead(VBAT_PIN);
  measuredvbat *= 2;    // we divided by 2, so multiply back
  measuredvbat *= 3.3;  // Multiply by 3.3V, our reference voltage
  measuredvbat /= 1024; // convert to voltage
  //Serial.print("VBat: " );
  //Serial.println(measuredvbat);
  return measuredvbat;
}

bool is_idle() {
  return !not_idle;
}

void set_idle() {
  if(not_idle)
    not_idle--;
  if(!not_idle)
    last_idle_time = millis();
}

void set_not_idle() {
  not_idle++;
}

void prepare_to_sleep() {
  digitalWrite(ONBOARD_LED_PIN, LOW);
}

void standby_if_idle() {
  if(is_idle() && time_after(millis(), last_idle_time + MAX_IDLE_TIME_MS)) {
    prepare_to_sleep();
    energy_saving.standby();
  }
}

void heartbeat() {
  if(time_after(millis(), heartbeat_last_toggle_time + HEATBEAT_TOGGLE_SPEED_MS)) {
    digitalWrite(ONBOARD_LED_PIN, !digitalRead(ONBOARD_LED_PIN));
    heartbeat_last_toggle_time = millis();
  }
}

void get_touch_status(uint16_t *touched, uint16_t *released) {
  uint8_t curr_touched = 0;

  curr_touched = touch_sensor.touched();
  *touched = ~last_touched & curr_touched;
  *released = last_touched & ~curr_touched;
  last_touched = curr_touched;

  if(*touched) {
    debug("Touched\n", DEBUG_VERBOSE);
  }
  else if(*released) {
    debug("Released\n", DEBUG_VERBOSE);
  }
}

void loop() {
  uint16_t touch_start, touch_released;

  get_touch_status(&touch_start, &touch_released);
  if(touch_start) {
    touching = true;
  }
  else if(touch_released) {
    touching = false;
  }

  if(touching) {
    debug("Touching\n", DEBUG_INFO);
  }
  else {
    debug("!Touching\n", DEBUG_VERBOSE);
  }

  if(!tone_playing && (digitalRead(YELLOW_BUTTON_PIN) == LOW || touching)) {
    debug("Start tone\n", DEBUG_INFO);
    tone(TONE_PIN, TONE_FREQ);
    tone_playing = true;
    set_not_idle();
  }
  else if(tone_playing && (digitalRead(YELLOW_BUTTON_PIN) == HIGH && !touching)){
    debug("Stop tone\n", DEBUG_INFO);
    noTone(TONE_PIN);
    tone_playing = false;
    set_idle();
  }

#ifdef DEBUG
  delay(1000);
#endif

#ifndef DEBUG
  standby_if_idle();
#endif
  heartbeat();
}
