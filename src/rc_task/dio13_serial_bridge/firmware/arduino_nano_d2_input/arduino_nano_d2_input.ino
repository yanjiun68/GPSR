/*
 * Arduino Nano digital-input publisher for dio13_serial_bridge.
 *
 * Wiring: connect the switch/contact between D2 and GND.  Do not apply an
 * external 5 V signal to D2 for this circuit: INPUT_PULLUP provides the idle
 * HIGH level internally.  A connection to GND is reported as GROUNDED.
 */

constexpr uint8_t INPUT_PIN = 2;              // Nano D2; D13 is intentionally unused.
constexpr unsigned long BAUD_RATE = 115200;
constexpr unsigned long PUBLISH_PERIOD_MS = 100;  // 10 Hz constant report rate.

unsigned long last_publish_ms = 0;

void publishInputState() {
  const bool grounded = digitalRead(INPUT_PIN) == LOW;
  Serial.print(F("NANO_D2="));
  Serial.println(grounded ? F("GROUNDED") : F("RELEASED"));
}

void setup() {
  pinMode(INPUT_PIN, INPUT_PULLUP);
  Serial.begin(BAUD_RATE);
  // Publish the actual pin level immediately, then continue at 10 Hz.
  publishInputState();
  last_publish_ms = millis();
}

void loop() {
  const unsigned long now = millis();
  if (now - last_publish_ms >= PUBLISH_PERIOD_MS) {
    last_publish_ms = now;
    publishInputState();
  }
}
