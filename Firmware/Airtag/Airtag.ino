#include <ArduinoBLE.h>
#include "PlayRtttl.hpp"

const int TONE_PIN = D6;
BLEService keyFinderService("180A");
//BLECharacteristic keyFinderCharacteristic("2A19", BLERead | BLEWrite, 1);
BLECharacteristic alarmControlCharacteristic("2A3F", BLERead | BLEWrite, 1);  // New characteristic to control alarm
BLECharacteristic alarmEnableCharacteristic("2A06", BLERead | BLEWrite, 1);   // New characteristic to enable/disable alarm

char alarmTone[] = "Alarm:d=16,o=5,b=125:c,e,g,c6,e6,g6,c7,e7,g7";

unsigned long previousMillis = 0;  // Stores the last time the LEDs were updated
const long interval = 250;         // Interval at which to blink (milliseconds)

bool isPlayingAlarm = false;
bool ledState = LOW;
bool alarmEnabled = false;  // Flag to enable or disable alarm

void setup() {
  pinMode(P1_11, OUTPUT);
  pinMode(P0_26, OUTPUT);
  pinMode(P0_27, OUTPUT);

  Serial.begin(9600);
  /*
  while (!Serial)
    ;
    */
  if (!BLE.begin()) {
    Serial.println("Starting BLE failed!");
    while (1)
      ;
  }

  BLE.setLocalName("Airtag");
  BLE.setAdvertisedService(keyFinderService);
  //keyFinderService.addCharacteristic(keyFinderCharacteristic);
  keyFinderService.addCharacteristic(alarmControlCharacteristic);  // Add new characteristic
  keyFinderService.addCharacteristic(alarmEnableCharacteristic);   // Add new characteristic
  BLE.addService(keyFinderService);

  //keyFinderCharacteristic.writeValue((uint8_t)0);
  alarmControlCharacteristic.writeValue((uint8_t)0);
  alarmEnableCharacteristic.writeValue((uint8_t)alarmEnabled);  // Set initial alarm enabled state

  BLE.advertise();
  Serial.println("BLE device is now advertising");
  digitalWrite(P0_26, LOW);  // Turn off LEDs
  digitalWrite(P0_27, LOW);
  isPlayingAlarm = false;
}

void loop() {
  BLEDevice central = BLE.central();

  if (central) {
    Serial.print("Connected to central: ");
    Serial.println(central.address());

    digitalWrite(P0_26, LOW);  // Turn off LEDs
    digitalWrite(P0_27, LOW);
    noTone(P1_11);  // Ensure the buzzer is off

    isPlayingAlarm = false;

    while (central.connected()) {
      // Check if alarm control characteristic has been written
      if (alarmControlCharacteristic.written()) {
        uint8_t alarmControlValue = alarmControlCharacteristic.value()[0];
        if (alarmControlValue) {
          // Activate alarm
          isPlayingAlarm = true;
        } else {
          // Deactivate alarm
          isPlayingAlarm = false;
          noTone(TONE_PIN);
          digitalWrite(P0_26, LOW);
          digitalWrite(P0_27, LOW);
        }
      }

      // Check if alarm enable characteristic has been written
      if (alarmEnableCharacteristic.written()) {
        alarmEnabled = alarmEnableCharacteristic.value()[0];
      }
      if (isPlayingAlarm) {
        unsigned long currentMillis = millis();

        // Flash LEDs like a police siren
        if (currentMillis - previousMillis >= interval) {
          previousMillis = currentMillis;
          ledState = !ledState;  // Toggle the LED state
          digitalWrite(P0_26, ledState ? LOW : HIGH);
          digitalWrite(P0_27, ledState ? HIGH : LOW);
        }

        // Update the alarm tone continuously
        if (!updatePlayRtttl()) {
          // Restart the tone if it has finished
          startPlayRtttl(TONE_PIN, alarmTone);
        }
      } else {
        unsigned long currentTime = millis();
        if (ledState && currentTime - previousMillis >= 250) {
          previousMillis = currentTime;
          ledState = LOW;  // Turn off LED
          digitalWrite(P0_26, LOW);
        }
        if (currentTime - previousMillis >= 3000) {
          previousMillis = currentTime;
          ledState = HIGH;  // Turn on LED
          digitalWrite(P0_26, HIGH);
        }
      }
    }

    Serial.print("Disconnected from central: ");
    Serial.println(central.address());

    // Reset state for disconnection

    // Start playing the alarm tone if alarm is enabled
    if (alarmEnabled) {
      startPlayRtttl(TONE_PIN, alarmTone);
      isPlayingAlarm = true;
      previousMillis = 0;
    }
  }

  if (isPlayingAlarm) {
    unsigned long currentMillis = millis();

    // Flash LEDs like a police siren
    if (currentMillis - previousMillis >= interval) {
      previousMillis = currentMillis;
      ledState = !ledState;  // Toggle the LED state
      digitalWrite(P0_26, ledState ? LOW : HIGH);
      digitalWrite(P0_27, ledState ? HIGH : LOW);
    }

    // Update the alarm tone continuously
    if (!updatePlayRtttl()) {
      // Restart the tone if it has finished
      startPlayRtttl(TONE_PIN, alarmTone);
    }
  }
  else
  {
            unsigned long currentTime = millis();
        if (ledState && currentTime - previousMillis >= 250) {
          previousMillis = currentTime;
          ledState = LOW;  // Turn off LED
          digitalWrite(P0_27, LOW);
        }
        if (currentTime - previousMillis >= 3000) {
          previousMillis = currentTime;
          ledState = HIGH;  // Turn on LED
          digitalWrite(P0_27, HIGH);
        }
  }

  delay(10);  // Small delay to prevent excessive CPU usage
}
