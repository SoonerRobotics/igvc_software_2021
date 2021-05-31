#include <SPI.h>
#include <RH_RF95.h>
#include <LiquidCrystal_PCF8574.h>
#include <Wire.h>
#include "common.h"

#define RFM95_CS 8
#define RFM95_RST 4
#define RFM95_INT 3

#define STOP_PIN 10
#define MOBSTOP_PIN 11
#define START_PIN 12
#define RF95_FREQ 915.0

#define LED 13
#define VBATPIN A7

RH_RF95 rf95(RFM95_CS, RFM95_INT);

LiquidCrystal_PCF8574 lcd(0x27); // set the LCD address to 0x27 for a 16 chars and 2 line display

volatile char sendSig = 0;

bool isConnected = false;
char led = 0;
unsigned long lastAck = 0;
unsigned long lastBeat = 0;
unsigned long lastDisplay = 0;
unsigned long lastKill = 0;
float batSmoothed = 50;
RadioPacket toSend;

void stopInterrupt()
{
  if (sendSig == 0)
    sendSig = 1;
}

void mobStopInterrupt()
{
  if (sendSig == 0)
    sendSig = 2;
}

void startInterrupt()
{
  if (sendSig == 0)
    sendSig = 3;
}


void setup()
{
  Serial.begin(115200);

  // display init to blank screen
  lcd.begin(16, 2); // initialize the lcd
  lcd.setBacklight(255);
  lcd.home();
  lcd.clear();
  lcd.noBlink();
  lcd.noCursor();

  pinMode(13, OUTPUT);
  digitalWrite(13, led);

  // reset rfm95
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  // init rfm95
  rf95.init();
  rf95.setFrequency(RF95_FREQ);
  rf95.setTxPower(23, false);

  // button interrupt setup
  pinMode(STOP_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(STOP_PIN), stopInterrupt, FALLING);

  pinMode(MOBSTOP_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(MOBSTOP_PIN), mobStopInterrupt, FALLING);

  pinMode(START_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(START_PIN), startInterrupt, FALLING);

  // initial battery status
  float measuredvbat = analogRead(VBATPIN);
  measuredvbat *= 2;    // we divided by 2, so multiply back
  measuredvbat *= 3.3;  // Multiply by 3.3V, our reference voltage
  measuredvbat /= 1024; // convert to voltage
  batSmoothed = (measuredvbat - 3.3) * 111.1; //111.1 = 100/90 assuming 90 is max percent
  if (batSmoothed > 99) {
    batSmoothed = 99;
  }

  strncpy(toSend.password, GLOBAL_PASSWORD, sizeof(GLOBAL_PASSWORD));

}

void updateDisplay() {
  lcd.home();
  lcd.clear();

  if (sendSig > 0) {
    lcd.print("Sending: ");
    lcd.print((float)sendSig, 0);
    return;
  }

  // Connection status
  if (isConnected) {
    lcd.print("Connected.   ");
    lcd.setCursor(0, 1);
    lcd.print("Last ack: ");
    lcd.print((millis() - lastAck) / 1000.0, 0);
    lcd.print("s");
  } else {
    lcd.print("Waiting...");
  }

  // Battery status
  float measuredvbat = analogRead(VBATPIN);
  measuredvbat *= 2;    // we divided by 2, so multiply back
  measuredvbat *= 3.3;  // Multiply by 3.3V, our reference voltage
  measuredvbat /= 1024; // convert to voltage
  measuredvbat = (measuredvbat - 3.3) * 111.1; //111.1 = 100/90 assuming 90 is max percent
  if (measuredvbat > 99) {
    measuredvbat = 99;
  }

  batSmoothed = 0.9 * batSmoothed + 0.1 * measuredvbat;

  lcd.setCursor(13, 0);
  if (batSmoothed < 10) {
    lcd.print("0");
  }
  lcd.print(batSmoothed, 0);
  lcd.print("%");
}

void loop()
{
  if (rf95.available())
  {

    led = !led;
    digitalWrite(13, led);

    uint8_t buf[sizeof(RadioPacket)];
    uint8_t len = sizeof(buf);

    if (rf95.recv(buf, &len))
    {
      // Convert packet to character array to determine message contents
      RadioPacket msg = *(RadioPacket*)buf;

      // Check password
      if (strcmp(GLOBAL_PASSWORD, msg.password) == 0) {

        // Handshake
        if (!isConnected && msg.id == 1) {
          lastAck = millis();
          lastBeat = millis();
          isConnected = true;
          toSend.id = 2;
          rf95.send((uint8_t*)&toSend, sizeof(toSend));
          rf95.waitPacketSent();
        }

        // Heartbeat ACK
        if (isConnected && msg.id == 4) {
          lastAck = millis();
        }

        // Signal ACK
        if (isConnected && msg.id == 9) {
          lastAck = millis();
          sendSig = 0;
        }
      }
    }
  }

  // Mark us as disconnected if we haven't received an ACK in a while
  if (isConnected && (millis() - lastAck) > 5000) {
    isConnected = false;
    sendSig = 0;
  }

  // Send a signal if we have one waiting
  if (isConnected && sendSig > 0 && (millis() - lastKill) > 300) {
    lastKill = millis();
    toSend.id = 0;
    toSend.message[0] = sendSig;

    rf95.send((uint8_t*)&toSend, sizeof(toSend));
    rf95.waitPacketSent();
  }

  // Send a hearbeat periodically if we haven't received an ACK in a while
  if (isConnected && sendSig == 0 && (millis() - lastAck) > 500 && (millis() - lastBeat) > 300) {
    lastBeat = millis();
    toSend.id = 3;
    rf95.send((uint8_t*)&toSend, sizeof(toSend));
    rf95.waitPacketSent();
  }

  // Update display periodically
  if ((millis() - lastDisplay) > 800) {
    lastDisplay = millis();
    updateDisplay();
  }
}
