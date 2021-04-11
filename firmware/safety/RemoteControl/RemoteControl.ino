#include <SPI.h>
#include <RH_RF95.h>
#include <Adafruit_GFX.h>
#include <Adafruit_PCD8544.h>
#include "credentials.h"

#define RFM95_CS 8
#define RFM95_RST 4
#define RFM95_INT 3

#define STOP_PIN 9
#define RF95_FREQ 915.0

RH_RF95 rf95(RFM95_CS, RFM95_INT);
Adafruit_PCD8544 display = Adafruit_PCD8544(5, 6, 10, 11, 12);

volatile bool sendStop = false;

/*  
 *   This function needs to be fleshed out.
 *  Should send a NACK if the remote password is wrong, but should also
 *  display voltage if the Robot Radio Module connection has been ensured.
 */
void listen()
{
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(BLACK);
  display.setCursor(0,0);
  if (rf95.waitAvailableTimeout(1000))
  {
    uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);

    if (rf95.recv(buf, &len))
    {
      // Convert packet to character array to determine message contents
      char* msg = (char *)buf;

      // If the package is the password, emergency stop the robot
      if(strcmp(REMOTE_PASSWORD, msg) == 0)
      {
        display.println("Robot on!");
        display.display();
        delay(3000);
        // Send acknowledgement
        uint8_t data[] = "ACK";
        rf95.send(data, sizeof(data));
        rf95.waitPacketSent();
      }
      // Otherwise, should either send NACK or display voltage...
      // Not sure how to implement at the moment
      else
      {
        display.println("Voltage: ");
        display.display();
      }            
    }
    // Receive failure, notify user
    else
    {
      display.println("Receive failed");
    }
  }
}

void stopRoutine()
{
  // display text press
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(BLACK);
  display.setCursor(0,0);
  display.println("Sent kill!");
  display.display();
    
  // send packet
  rf95.send((uint8_t *)ROBOT_PASSWORD, sizeof(ROBOT_PASSWORD));
  //rf95.send((uint8_t *)"Wrong", sizeof("Wrong")); // Generic failure test
  delay(10);
  rf95.waitPacketSent();

  if (rf95.waitAvailableTimeout(1000))
  {
    // Now wait for a reply
    uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);
      
    if(rf95.recv(buf, &len))
    {
      // Convert packet to character array to determine message contents
      char* msg = (char *)buf;
      
      // Check acknowledgement
      if(strcmp("ACK", msg) == 0)
      {
        sendStop = false;
        display.print("Kill success!");
        display.display();
      }
      else
      {
        display.println("Kill failed...");
        display.println("Resending kill");
        display.display();
        delay(3000);
      }
    }
    // Receive failure, notify user
    else
    {
      display.println("Receive failed");
      delay(3000);
    }
  }
}

void ISR()
{
  sendStop = true;
}

void setup()
{
  Serial.begin(115200);

  // display init to blank screen
  display.begin();
  display.setContrast(50);
  display.setReinitInterval(10);
  display.clearDisplay();
  display.display();

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
  pinMode(STOP_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(STOP_PIN), ISR, FALLING);
}

void loop()
{
  listen();
  while(sendStop)
  {
    stopRoutine();
  }
}
