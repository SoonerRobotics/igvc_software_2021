#include <shared.h>

RH_RF95 rf95(RFM95_CS, RFM95_INT);

void ensureConnection()
{
  bool ackReceived = false;
  while(!ackReceived)
  {
    // Send packet
    rf95.send((uint8_t *)REMOTE_PASSWORD, sizeof(REMOTE_PASSWORD));
    delay(10);
    rf95.waitPacketSent();

    if(rf95.waitAvailableTimeout(1000))
    {
      // Now wait for a reply
      uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
      uint8_t len = sizeof(buf);

      if(rf95.recv(buf, &len))
      {
        char* msg = (char *)buf;
        if(strcmp("ACK", msg) == 0)
        {
          ackReceived = true;
          digitalWrite(LED, HIGH);
          delay(1000);
          digitalWrite(LED, LOW);
        }
      } 
    }
  }
}

void setup()
{
  Serial.begin(115200);
  
  // init led
  pinMode(LED, OUTPUT);

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

  ensureConnection();
}

void loop()
{
  if (rf95.available())
  {
    uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);

    if (rf95.recv(buf, &len))
    {
      // Convert packet to character array to determine message contents
      char* msg = (char *)buf;

      // If the package is the password, emergency stop the robot
      if(strcmp(ROBOT_PASSWORD, msg) == 0)
      {
        digitalWrite(LED, HIGH);

        // Send acknowledgement
        uint8_t data[] = "ACK";
        rf95.send(data, sizeof(data));
        rf95.waitPacketSent();
      }
      // Otherwise, reject the message
      else
      {
        // Send a not-acknowledgement
        uint8_t data[] = "NACK";
        rf95.send(data, sizeof(data));
        rf95.waitPacketSent();
      }            
    }
    // Receive failure, notify user
    // TODO: put an LED on this
    else
    {
      Serial.println("Receive failed");
    }
  }
}
