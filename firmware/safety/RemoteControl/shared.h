#include <SPI.h>
#include <RH_RF95.h>
#include <Adafruit_GFX.h>
#include <Adafruit_PCD8544.h>

#define RFM95_CS 8
#define RFM95_RST 4
#define RFM95_INT 3

#define STOP_PIN 9
#define RF95_FREQ 915.0
#define LED 12

struct Packet {
    unsigned char id;
    char password[16];
}

void ensureConnection() {
  
}

void sending(Packet pkt) {
  Packet* pkt = &pkt;
  rf95.send((uint8_t*)&pkt, sizeof(pkt));
  rf95.waitPacketSent();S
}

void receiving() {
  uint8_t buf[sizeof(Packet)];

  if (rf95.recv(buf, sizeof(Packet))) {
    Packet msg = *(Packet*) buf;

    if(msg.id == HEARTBEAT_ID) {
      last_heartbeat = millis();
    }
  }
}

void stopRoutine() {
  
}
