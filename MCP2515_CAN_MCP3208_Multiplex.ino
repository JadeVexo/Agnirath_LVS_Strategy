/*
 * Adafruit MCP2515 FeatherWing CAN Receiver Callback Example
 */

#include <Adafruit_MCP2515.h>
#include <MCP3208.h>

// Set CAN bus baud rate
#define CAN_BAUDRATE (1000000) //Requires twice the desired baud rate for some reason.

int count = 0;

Adafruit_MCP2515 mcp(10);
MCP3208 adc;


void setup() {
  Serial.begin(115200);
  while(!Serial) delay(10);

  Serial.println("MCP2515 Receiver Callback test!");

  adc.begin(9);
  if (!mcp.begin(1000000)) {//Requires twice the desired baud rate for some reason.
    Serial.println("Error initializing MCP2515.");
    while(1) delay(10);
  }
  Serial.println("MCP2515 chip found");

  // register the receive callback. Calls the function onReceive() whenever an interrupt is received on pin D2.
  mcp.onReceive(2, onReceive);
}

void loop() {
  for (int chan=0; chan<8; chan++) {
    Serial.print(adc.readADC(chan)); Serial.print("\t");
  }

  Serial.print("["); Serial.print(count); Serial.println("]");
  count++;
  
  delay(1000);
}

void onReceive(int packetSize) {
  // received a packet
  Serial.print("Received ");

  if (mcp.packetExtended()) {
    Serial.print("extended ");
  }

  if (mcp.packetRtr()) {
    // Remote transmission request, packet contains no data
    Serial.print("RTR ");
  }

  Serial.print("packet with id 0x");
  Serial.print(mcp.packetId(), HEX);

  if (mcp.packetRtr()) {
    Serial.print(" and requested length ");
    Serial.println(mcp.packetDlc());
  } else {
    Serial.print(" and length ");
    Serial.println(packetSize);

    // only print packet data for non-RTR packets
    while (mcp.available()) {
      Serial.print(mcp.read(),HEX);
    }
    Serial.println();
  }

  Serial.println();
}
