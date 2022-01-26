/*
* Brian R Taylor
* brian.taylor@bolderflight.com
* 
* Copyright (c) 2022 Bolder Flight Systems Inc
*
* Permission is hereby granted, free of charge, to any person obtaining a copy
* of this software and associated documentation files (the “Software”), to
* deal in the Software without restriction, including without limitation the
* rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
* sell copies of the Software, and to permit persons to whom the Software is
* furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included in
* all copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
* FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
* IN THE SOFTWARE.
*/

#include "easycan.h"

/* Loopback test from CAN0 to CAN1 and CAN1 to CAN0 on a Teensy 3.6 */

/*
* One object for each CAN bus, 32 CanMsg's can be stored in each RX and TX FIFO
*/
bfs::EasyCan<CAN0, 32, 32> can0;
bfs::EasyCan<CAN1, 32, 32> can1;

/* A CanMsg to send */
bfs::CanMsg msg;

void setup() {
  /* Start serial for feedback */
  Serial.begin(115200);
  while (!Serial) {}
  Serial.println("STARTING TEST");
  /* Enable the CAN transceivers, these will be config dependent */
  pinMode(26, OUTPUT);
  pinMode(27, OUTPUT);
  digitalWriteFast(26, LOW);
  digitalWriteFast(27, LOW);
  /* Begin the CAN */
  can0.Begin(1000000);
  can1.Begin(1000000);
  /*
  * Setup filters on CAN1, enable ID 1 and ID 15 - 20
  */
  can1.FilterRejectAll();
  can1.SetFilter(0, 1, STD);
  can1.SetFilterRange(1, 15, 20, STD);
  /* Write messages between the CAN buses, non-blocking, no timeout */
  for (std::size_t i = 0; i < 24; i++) {
    msg.id = i;
    can0.Write(msg, -1, false);
    can1.Write(msg, -1, false);
  }
  /* Delay to let messages propagate */
  delay(10);
  /* See what each reeceived */
  Serial.println("CAN0 Received: ");
  std::size_t msg_avail = can0.available();
  Serial.print(msg_avail);
  Serial.println(" messages:");
  while (can0.available()) {
    can0.Read(&msg, 1);
    Serial.print("ID & Timestamp:\t");
    Serial.print(msg.id);
    Serial.print("\t");
    Serial.println((uint32_t)msg.timestamp_us);
  }
  Serial.println("\nCAN1 Received: ");
  msg_avail = can1.available();
  Serial.print(msg_avail);
  Serial.println(" messages:");
  while (can1.available()) {
    bfs::optional<bfs::CanMsg> ret = can1.Read();
    if (ret) {
      Serial.print("ID & Timestamp:\t");
      Serial.print(ret.value().id);
      Serial.print("\t");
      Serial.println((uint32_t)ret.value().timestamp_us);
    }
  }
}

void loop() {}
