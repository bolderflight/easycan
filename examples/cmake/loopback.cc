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

bfs::EasyCan<CAN0, 256, 256> can0;
bfs::EasyCan<CAN1, 256, 256> can1;

bfs::CanMsg msg;

int main() {
  Serial.begin(115200);
  while (!Serial) {}
  Serial.println("STARTING TEST");
  /* Enable the CAN transceivers */
  pinMode(26, OUTPUT);
  pinMode(27, OUTPUT);
  digitalWriteFast(26, LOW);
  digitalWriteFast(27, LOW);
  /* Begin the CAN */
  can0.Begin(1000000);
  can1.Begin(1000000);
  can1.FilterRejectAll();
  // can1.SetFilter(0, 1, STD);
  // can1.SetFilter(1, 2, STD);
  // can1.SetFilter(2, 3, STD);
  // can1.SetFilter(3, 4, STD);
  // bool status = can1.SetFilter(0, 0x0000, 0x0007, STD);
  // Serial.println(status);
  // can1.SetFilter(0, 2, EXT);
  // can1.SetFilter(0, 3, EXT);
  // can1.SetFilter(0, 4, EXT);
  // can1.SetFilter(0, 5, EXT);
  // can1.FilterAcceptAll();
  /* Write a message */
  int t1, t2;
  for (int i = 0; i < 24; i++) {
    msg.id = i;
    can0.Write(msg, -1, false);
    if (i < 10) {
      can1.Write(msg, -1, false);
    }
  }
  delay(10);
  Serial.println(can1.available());
}

