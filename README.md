[![Pipeline](https://gitlab.com/bolderflight/software/easycan/badges/main/pipeline.svg)](https://gitlab.com/bolderflight/software/easycan/) [![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

![Bolder Flight Systems Logo](img/logo-words_75.png) &nbsp; &nbsp; ![Arduino Logo](img/arduino_logo_75.png)

# EasyCan
An easy to use, timing-deterministic CAN library for Teensy 3.x and 4.x. Supports Arduino and CMake build systems.
   * [License](LICENSE.md)
   * [Changelog](CHANGELOG.md)
   * [Contributing guide](CONTRIBUTING.md)

# Description
The goal for EasyCan was to develop a Teensy 3.x and 4.x CAN driver that:
   * Has a simple API
   * Sends and receives messages sequentially
   * Supports interrupt driven reception of messages with integrated FIFO buffering
   * Timestamps received messages with the system monotonic time since boot (i.e. micros())
   * Has strong timeout support and blocking / non-blocking options for writing messages

EasyCan is a wrapper around [FlexCAN_T4](https://github.com/tonton81/FlexCAN_T4), which meets these goals and supports CAN2.0 mode.

# Installation

## Arduino
Simply clone or download and extract the zipped library into your Arduino/libraries folder. In addition to this library, the [Bolder Flight Systems Circular Buffer library](https://github.com/bolderflight/circle_buf) and [FlexCAN_T4 library](https://github.com/tonton81/FlexCAN_T4) must also be installed. The FlexCAN_T4 library is also available from [our mirror](https://github.com/bolderflight/flexcan). The library is added as:

```C++
#include "easycan.h"
```

A loopback example using the dual CAN buses on Teensy 3.6 is located in *examples/arduino/loopback/loopback.ino*. This library has been tested with Teensy 3.x and 4.x devices and is expected to work correctly with those.

## CMake
CMake can be used to build this library, which is exported as a library target called *easycan*. The header is added as:

```C++
#include "easycan.h"
```

The library can be also be compiled stand-alone using the CMake idiom of creating a *build* directory and then, from within that directory issuing:

```
cmake .. -DMCU=MK66FX1M0
make
```

This will build the library and an example executables called *loopback_example*. The example executable source file is located at *examples/cmake/loopback.cc*. Notice that the *cmake* command includes a define specifying the microcontroller the code is being compiled for. This is required to correctly configure the code, CPU frequency, and compile/linker options. The available MCUs are:
   * MK20DX128
   * MK20DX256
   * MK64FX512
   * MK66FX1M0
   * MKL26Z64
   * IMXRT1062_T40
   * IMXRT1062_T41

These are known to work with the same packages used in Teensy products. Also switching packages is known to work well, as long as it's only a package change.

The *loopback_example* target creates an executable for testing the library using the dual CAN buses on Teensy 3.6. This target also has a *_hex* for creating the hex file and an *_upload* for using the [Teensy CLI Uploader](https://www.pjrc.com/teensy/loader_cli.html) to flash the Teensy. Please note that the CMake build tooling is expected to be run under Linux or WSL, instructions for setting up your build environment can be found in our [build-tools repo](https://github.com/bolderflight/build-tools). 

# Namespace
This library is within the namespace *bfs*.

# CanMsg
CAN messages are defined in a structure as:

| Type | Parameter | Description |
| --- | --- | --- |
| uint32_t |id | 11 or 29 bit ID |
| bool  |flags.extended | Specify whether 11 or 29 bit ID, false for standard 11 bit IDs and true for extended 29 bit IDs |
| bool  |flags.remote | Specify whether RTR frame, false for normal frame and true for remote frame |
| bool  |flags.overrun | Was there an overrun? false for no overruns detected, true if buffer overrun |
| uint8_t |len | Number of data bytes in the frame (0 - 8) |
| int64_t |timestamp_us | System monotonic time since boot when the message was received, us |
| uint8_t | buf[8] | Data bytes for the frame |

# EasyCan

## Methods

**EasyCan<CAN_DEV_TABLE BUS, std::size_t RX_BUF, std::size_t TX_BUF>** Creates an *EasyCan* object. The object is templated with the CAN bus, receive buffer size, and transmit buffer size. Receive and transmit buffer sizes are defined in terms of the number of *CanMsg* data structures the buffer can hold.

For Teensy 3.1, 3.2, and 3.5 the only CAN bus available is CAN0. For Teensy 3.6, CAN buses available are CAN0 and CAN1. For Teensy 4.0 and 4.1, CAN buses available are CAN1, CAN2, and CAN3. See the [Teensy pinout card](https://www.pjrc.com/teensy/pinout.html) for associating bus numbers with pins.

An example of instantiating an object for CAN bus 0 is:

```C++
bfs::EasyCan<CAN0, 128, 128> can0;
```

**bool Begin(const int32_t baud)** Initializes the CAN bus at the specified baud rate and will start receiving messages sent on the bus. Returns true on successfully initializing and configuring the bus.

```C++
can0.Begin(1000000);
```

**void SetRx(FLEXCAN_PINS pin)** Enables using the alternate RX pin for the CAN bus.

```C++
/* Use the alternate RX pin */
can0.SetRx(ALT);
/* Use the default RX pin */
can0.SetRx(DEF);
```

**void SetTx(FLEXCAN_PINS pin)** Enables using the alternate TX pin for the CAN bus.

```C++
/* Use the alternate TX pin */
can0.SetTx(ALT);
/* Use the default TX pin */
can0.SetTx(DEF);
```

## Filtering
After initialization, by default the object will begin receiving all messages on the bus. Filters can be configured to only receive selected messages. Up to 32 filters can be defined.

**int8_t NUM_FILTERS() const** Returns the maximum number of filters that can be defined (i.e. 32)

**void FilterRejectAll()** Rejects all messages on the bus, this should be run before setting up filters to allow through messages.

```C++
can0.FilterRejectAll();
```

**void FilterAcceptAll()** Accepts all messages on the bus.

```C++
can0.FilterAcceptAll();
```

**bool SetFilter(uint8_t filter, uint32_t id1, const FLEXCAN_IDE &ide)** Configures a filter to accept an individual CAN ID. Whether the CAN ID is an 11 bit STD ID or a 29 bit EXT ID should be specified. Returns true on successfully configuring the filter.

For example, to setup filter number 0 to accept a STD CAN ID of 0x10 would be:

```C++
can0.SetFilter(0, 0x10, STD);
```

**bool SetFilterRange(uint8_t filter, uint32_t id1, uint32_t id2, const FLEXCAN_IDE &ide)** This method enables a range of filter IDs to be specified, from *id1* to *id2*. For example, to setup filter number 0 to accept STD CAN IDs of 1 to 3:

```C++
can0.SetFilterRange(0, 1, 3, STD);
```

## Writing Messages

**bool Write(const CanMsg &msg, int32_t timeout_duration_us, bool blocking)** Writes a message to the CAN bus with several options depending on the specified timeout duration (in us) and whether the method should be blocking (blocking = true) or non-blocking (blocking = false). The following logic table defines the behavior. Timeout duration is always from the current monotonic system time in us, i.e. a duration of 1000 is 1,000 microseconds from when the method was called.

| Timeout Duration | Blocking | Behavior | Return value |
| --- | --- | --- | --- |
| Positive | true | Method blocks until the message is sent or the timeout expires | true if the message was sent |
| Positive | false | The message is sent to the transmit FIFO buffer, when the message comes to the front of the buffer it is sent if the timeout hasn't expired, otherwise it is discarded. | true if there is space in the FIFO buffer |
| Negative | true | Method blocks indefinitely until the message is sent | true |
| Negative | false | The message is sent to the transmit FIFO buffer, when the message comes to the front of the buffer, it is sent | true if there is space in the FIFO buffer |
| Zero | - | Method attempts to send the message immediately | true if the message is able to be immediately sent, otherwise false |

```C++
bfs::CanMsg msg;
/* Blocking write, waiting up to 1ms to send */
can0.Write(msg, 1000, true);
/* Non blocking write, no timeout */
can0.Write(msg, -1, false);
```

**bool tx_ready()** Returns true if the CAN bus is ready to immediately transmit a message.

```C++
/* Check if message can be sent immediately, and then send it */
if (can0.tx_ready()) {
   can0.write(msg, 0, true);
}
```

**std::size_t available_for_write() const** Returns the current space in the transmit buffer available for storing transmit messages.

```C++
/* Check space available for FIFO */
if (can0.available_for_write()) {
   can0.Write(msg, 1000, false);
}
```

## Reading Messages
Interrupts automatically move received and filtered messages to a buffer. Methods are available for checking the number of messages in the buffer and reading messages off the buffer.

**std::size_t available() const** Returns the number of messages currently in the receive buffer.

```C++
std::size_t req = can0.available();
```

**std::size_t Read(CanMsg * const data, const std::size_t len)** Moves messages from the receive buffer into an array pointed to by *data* up to length *len*. Returns the number of messages moved.

```C++
/* Buffer to work with CanMsg */
std::array<bfs::CanMsg, 100> msgs;
/* Requesting up to the buffer length, return the actual number copied */
std::size_t msg_read = can0.Read(msgs.data(), 100);
```
