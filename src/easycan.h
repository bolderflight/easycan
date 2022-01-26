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

#ifndef SRC_EASYCAN_H_
#define SRC_EASYCAN_H_

#include <array>
#include "FlexCAN_T4.h"  // NOLINT
#include "circle_buf.h"  // NOLINT

namespace bfs {
/* CAN message structure */
struct CanMsg {
  struct {
    bool extended = 0;
    bool remote = 0;
    bool overrun = 0;
  } flags;
  uint8_t len = 8;
  uint8_t buf[8] = {0};
  uint32_t id;
  int64_t timestamp_us;
};

/* Utilities needed for timing and to route messages from interrupt */
namespace internal {

/* Interface class for EasyCan to enable using object pointers */
class EasyCanIface {
  friend class EasyCanRouter;
 protected:
  virtual void OnReceive(const CAN_message_t &msg) = 0;
  virtual void OnTransmit() = 0;
};

/* 64 bit version of micros() to avoid rollover */
uint64_t micros64() {
  static uint32_t low32, high32;
  uint32_t new_low32 = micros();
  if (new_low32 < low32) {
    high32++;
  }
  low32 = new_low32;
  return((uint64_t) high32 << 32 | low32);
}

/*
* Maximum number of buses, really it's 3, but on Teensy 4.x, the buses are
* numbered CAN1, CAN2, CAN3 instead of base 0.
*/
static constexpr int8_t MAX_NUM_IFACE_ = 4;
static constexpr int8_t MAX_IFACE_INDEX_ = MAX_NUM_IFACE_ - 1;

/*
* Manages an address book of EasyCanIface derived classes based on bus number
* and routes messages to the correct one.
*/
class EasyCanRouter {
 public:
  /* Registers an EasyCanIface derived class with the router by bus number */
  bool Register(const int8_t bus_num, EasyCanIface * can) {
    if ((bus_num < 0) || (bus_num > MAX_IFACE_INDEX_)) {return false;}
    iface_[bus_num] = can;
    return true;
  }

  /* Routes received CAN_message_t to the correct EasyCanIface derived class */
  void RouteRx(const CAN_message_t &msg) {
    iface_[msg.bus]->OnReceive(msg);
  }

  /* Routes on transmit interrupts to the correct EasyCanIface derived class */
  void RouteTx(const CAN_message_t &msg) {
    iface_[msg.bus]->OnTransmit();
  }
 private:
  std::array<EasyCanIface *, MAX_NUM_IFACE_> iface_;
} easy_can_router;

/* Handles RX interrupts from FlexCAN_T4 */
void RxHandler(const CAN_message_t &msg) {
  easy_can_router.RouteRx(msg);
}
/* Handles TX interrupts from FlexCAN_T4 */
void TxHandler(const CAN_message_t &msg) {
  easy_can_router.RouteTx(msg);
}
}  // namespace internal

/* Easy CAN class, templated by CAN bus, RX buf depth, and TX buf depth */
template<CAN_DEV_TABLE BUS, std::size_t RX_BUF, std::size_t TX_BUF>
class EasyCan : public internal::EasyCanIface {
 public:
  EasyCan() {
    /* Figure out our bus number */
    if (BUS == CAN0) {
      bus_num_ = 0;
    } else if (BUS == CAN1) {
      bus_num_ = 1;
    } else if (BUS == CAN2) {
      bus_num_ = 2;
    } else if (BUS == CAN3) {
      bus_num_ = 3;
    }
  }
  bool Begin(const int32_t baud) {
    can_.begin();
    can_.setBaudRate(baud);
    /* Set the transmit mailbox */
    if (!can_.setMB(static_cast<FLEXCAN_MAILBOX>(TX_MB_NUM_), TX)) {
      return false;
    }
    can_.setRFFN(RFFN_32);        // 32 FIFO filters available
    can_.enableFIFO();            // enable the FIFO
    can_.setMRP(0);               // prioritize FIFO
    can_.enableFIFOInterrupt();   // enable the FIFO interrupt
    /* Register with router */
    if (!internal::easy_can_router.Register(bus_num_, this)) {
      return false;
    }
    /* Register interrupt handlers */
    can_.onReceive(FIFO, internal::RxHandler);
    can_.onTransmit(internal::TxHandler);
    return true;
  }
  /* Enables using the alternate pins */
  void SetRx(FLEXCAN_PINS pin = DEF) {can_.setRx(pin);};
  void SetTx(FLEXCAN_PINS pin = DEF) {can_.setTx(pin);};
  /* Rejects all messages */
  void FilterRejectAll() {can_.setFIFOFilter(REJECT_ALL);}
  /* Accept all messages */
  void FilterAcceptAll() {can_.setFIFOFilter(ACCEPT_ALL);}
  /* Configure a filter given a filter number, id, STD/EXT id, RTR (optional) */
  bool SetFilter(uint8_t filter, uint32_t id1, const FLEXCAN_IDE &ide) {
    return can_.setFIFOFilter(filter, id1, ide);
  }
  /*
  * Configure a filter given a filter number, range of IDs, STD/EXT,
  * RTR (optional)
  */
  bool SetFilterRange(uint8_t filter, uint32_t id1, uint32_t id2,
                      const FLEXCAN_IDE &ide) {
    bool status = can_.setFIFOFilterRange(filter, id1, id2, ide);
    can_.enhanceFilter(FIFO);
    return status;
  }
  /* Return the number of filters */
  int8_t NUM_FILTERS() const {return NUM_FILTERS_;}
  /*
  * Writes a CAN message.
  *
  * If timeout_duration is positive and blocking is true,
  * the CAN bus will wait until the message is sent or the timeout expires.
  * True is returned if the message was sent.
  *
  * If the timeout_durection is positive and blocking is false, the message
  * is placed in a FIFO buffer. True is returned if there is space available
  * in the FIFO buffer. When the message comes to the front of the buffer
  * it is sent if the timeout has not expired, otherwise it is discarded.
  *
  * If the timeout duration is zero. The message is attempted to be sent
  * immediately, true is returned if it is able to be immediately sent.
  * Blocking doesn't matter in this scenario.
  *
  * If the timeout duration is negative and blocking is true, this function
  * will wait indefinitely until the message is sent and true is returned.
  *
  * If the timeout duration is negative and block is flase, the message is
  * placed in a FIFO buffer and true is returned if there is space available.
  * When the message comes to the front of the buffer, it is sent.
  */
  bool Write(const CanMsg &msg, int32_t timeout_duration_us, bool blocking) {
    /* Write immediately, if able */
    if (timeout_duration_us == 0) {
      if (tx_ready()) {
        can_.write(static_cast<FLEXCAN_MAILBOX>(TX_MB_NUM_),
                   ConvertMessage(msg));
        return true;
      } else {
        return false;
      }
    } else if (timeout_duration_us > 0) {
      /* Block until we can transmit or timeout */
      if (blocking) {
        elapsed_us_ = 0;
        while (elapsed_us_ < timeout_duration_us) {
          if (tx_ready()) {
            can_.write(static_cast<FLEXCAN_MAILBOX>(TX_MB_NUM_),
                       ConvertMessage(msg));
            return true;
          }
        }
        return false;
      /* Non-blocking, put the message in a transmit buffer */
      } else {
        /* If we can write immediately, transmit */
        if (tx_ready()) {
          can_.write(static_cast<FLEXCAN_MAILBOX>(TX_MB_NUM_),
                     ConvertMessage(msg));
          return true;
        /* Store in a transmit buffer */
        } else {
          temp_msg_ = msg;
          /* Store the timeout in the timestamp field */
          temp_msg_.timestamp_us = micros64() + timeout_duration_us;
          return tx_buf_.Write(temp_msg_);
        }
      }
    /* No timeout to write */
    } else {
      /* Block and wait indefinitely to send message */
      if (blocking) {
        while (1) {
          if (tx_ready()) {
            can_.write(static_cast<FLEXCAN_MAILBOX>(TX_MB_NUM_),
                       ConvertMessage(msg));
            return true;
          }
        }
      /* Non-blocking, put on the TX buffer to send */
      } else {
        /* If we can write immediately, transmit */
        if (tx_ready()) {
          can_.write(static_cast<FLEXCAN_MAILBOX>(TX_MB_NUM_),
                     ConvertMessage(msg));
          return true;
        /* Store in a transmit buffer */
        } else {
          temp_msg_ = msg;
          /* Store the timeout in the timestamp field */
          temp_msg_.timestamp_us = -1;
          return tx_buf_.Write(temp_msg_);
        }
      }
    }
  }
  /* Returns true if the TX mailbox is ready to transmit */
  bool tx_ready() {
    return (FLEXCAN_get_code(FLEXCANb_MBn_CS(BUS, TX_MB_NUM_)) ==
            FLEXCAN_MB_CODE_TX_INACTIVE);
  }
  /* Amount of TX buffer available */
  std::size_t available_for_write() const {
    return tx_buf_.capacity() - tx_buf_.size();
  }
  /* Number of packets in RX buffer */
  std::size_t available() const {
    return rx_buf_.size();
  }
  /* Copy packets out of RX buffer, return number of packets copied */
  std::size_t Read(CanMsg * const data, const std::size_t len) {
    return rx_buf_.Read(data, len);
  }

 protected:
  /* On receive interrupt handler */
  void OnReceive(const CAN_message_t &msg) {
    /* Stuff a message in the RX buffer */
    if (!rx_buf_.Write(ConvertMessage(msg))) {
      // XXX error
    }
  }
  /* On transmit interrupt handler */
  void OnTransmit() {
    /* Check that CAN is ready to send and that we have messages to send */
    if (tx_ready() && tx_buf_.size()) {
      /* Pop a message off the TX buffer */
      tx_buf_.Read(&temp_msg_, 1);
      /* Check the timeout */
      if (temp_msg_.timestamp_us < 0) {
        can_.write(static_cast<FLEXCAN_MAILBOX>(TX_MB_NUM_),
                   ConvertMessage(temp_msg_));
      } else if (micros64() < temp_msg_.timestamp_us) {
        can_.write(static_cast<FLEXCAN_MAILBOX>(TX_MB_NUM_),
                   ConvertMessage(temp_msg_));
      } else {
        /* Timeout reached, still should trigger until the TX buffer is empty */
        if (tx_buf_.size()) {
          OnTransmit();
        }
      }
    }
  }

 private:
  /* Bus number */
  int8_t bus_num_;
  /* TX mailbox number */
  static constexpr int8_t TX_MB_NUM_ = 14;
  /* Maximum number of filters */
  static constexpr int8_t NUM_FILTERS_ = 32;
  /* Temp CanMsg */
  CanMsg temp_msg_;
  /* Elapsed time */
  elapsedMicros elapsed_us_;
  /* CAN driver */
  FlexCAN_T4<BUS, RX_SIZE_2, TX_SIZE_2> can_;
  /* Circular buffers to hold RX and TX messages */
  CircleBuf<CanMsg, RX_BUF> rx_buf_;
  CircleBuf<CanMsg, TX_BUF> tx_buf_;
  /* Convert from CAN_message_t to CanMsg */
  CanMsg ConvertMessage(const CAN_message_t &msg) {
    CanMsg ret;
    ret.timestamp_us = micros64();
    ret.id = msg.id;
    ret.flags.extended = msg.flags.extended;
    ret.flags.remote = msg.flags.remote;
    ret.flags.overrun = msg.flags.overrun;
    ret.len = msg.len;
    for (std::size_t i = 0; i < ret.len; i++) {
      ret.buf[i] = msg.buf[i];
    }
    return ret;
  }
  /* Convert from CanMsg to CAN_message_t */
  CAN_message_t ConvertMessage(const CanMsg &msg) {
    CAN_message_t ret;
    ret.id = msg.id;
    ret.flags.extended = msg.flags.extended;
    ret.flags.remote = msg.flags.remote;
    ret.flags.overrun = msg.flags.overrun;
    ret.len = msg.len;
    for (std::size_t i = 0; i < ret.len; i++) {
      ret.buf[i] = msg.buf[i];
    }
    return ret;
  }
};

}  // namespace bfs

#endif  // SRC_EASYCAN_H_
