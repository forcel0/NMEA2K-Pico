/*
NMEA2000_PICO.cpp

Copyright (c) 2015-2020 Timo Lappalainen, Kave Oy, www.kave.fi

Permission is hereby granted, free of charge, to any person obtaining a copy of
this software and associated documentation files (the "Software"), to deal in
the Software without restriction, including without limitation the rights to use,
copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the
Software, and to permit persons to whom the Software is furnished to do so,
subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF
CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE
OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include "NMEA2000_PICO.h"

#if defined(DEBUG_MCP_CAN_SPEED)
uint64_t  McpElapsed=0;
uint64_t  McpStart;
# define DbgStartMcpSpeed McpStart=time_us_64()
# define DbgEndMcpSpeed McpElapsed=time_us_64()-McpStart
# define DbgTestMcpSpeed if ( McpElapsed>0 )
# define DbgClearMcpSpeed McpElapsed=0
# define DbgPrintN2kMcpSpeed(fmt, args...)     printf(fmt , ## args)
# define DbgPrintLnN2kMcpSpeed(fmt, args...)   printf(fmt + "\n", ## args)
#else
# define DbgPrintN2kMcpSpeed(fmt, args...)
# define DbgPrintLnN2kMcpSpeed(fmt, args...)
# define DbgStartMcpSpeed
# define DbgEndMcpSpeed
# define DbgTestMcpSpeed
# define DbgClearMcpSpeed
#endif

bool CanInUse = false;
tNMEA2000_pico *pNMEA2000_mcp = 0;

void CanIdToN2k(unsigned long id, unsigned char &prio, unsigned long &pgn, unsigned char &src, unsigned char &dst);
// #if defined(ESP8266)
// ICACHE_RAM_ATTR void Can1Interrupt();
// #else
void Can1Interrupt(uint gpio, uint32_t events);
// #endif

//*****************************************************************************
void PrintDecodedCanIdAndLen(unsigned long id, unsigned char len) {
  unsigned char prio;
  unsigned long pgn;
  unsigned char src;
  unsigned char dst;

  if (id!=0) {
    CanIdToN2k(id,prio,pgn,src,dst);
    printf("%lu", to_ms_since_boot(get_absolute_time()));
    printf(": pgn: "); printf("%lu", pgn); printf(", prio: "); printf("0x%02X", prio);
    printf(", src: "); printf("0x%02X", src); printf(", dst: "); printf("0x%02X", dst);
  } 
  else {
    printf("id: "); printf("%lu", id);
  }
  printf(", len: "); printf("0x%02X\n",len);
}

//*****************************************************************************
tNMEA2000_pico::tNMEA2000_pico(uint8_t _N2k_CAN_CS_PIN, uint8_t _N2k_CAN_INT_PIN, uint16_t _rx_frame_buf_size) : tNMEA2000(), 
                    N2kCAN(spi0, _N2k_CAN_CS_PIN, PICO_DEFAULT_SPI_TX_PIN, PICO_DEFAULT_SPI_RX_PIN, PICO_DEFAULT_SPI_SCK_PIN, MCP2515::DEFAULT_SPI_CLOCK) 
{
  IsOpen = false;

  if(pNMEA2000_mcp ==0 ) { // Currently only first instance can use interrupts.
    N2k_CAN_INT_PIN = _N2k_CAN_INT_PIN;
    if(UseInterrupt()) {
      MaxCANReceiveFrames = _rx_frame_buf_size;
      pNMEA2000_mcp = this;
    }
  }
  else {
    N2k_CAN_INT_PIN = 0xff;
  }
}

//*****************************************************************************
bool tNMEA2000_pico::CANSendFrame(unsigned long id, unsigned char len, const unsigned char *buf, bool wait_sent)
{
  struct can_frame tx;
  bool result;

  tx.can_id = id;
  // can only handle 8 byte payload so don't try to send more
  tx.can_dlc = len > MCP2515::CAN_MAX_CHAR_IN_MESSAGE ? MCP2515::CAN_MAX_CHAR_IN_MESSAGE : len;

  memcpy(tx.data ,buf, tx.can_dlc);

  // Also sending should be changed to be done by interrupt. This requires modifications for mcp_can.
  volatile tFrameBuffer *pTxBuf=0;
  if(UseInterrupt()) {
    uint32_t flags = save_and_disable_interrupts(); // disable interrupts

    pTxBuf=(wait_sent?pTxBufferFastPacket:pTxBuffer);
    // If buffer is not empty, it has pending messages, so add new message to it
    if (!pTxBuf->IsEmpty()) {
      result = pTxBuf->AddFrame(id,len,buf);
    } 
    else { // If we did not use buffer, send it directly
      DbgStartMcpSpeed;
      result = (N2kCAN.sendMessage(wait_sent ? (MCP2515::TXBn)N2kCAN.getLastTxBuffer() : (MCP2515::TXBn)0xff, &tx) == MCP2515::RETURN_OK);
      DbgEndMcpSpeed;
      if ( !result ) {
        DbgClearMcpSpeed;
        result = pTxBuf->AddFrame(id,len,buf);
      }
    }
    restore_interrupts(flags); // restore interrupts
  } 
  else {
    result = (N2kCAN.sendMessage(wait_sent ? (MCP2515::TXBn)N2kCAN.getLastTxBuffer() : (MCP2515::TXBn)0xff, &tx) == MCP2515::RETURN_OK);
    //result = (N2kCAN.trySendExtMsgBuf(id, len, buf, wait_sent ? N2kCAN.getLastTxBuffer() : 0xff) == MCP2515::ERROR_OK);
  }

  DbgTestMcpSpeed { DbgPrintN2kMcpSpeed("Send elapsed: "); DbgPrintLnN2kMcpSpeed(McpElapsed); }

  return result;
}

//*****************************************************************************
void tNMEA2000_pico::InitCANFrameBuffers() {
  if(UseInterrupt())
  {
    if (MaxCANReceiveFrames<2 ) MaxCANReceiveFrames = 2;
    if (MaxCANSendFrames<10 ) MaxCANSendFrames = 10;
    uint16_t CANGlobalBufSize = MaxCANSendFrames - 4;

    MaxCANSendFrames = 4;  // we do not need much libary internal buffer since driver has them.
    uint16_t FastPacketBufferSize = (CANGlobalBufSize * 9 / 10);
    CANGlobalBufSize -= FastPacketBufferSize;
    pRxBuffer = new tFrameBuffer(MaxCANReceiveFrames);
    pTxBuffer = new tFrameBuffer(CANGlobalBufSize);
    pTxBufferFastPacket = new tFrameBuffer(FastPacketBufferSize);
  }
  tNMEA2000::InitCANFrameBuffers(); // call main initialization
}

//*****************************************************************************
bool tNMEA2000_pico::CANOpen() {
  if (IsOpen) return true;

  if (CanInUse) return false; // currently prevent accidental second instance. Maybe possible in future.

  N2kCAN.reserveTxBuffers(1); // Reserve one buffer for fast packet.

  IsOpen = (N2kCAN.Begin() == MCP2515::RETURN_OK);

    if (IsOpen && UseInterrupt() ) {
      uint32_t flags = save_and_disable_interrupts(); // disable interrupts

      // Enable interrupt on falling edge
      gpio_init(N2k_CAN_INT_PIN);
      gpio_set_dir(N2k_CAN_INT_PIN, GPIO_IN);
      gpio_pull_up(N2k_CAN_INT_PIN);  // Optional pull-up resistor
      gpio_set_irq_enabled_with_callback(N2k_CAN_INT_PIN, GPIO_IRQ_EDGE_FALL, true, &Can1Interrupt);

      InterruptHandler(N2k_CAN_INT_PIN, GPIO_IRQ_EDGE_FALL); // read out possible data to clear isr bit.
      restore_interrupts(flags); // restore interrupts
    }

  CanInUse=IsOpen;

  return IsOpen;
}

//*****************************************************************************
bool tNMEA2000_pico::CANGetFrame(unsigned long &id, unsigned char &len, unsigned char *buf) {
  struct can_frame rx;
  bool HasFrame=false;

    if(UseInterrupt()) {
      uint32_t flags = save_and_disable_interrupts(); // disable interrupts
      HasFrame = pRxBuffer->GetFrame(id, len, buf);
      restore_interrupts(flags); // restore interrupts
    }
    else {
      if(MCP2515::RETURN_MSGAVAIL == N2kCAN.readRxStatus()) {    // check if data coming
        N2kCAN.readMessage(&rx);    // read data,  len: data length, buf: data buf

        id = rx.can_id;
        len = rx.can_dlc;
        memcpy(buf, rx.data, rx.can_dlc);

        HasFrame=true;
      }
    }

    // if (HasFrame) PrintDecodedCanIdAndLen(id,len);

    return HasFrame;
}

//*****************************************************************************
// I am still note sure am I handling volatile right here since mcp_can has not
// been defined volatile. see. http://blog.regehr.org/archives/28
// In my tests I have used only to receive data or transmit data but not both.
void tNMEA2000_pico::InterruptHandler(uint gpio, uint32_t events) {
#if defined(DEBUG_NMEA2000_ISR)
  uint64_t ISRStart = time_us_64();
#endif

  uint8_t RxTxStatus;

  // Iterate over all pending messages.
  // If either the bus is saturated or the MCU is busy, both RX buffers may be in use and
  // reading a single message does not clear the IRQ conditon.
  // Also we need to check and clear all transmit flags to clear IRQ condition.
  // Note that this handler expects that Wakeup and Error interrupts has not been enabled.
  do {
    RxTxStatus = N2kCAN.readRxTxStatus();  // One single read on every loop
    uint8_t tempRxTxStatus = RxTxStatus;   // Use local status inside loop
    uint8_t status;
    volatile can_frame *frame;

    while((status = N2kCAN.checkClearRxStatus(&tempRxTxStatus)) != 0) {           // check if data is coming
      if((frame = pRxBuffer->GetWriteFrame()) != 0) {
        N2kCAN.readMessage((MCP2515::RXBn)N2kCAN.statusToBuffer(status), (can_frame*)frame); // readMsgBufID(status, &(frame->can_id) ,&ext, &rtr, &(frame->can_dlc), frame->data);
        pRxBuffer->IncWrite();
//      asm volatile ("" : : : "memory");
        //N2kCAN.readMsgBuf(&len,buf);
        //id=N2kCAN.getCanId();
        //pRxBuffer->AddFrame(id,len,buf);
      }
      else { // Buffer full, skip frame
        can_frame *FrameToSkip;
        N2kCAN.readMessage((MCP2515::RXBn)N2kCAN.statusToBuffer(status), (can_frame*)FrameToSkip); // readMsgBufID(status, &(frame->can_id) ,&ext, &rtr, &(frame->can_dlc), frame->data);
      }
    }

    if(!pTxBufferFastPacket->IsEmpty()) { // Do we have something to send on fast packet frame buffer
      // CanIntChk=tempRxTxStatus;
      if ((status = N2kCAN.checkClearTxStatus(&tempRxTxStatus, N2kCAN.getLastTxBuffer())) != 0) {
        frame=pTxBufferFastPacket->GetReadFrame();
        N2kCAN.sendMessage((MCP2515::TXBn)N2kCAN.statusToBuffer(status), (can_frame*)frame);
        pTxBufferFastPacket->DecRead();
      }
    }
    else { // Nothing to send, so clear flag for this buffer
      status=N2kCAN.checkClearTxStatus(&tempRxTxStatus,N2kCAN.getLastTxBuffer());
      N2kCAN.clearBufferTransmitIfFlags(status);
    }

    if ( !pTxBuffer->IsEmpty() ) { // Do we have something to send on single frame buffer
      while ( (status=N2kCAN.checkClearTxStatus(&tempRxTxStatus)) != 0 && 
              (frame=pTxBuffer->GetReadFrame())!=0 ) {
                N2kCAN.sendMessage((MCP2515::TXBn)N2kCAN.statusToBuffer(status), (can_frame*)frame);
        pTxBuffer->DecRead();
      }
    } 

    // Finally clear rest transmit flags
    N2kCAN.clearBufferTransmitIfFlags(tempRxTxStatus);

  } while (RxTxStatus != 0);

#if defined(DEBUG_NMEA2000_ISR)
  ISRElapsed=micros()-ISRStart;
#endif
}

#if defined(DEBUG_NMEA2000_ISR)
//*****************************************************************************
void tNMEA2000_mcp::TestISR() {    // if ( CanIntChk ) { Serial.print("CAN int chk: "); Serial.println(CanIntChk); CanIntChk=0; }
    if ( ISRElapsed ) { printf("ISR Elapsed: "); printf("%lu\n", ISRElapsed); ISRElapsed = 0; }
}
#endif


// //*****************************************************************************
// #if defined(ESP8266)
// ICACHE_RAM_ATTR void Can1Interrupt() {
// #else
void Can1Interrupt(uint gpio, uint32_t events) {
// #endif
   pNMEA2000_mcp->InterruptHandler(gpio, events);
 }
