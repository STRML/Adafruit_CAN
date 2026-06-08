// Copyright 2020 © Jeff Epler for Adafruit Industries. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full
// license information.

#if defined(ADAFRUIT_FEATHER_M4_CAN)

#include <stdint.h>
#include <stdlib.h>

#include "CANSAME5x.h"
#include "wiring_private.h"

#define DEBUG_CAN (0)
#if DEBUG_CAN
#define DEBUG_PRINT(...) (Serial.print(__VA_ARGS__), ((void)0))
#define DEBUG_PRINTLN(...) (Serial.println(__VA_ARGS__), ((void)0))
#else
#define DEBUG_PRINT(...) ((void)0)
#define DEBUG_PRINTLN(...) ((void)0)
#endif

namespace {
#include "CANSAME5x_port.h"
}

#define hw (reinterpret_cast<Can *>(this->_hw))
#define state (reinterpret_cast<_canSAME5x_state *>(this->_state))

#define DIV_ROUND(a, b) (((a) + (b) / 2) / (b))
#define DIV_ROUND_UP(a, b) (((a) + (b) - 1) / (b))

#define GCLK_CAN1 GCLK_PCHCTRL_GEN_GCLK1_Val
#define GCLK_CAN0 GCLK_PCHCTRL_GEN_GCLK1_Val
// TX FIFO depth (MCAN Tx FIFO elements). Non-blocking sends queue here and the
// hardware drains them in submission order (TFQM=0). Override with a build flag
// if you want a shallower FIFO; defaults to the MCAN maximum of 32. Each
// element costs about 16 bytes of message RAM.
#ifndef ADAFRUIT_ZEROCAN_TX_FIFO_SIZE
#define ADAFRUIT_ZEROCAN_TX_FIFO_SIZE (32)
#endif
#if (ADAFRUIT_ZEROCAN_TX_FIFO_SIZE < 1) || (ADAFRUIT_ZEROCAN_TX_FIFO_SIZE > 32)
#error                                                                         \
    "ADAFRUIT_ZEROCAN_TX_FIFO_SIZE must be in 1..32 (MCAN total TX element cap)"
#endif
#define ADAFRUIT_ZEROCAN_RX_FILTER_SIZE (1)
#define ADAFRUIT_ZEROCAN_RX_FIFO_SIZE (8)
#define ADAFRUIT_ZEROCAN_MAX_MESSAGE_LENGTH (8)

namespace {

template <class T, std::size_t N>
constexpr size_t size(const T (&array)[N]) noexcept {
  return N;
}

// Adapted from ASF3 interrupt_sam_nvic.c:

volatile unsigned long cpu_irq_critical_section_counter = 0;
volatile unsigned char cpu_irq_prev_interrupt_state = 0;

void cpu_irq_enter_critical(void) {
  if (!cpu_irq_critical_section_counter) {
    if (__get_PRIMASK() == 0) { // IRQ enabled?
      __disable_irq();          // Disable it
      __DMB();
      cpu_irq_prev_interrupt_state = 1;
    } else {
      // Make sure the to save the prev state as false
      cpu_irq_prev_interrupt_state = 0;
    }
  }

  cpu_irq_critical_section_counter++;
}

void cpu_irq_leave_critical(void) {
  // Check if the user is trying to leave a critical section
  // when not in a critical section
  if (cpu_irq_critical_section_counter > 0) {
    cpu_irq_critical_section_counter--;

    // Only enable global interrupts when the counter
    // reaches 0 and the state of the global interrupt flag
    // was enabled when entering critical state */
    if ((!cpu_irq_critical_section_counter) && cpu_irq_prev_interrupt_state) {
      __DMB();
      __enable_irq();
    }
  }
}

// This appears to be a typo (transposition error) in the ASF4 headers
// It's called the "Extended ID Filter Entry"
typedef CanMramXifde CanMramXidfe;

typedef uint32_t can_filter_t;

struct _canSAME5x_tx_buf {
  CAN_TXBE_0_Type txb0;
  CAN_TXBE_1_Type txb1;
  __attribute__((aligned(4))) uint8_t data[8];
};

struct _canSAME5x_rx_fifo {
  CAN_RXF0E_0_Type rxf0;
  CAN_RXF0E_1_Type rxf1;
  __attribute((aligned(4))) uint8_t data[ADAFRUIT_ZEROCAN_MAX_MESSAGE_LENGTH];
} can_rx_fifo_t;
// Reference the type once so -Wunused-variable stays quiet under -Werror
// builds.
static inline void _suppress_unused_rx_fifo() { (void)sizeof(can_rx_fifo_t); }

struct _canSAME5x_state {
  _canSAME5x_tx_buf tx_buffer[ADAFRUIT_ZEROCAN_TX_FIFO_SIZE];
  _canSAME5x_rx_fifo rx_fifo[ADAFRUIT_ZEROCAN_RX_FIFO_SIZE];
  CanMramSidfe standard_rx_filter[ADAFRUIT_ZEROCAN_RX_FILTER_SIZE];
  CanMramXifde extended_rx_filter[ADAFRUIT_ZEROCAN_RX_FILTER_SIZE];
};

// This data must be in the first 64kB of RAM.  The "canram" section
// receives special support from the linker file in the Feather M4 CAN's
// board support package.
__attribute__((section(".canram"))) _canSAME5x_state can_state[2];

constexpr uint32_t can_frequency = VARIANT_GCLK1_FREQ;
bool compute_nbtp(uint32_t baudrate, CAN_NBTP_Type &result) {
  uint32_t clocks_per_bit = DIV_ROUND(can_frequency, baudrate);
  uint32_t clocks_to_sample = DIV_ROUND(clocks_per_bit * 7, 8);
  uint32_t clocks_after_sample = clocks_per_bit - clocks_to_sample;
  uint32_t divisor = max(DIV_ROUND_UP(clocks_to_sample, 256),
                         DIV_ROUND_UP(clocks_after_sample, 128));
  if (divisor > 32) {
    return false;
  }
  result.bit.NTSEG1 = DIV_ROUND(clocks_to_sample, divisor) - 2;
  result.bit.NTSEG2 = DIV_ROUND(clocks_after_sample, divisor) - 1;
  result.bit.NBRP = divisor - 1;
  result.bit.NSJW = DIV_ROUND(clocks_after_sample, divisor * 4);
  return true;
}

EPioType find_pin(const can_function *table, size_t n, int arduino_pin,
                  int &instance) {
  if (arduino_pin < 0 || arduino_pin >= (int)PINS_COUNT) {
    return (EPioType)-1;
  }

  unsigned port = g_APinDescription[arduino_pin].ulPort;
  unsigned pin = g_APinDescription[arduino_pin].ulPin;
  for (size_t i = 0; i < n; i++) {
    if (table[i].port == port && table[i].pin == pin) {
      if (instance == -1 || table[i].instance == instance) {
        DEBUG_PRINT("found #");
        DEBUG_PRINTLN(i);
        instance = table[i].instance;
        return EPioType(table[i].mux);
      }
    }
  }
  return (EPioType)-1;
}

} // namespace

CANSAME5x::CANSAME5x(uint8_t TX_PIN, uint8_t RX_PIN)
    : _tx(TX_PIN), _rx(RX_PIN) {}
#ifdef PIN_CAN_TX
CANSAME5x::CANSAME5x() : _tx(PIN_CAN_TX), _rx(PIN_CAN_RX) {}
#else
CANSAME5x::CANSAME5x() : _tx(-1) {}
#endif

CANSAME5x::~CANSAME5x() {}

int CANSAME5x::begin(long baudrate) {
  if (_tx == -1) {
    return 0;
  }

  DEBUG_PRINT("_rx ");
  DEBUG_PRINT(_rx);
  DEBUG_PRINT(" ulPort=");
  DEBUG_PRINT(g_APinDescription[_rx].ulPort);
  DEBUG_PRINT(" ulPin=");
  DEBUG_PRINTLN(g_APinDescription[_rx].ulPin);

  DEBUG_PRINTLN("rx pin table");
  for (size_t i = 0; i < size(can_rx); i++) {
    DEBUG_PRINT(i);
    DEBUG_PRINT(" port=");
    DEBUG_PRINT(can_rx[i].port);
    DEBUG_PRINT(" pin=");
    DEBUG_PRINT(can_rx[i].pin);
    DEBUG_PRINT(" instance=");
    DEBUG_PRINTLN(can_rx[i].instance);
  }

  DEBUG_PRINT("_tx ");
  DEBUG_PRINT(_tx);
  DEBUG_PRINT(" ulPort=");
  DEBUG_PRINT(g_APinDescription[_tx].ulPort);
  DEBUG_PRINT(" ulPin=");
  DEBUG_PRINTLN(g_APinDescription[_tx].ulPin);

  DEBUG_PRINTLN("tx pin table");
  for (size_t i = 0; i < size(can_tx); i++) {
    DEBUG_PRINT(i);
    DEBUG_PRINT(" port=");
    DEBUG_PRINT(can_tx[i].port);
    DEBUG_PRINT(" pin=");
    DEBUG_PRINT(can_tx[i].pin);
    DEBUG_PRINT(" instance=");
    DEBUG_PRINTLN(can_tx[i].instance);
  }

  int instance = -1;
  EPioType tx_function = find_pin(can_tx, size(can_tx), _tx, instance);
  EPioType rx_function = find_pin(can_rx, size(can_rx), _rx, instance);

  if (tx_function == EPioType(-1) || rx_function == EPioType(-1) ||
      instance == -1) {
    return 0;
  }

  CAN_NBTP_Type nbtp;
  if (!compute_nbtp(baudrate, nbtp)) {
    return 0;
  }

  _idx = instance;
  _hw = reinterpret_cast<void *>(_idx == 0 ? CAN0 : CAN1);
  _state = reinterpret_cast<void *>(&can_state[_idx]);

  memset((void *)state, 0, sizeof(*state)); // NOLINT: trivial POD-like struct

  pinPeripheral(_tx, tx_function);
  pinPeripheral(_rx, rx_function);

  if (_idx == 0) {
    GCLK->PCHCTRL[CAN0_GCLK_ID].reg = GCLK_CAN0 | (1 << GCLK_PCHCTRL_CHEN_Pos);
  } else {
    GCLK->PCHCTRL[CAN1_GCLK_ID].reg = GCLK_CAN1 | (1 << GCLK_PCHCTRL_CHEN_Pos);
  }
  // reset and allow configuration change
  hw->CCCR.bit.INIT = 1;
  while (!hw->CCCR.bit.INIT) {
  }
  hw->CCCR.bit.CCE = 1;

  // All TX data has an 8 byte payload (max)
  {
    CAN_TXESC_Type esc = {};
    esc.bit.TBDS = CAN_TXESC_TBDS_DATA8_Val;
    hw->TXESC.reg = esc.reg;
  }

  // Set up TX FIFO (not a dedicated buffer). Non-blocking sends push to the
  // FIFO put index and the hardware transmits in submission order. NDTB=0 (no
  // dedicated buffers), TFQS=depth, TFQM=0 (FIFO order, not Queue/priority
  // mode).
  {
    CAN_TXBC_Type bc = {};
    bc.bit.TBSA = (uint32_t)state->tx_buffer;
    bc.bit.NDTB = 0;
    bc.bit.TFQS = ADAFRUIT_ZEROCAN_TX_FIFO_SIZE;
    bc.bit.TFQM =
        0; // 0 = FIFO mode: messages transmitted in the order submitted
    hw->TXBC.reg = bc.reg;
  }

  // All RX data has an 8 byte payload (max)
  {
    CAN_RXESC_Type esc = {};
    esc.bit.F0DS = CAN_RXESC_F0DS_DATA8_Val;
    esc.bit.F1DS = CAN_RXESC_F1DS_DATA8_Val;
    esc.bit.RBDS = CAN_RXESC_RBDS_DATA8_Val;
    hw->RXESC.reg = esc.reg;
  }

  // Set up RX fifo 0
  {
    CAN_RXF0C_Type rxf = {};
    rxf.bit.F0SA = (uint32_t)state->rx_fifo;
    rxf.bit.F0S = ADAFRUIT_ZEROCAN_RX_FIFO_SIZE;
    hw->RXF0C.reg = rxf.reg;
  }

  // Reject all packets not explicitly requested
  {
    CAN_GFC_Type gfc = {};
    gfc.bit.RRFE = 0;
    gfc.bit.ANFS = CAN_GFC_ANFS_REJECT_Val;
    gfc.bit.ANFE = CAN_GFC_ANFE_REJECT_Val;
    hw->GFC.reg = gfc.reg;
  }

  // Initially, receive all standard and extended packets to FIFO 0
  state->standard_rx_filter[0].SIDFE_0.bit.SFID1 = 0; // ID
  state->standard_rx_filter[0].SIDFE_0.bit.SFID2 = 0; // mask
  state->standard_rx_filter[0].SIDFE_0.bit.SFEC = CAN_SIDFE_0_SFEC_STF0M_Val;
  state->standard_rx_filter[0].SIDFE_0.bit.SFT = CAN_SIDFE_0_SFT_CLASSIC_Val;

  state->extended_rx_filter[0].XIDFE_0.bit.EFID1 = 0; // ID
  state->extended_rx_filter[0].XIDFE_0.bit.EFEC = CAN_XIDFE_0_EFEC_STF0M_Val;
  state->extended_rx_filter[0].XIDFE_1.bit.EFID2 = 0; // mask
  state->extended_rx_filter[0].XIDFE_1.bit.EFT = CAN_XIDFE_1_EFT_CLASSIC_Val;

  // Set up standard RX filters
  {
    CAN_SIDFC_Type dfc = {};
    dfc.bit.LSS = ADAFRUIT_ZEROCAN_RX_FILTER_SIZE;
    dfc.bit.FLSSA = (uint32_t)state->standard_rx_filter;
    hw->SIDFC.reg = dfc.reg;
  }

  // Set up extended RX filters
  {
    CAN_XIDFC_Type dfc = {};
    dfc.bit.LSE = ADAFRUIT_ZEROCAN_RX_FILTER_SIZE;
    dfc.bit.FLESA = (uint32_t)state->extended_rx_filter;
    hw->XIDFC.reg = dfc.reg;
  }

  // Enable receive IRQ (masked until enabled in NVIC)
  hw->IE.bit.RF0NE = true;
  if (_idx == 0) {
    hw->ILE.bit.EINT0 = true;
  } else {
    hw->ILE.bit.EINT1 = true;
  }
  hw->ILS.bit.RF0NL = _idx;

  // Set nominal baud rate
  hw->NBTP.reg = nbtp.reg;

  // hardware is ready for use
  hw->CCCR.bit.CCE = 0;
  hw->CCCR.bit.INIT = 0;
  while (hw->CCCR.bit.INIT) {
  }

  instances[_idx] = this;

  return 1;
}

void CANSAME5x::end() {
  instances[_idx] = 0;
  pinMode(_tx, INPUT);
  pinMode(_rx, INPUT);
  // reset and disable clock
  hw->CCCR.bit.INIT = 1;
  while (!hw->CCCR.bit.INIT) {
  }
  if (_idx == 0) {
    GCLK->PCHCTRL[CAN0_GCLK_ID].reg = 0;
  } else {
    GCLK->PCHCTRL[CAN1_GCLK_ID].reg = 0;
  }
}

// Push one frame into the TX FIFO. Non-blocking: stages the frame at the
// hardware put index and sets the matching TXBAR add-request bit, then returns.
// The hardware transmits asynchronously in FIFO order. Returns the put index
// used (>= 0) or -1 if the frame could not be accepted (FIFO full, or the
// controller is in INIT/bus-off and would silently ignore the request). NEVER
// waits for transmission to occur.
int CANSAME5x::_pushTxFifo(uint32_t id, bool extended, bool rtr,
                           const uint8_t *data, uint8_t dlc) {
  // Refuse if the controller can't transmit right now. During INIT (set on
  // bus-off and before begin() completes) a TXBAR write is ignored by the
  // hardware, so accepting the frame here would silently drop it. Return -1
  // instead and let the caller recover the bus and retry.
  if (hw->CCCR.bit.INIT) {
    return -1;
  }
  // FIFO full: drop and let the caller account for it. This is the load-shed
  // path when the bus can't drain, and the only place a healthy system drops a
  // frame.
  if (hw->TXFQS.bit.TFQF) {
    return -1;
  }

  if (dlc > 8) {
    dlc = 8;
  }

  const uint32_t idx = hw->TXFQS.bit.TFQPI; // hardware-assigned put index
  _canSAME5x_tx_buf &buf = state->tx_buffer[idx];
  buf.txb0.bit.ESI = false;
  buf.txb0.bit.XTD = extended;
  buf.txb0.bit.RTR = rtr;
  if (extended) {
    buf.txb0.bit.ID = id;
  } else {
    buf.txb0.bit.ID = id << 18;
  }
  buf.txb1.bit.MM = 0;
  buf.txb1.bit.EFC = 0;
  buf.txb1.bit.FDF = 0;
  buf.txb1.bit.BRS = 0;
  buf.txb1.bit.DLC = dlc;
  // TX FIFO slots are reused across sends; a data frame must overwrite the
  // whole payload so a null-data send can't transmit a previous frame's stale
  // bytes.
  if (!rtr) {
    if (data != nullptr) {
      memcpy(buf.data, data, dlc);
    } else if (dlc > 0) {
      memset(buf.data, 0, dlc);
    }
  }

  // Request transmission of exactly this element. TXBAR is write-1-to-set, so
  // write a fresh single-bit mask rather than a read-modify-write (an RMW could
  // re-request an element that is already draining).
  hw->TXBAR.reg = (1u << idx);
  return (int)idx;
}

// Whole-frame non-blocking send (see issue #8). Fills and queues a frame in one
// call so callers don't need the beginPacket()/write()/endPacket() dance.
// Returns 1 if queued, 0 if dropped (FIFO full or controller not ready).
int CANSAME5x::sendFrame(uint32_t id, const uint8_t *data, uint8_t dlc,
                         bool extended, bool rtr) {
  return _pushTxFifo(id, extended, rtr, data, dlc) >= 0 ? 1 : 0;
}

// Non-blocking drop-in for the byte-at-a-time beginPacket()/write()/endPacket()
// flow. Validates the staged frame, then pushes it to the FIFO without waiting.
// Returns 1 if queued, 0 if dropped.
int CANSAME5x::endPacketAsync() {
  if (!CANControllerClass::endPacket()) {
    return 0;
  }
  return _pushTxFifo(_txId, _txExtended, _txRtr, _txData, _txLength) >= 0 ? 1
                                                                          : 0;
}

// The old endPacket() busy-waited up to ~8 ms for the frame to transmit,
// polling TXBTO with yield() in the loop. On a marginal bus that never ACKs,
// that stalled the caller for the full timeout on every send. Under FIFO mode
// there's no single dedicated buffer left to poll, so endPacket() now just
// delegates to the non-blocking path. Kept for API compatibility; new code
// should prefer sendFrame() or endPacketAsync().
int CANSAME5x::endPacket() { return endPacketAsync(); }

// True when the TX FIFO has no free element (next send would drop).
bool CANSAME5x::txFifoFull() { return hw->TXFQS.bit.TFQF; }

// Number of frames queued in the TX FIFO but not yet transmitted. TXFQS.TFFL is
// the "Tx FIFO Free Level" from the SAME51 datasheet (count of FREE elements,
// 0..TFQS), so pending = configured depth - free. Empty FIFO gives TFFL==depth
// (pending 0); full FIFO gives TFFL==0 (pending==depth). Handy before end() to
// know how many frames a reset would discard.
uint8_t CANSAME5x::txFifoPending() {
  return (uint8_t)(ADAFRUIT_ZEROCAN_TX_FIFO_SIZE - hw->TXFQS.bit.TFFL);
}

int CANSAME5x::_parsePacket() {
  if (!hw->RXF0S.bit.F0FL) {
    return 0;
  }

  int index = hw->RXF0S.bit.F0GI;
  auto &hw_message = state->rx_fifo[index];

  _rxExtended = hw_message.rxf0.bit.XTD;
  _rxRtr = hw_message.rxf0.bit.RTR;
  _rxDlc = hw_message.rxf1.bit.DLC;

  if (_rxExtended) {
    _rxId = hw_message.rxf0.bit.ID;
  } else {
    _rxId = hw_message.rxf0.bit.ID >> 18;
  }

  if (_rxRtr) {
    _rxLength = 0;
  } else {
    _rxLength = _rxDlc;
    memcpy(_rxData, hw_message.data, _rxLength);
  }

  _rxIndex = 0;

  hw->RXF0A.bit.F0AI = index;

  return _rxDlc;
}

int CANSAME5x::parsePacket() {
  cpu_irq_enter_critical();
  bus_autorecover();
  int result = _parsePacket();
  cpu_irq_leave_critical();
  return result;
}

void CANSAME5x::onReceive(void (*callback)(int)) {
  CANControllerClass::onReceive(callback);

  auto irq = _idx == 0 ? CAN0_IRQn : CAN1_IRQn;
  if (callback) {
    NVIC_EnableIRQ(irq);
  } else {
    NVIC_DisableIRQ(irq);
  }
}

void CANSAME5x::handleInterrupt() {
  uint32_t ir = hw->IR.reg;

  if (ir & CAN_IR_RF0N) {
    while (int i = parsePacket())
      _onReceive(i);
  }

  hw->IR.reg = ir;
}

int CANSAME5x::filter(int id, int mask) {
  // accept matching standard messages
  state->standard_rx_filter[0].SIDFE_0.bit.SFID1 = id;
  state->standard_rx_filter[0].SIDFE_0.bit.SFID2 = mask;
  state->standard_rx_filter[0].SIDFE_0.bit.SFEC = CAN_SIDFE_0_SFEC_STF0M_Val;
  state->standard_rx_filter[0].SIDFE_0.bit.SFT = CAN_SIDFE_0_SFT_CLASSIC_Val;

  // reject all extended messages
  state->extended_rx_filter[0].XIDFE_0.bit.EFID1 = 0; // ID
  state->extended_rx_filter[0].XIDFE_0.bit.EFEC = CAN_XIDFE_0_EFEC_REJECT_Val;
  state->extended_rx_filter[0].XIDFE_1.bit.EFID2 = 0; // mask
  state->extended_rx_filter[0].XIDFE_1.bit.EFT = CAN_XIDFE_1_EFT_CLASSIC_Val;

  return 1;
}

int CANSAME5x::filterExtended(long id, long mask) {
  // reject all standard messages
  state->standard_rx_filter[0].SIDFE_0.bit.SFID1 = 0;
  state->standard_rx_filter[0].SIDFE_0.bit.SFID2 = 0;
  state->standard_rx_filter[0].SIDFE_0.bit.SFEC = CAN_SIDFE_0_SFEC_REJECT_Val;
  state->standard_rx_filter[0].SIDFE_0.bit.SFT = CAN_SIDFE_0_SFT_CLASSIC_Val;

  // accept matching extended messages
  state->extended_rx_filter[0].XIDFE_0.bit.EFID1 = id;
  state->extended_rx_filter[0].XIDFE_0.bit.EFEC = CAN_XIDFE_0_EFEC_STF0M_Val;
  state->extended_rx_filter[0].XIDFE_1.bit.EFID2 = mask;
  state->extended_rx_filter[0].XIDFE_1.bit.EFT = CAN_XIDFE_1_EFT_CLASSIC_Val;

  return 1;
}

int CANSAME5x::observe() {
  hw->CCCR.bit.INIT = 1;
  while (!hw->CCCR.bit.INIT) {
  }
  hw->CCCR.bit.CCE = 1;

  hw->CCCR.bit.MON = 1;

  hw->CCCR.bit.CCE = 0;
  hw->CCCR.bit.INIT = 0;
  while (hw->CCCR.bit.INIT) {
  }
  return 1;
}

int CANSAME5x::loopback() {
  hw->CCCR.bit.INIT = 1;
  while (!hw->CCCR.bit.INIT) {
  }
  hw->CCCR.bit.CCE = 1;

  hw->CCCR.bit.TEST = 1;
  hw->TEST.bit.LBCK = 1;

  hw->CCCR.bit.CCE = 0;
  hw->CCCR.bit.INIT = 0;
  while (hw->CCCR.bit.INIT) {
  }
  return 1;
}

int CANSAME5x::sleep() {
  hw->CCCR.bit.CSR = 1;
  while (!hw->CCCR.bit.CSA) {
  }
  if (_idx == 0) {
    GCLK->PCHCTRL[CAN0_GCLK_ID].reg = 0;
  } else {
    GCLK->PCHCTRL[CAN1_GCLK_ID].reg = 0;
  }
  return 1;
}

int CANSAME5x::wakeup() {
  if (_idx == 0) {
    GCLK->PCHCTRL[CAN0_GCLK_ID].reg = GCLK_CAN0 | (1 << GCLK_PCHCTRL_CHEN_Pos);
  } else {
    GCLK->PCHCTRL[CAN1_GCLK_ID].reg = GCLK_CAN1 | (1 << GCLK_PCHCTRL_CHEN_Pos);
  }
  hw->CCCR.bit.INIT = 0;
  while (hw->CCCR.bit.INIT) {
  }
  return 1;
}

void CANSAME5x::bus_autorecover() {
  if (hw->PSR.bit.BO) {
    DEBUG_PRINTLN("bus autorecovery activated");
    hw->CCCR.bit.INIT = 0;
    // Bounded wait. The original while (INIT) {} spins forever if the bus stays
    // down (no recessive bits to clock INIT back to 0), which hangs the whole
    // sketch. 1000 iterations is roughly 10us at 120MHz, well under any
    // reasonable watchdog. If it's still set the bus is down; the caller can
    // retry recovery later instead of blocking here.
    for (int i = 0; i < 1000; i++) {
      if (!hw->CCCR.bit.INIT)
        return;
    }
  }
}

void CANSAME5x::onInterrupt() {
  for (size_t i = 0; i < size(instances); i++) {
    CANSAME5x *instance = instances[i];
    if (instance) {
      instance->handleInterrupt();
    }
  }
}

extern "C" __attribute__((externally_visible)) void CAN0_Handler() {
  cpu_irq_enter_critical();
  CANSAME5x::onInterrupt();
  cpu_irq_leave_critical();
}

extern "C" __attribute__((externally_visible)) void CAN1_Handler() {
  cpu_irq_enter_critical();
  CANSAME5x::onInterrupt();
  cpu_irq_leave_critical();
}

CANSAME5x *CANSAME5x::instances[2];

#endif
