//Arduino NCR5380 Library
//Copyright 2020 Edward Halferty
// Portions of this file copied from Linux source, copyright various people such as:
// Linus Torvalds, Drew Eckhardt, Ray Van Tassle, Ingmar Baumgart, Ronald van Cuijlenborg, Alan Cox, and others.

#include "ncr5380.h"

//Constructor sets the pins to use for the NCR5380 connection
NCR5380::NCR5380(int cs_, int drq, int irq, int ior_, int ready, int dack_, int eop_, int reset_, int iow_, int a0,
                   int a1, int a2, int d0, int d1, int d2, int d3, int d4, int d5, int d6, int d7)
{ SET_PIN_NUMBERS(); }

void NCR5380::setLoggingEnabled(bool x) { loggingEnabled = x; }

void NCR5380::setScsiId(int x) { scsiId = x; }

void NCR5380::NCR5380_write(byte addr, byte data) {
  SET_ADDR(addr);
  SET_DATA(data);
  PULSE_WRITE_PINS();
}

byte NCR5380::NCR5380_read(byte addr) {
  SET_ADDR(addr);
  SET_DATA_DIRECTION(INPUT);
  delay(1);
  SET_READ_PINS();
  delay(1);
  byte res = (digitalRead(_d7) << 7) | (digitalRead(_d6) << 6) | (digitalRead(_d5) << 5) | (digitalRead(_d4) << 4) |
             (digitalRead(_d3) << 3) | (digitalRead(_d2) << 2) | (digitalRead(_d1) << 1) | digitalRead(_d0);
  delay(1);
  CLEAR_READ_PINS();
  SET_DATA_DIRECTION(OUTPUT);
  return res;
}

bool NCR5380::NCR5380_poll_politely(int reg1, byte bit1, byte val1) { return NCR5380_poll_politely2(reg1, bit1, val1, reg1, bit1, val1); }

bool NCR5380::NCR5380_poll_politely2(int reg1, byte bit1, byte val1, int reg2, byte bit2, byte val2) {
  for (int i = NUM_POLL_ITERATIONS; i > 0; i--) {
    if ((NCR5380_read(reg1) & bit1) == val1) return true;
    if ((NCR5380_read(reg2) & bit2) == val2) return true;
  }
  return false;
}

bool NCR5380::NCR5380_arbitrate() {
  if (loggingEnabled) { Serial.print("Trying to arbitrate. ID=");Serial.print(scsiId);Serial.print("\n"); }
  //Set the phase bits to 0, otherwise the NCR5380 won't drive the data bus during SELECTION.
  NCR5380_write(TARGET_COMMAND_REG, 0);
  NCR5380_write(OUTPUT_DATA_REG, ID_MASK);
  NCR5380_write(MODE_REG, MR_ARBITRATE);
  // Wait for BUS FREE phase
  bool ok = NCR5380_poll_politely2(MODE_REG, MR_ARBITRATE, 0, INITIATOR_COMMAND_REG, ICR_ARBITRATION_PROGRESS, ICR_ARBITRATION_PROGRESS);
  if (!ok) {
    if (loggingEnabled) { Serial.print("Arbitration timeout\n"); }
    return false;
  }
  //Check for lost arbitration
  if ((NCR5380_read(INITIATOR_COMMAND_REG) & ICR_ARBITRATION_LOST) ||
      (NCR5380_read(CURRENT_SCSI_DATA_REG) & ID_HIGHER_MASK) ||
      (NCR5380_read(INITIATOR_COMMAND_REG) & ICR_ARBITRATION_LOST))
  {
    if (loggingEnabled) { Serial.print("Lost arbitration. deasserting MR_ARBITRATE\n"); }
    return false;
  }
  //After/during arbitration, BSY should be asserted.
  NCR5380_write(INITIATOR_COMMAND_REG, ICR_ASSERT_SEL | ICR_ASSERT_BSY);
  if (loggingEnabled) { Serial.print("Won arbitration\n"); }
  delay(1);
  return true;
}

// Should be called right after arbitrate()
bool NCR5380::NCR5380_select(int targetId) {
  //Start selection process, asserting the host and target ID's on the SCSI bus
  NCR5380_write(OUTPUT_DATA_REG, ID_MASK | (1 << targetId));
  //Raise ATN while SEL is true before BSY goes false from arbitration, since this is the only way to guarantee that
  //we'll get a MESSAGE OUT phase immediately after selection.
  NCR5380_write(INITIATOR_COMMAND_REG, ICR_ASSERT_BSY | ICR_ASSERT_DATA | ICR_ASSERT_ATN | ICR_ASSERT_SEL);
  NCR5380_write(MODE_REG, 0);
  //Reselect interrupts must be turned off prior to the dropping of BSY, otherwise we will trigger an interrupt.
  NCR5380_write(SELECT_ENABLE_REG, 0);
  delay(1);
  //Reset BSY
  NCR5380_write(INITIATOR_COMMAND_REG, ICR_ASSERT_DATA | ICR_ASSERT_ATN | ICR_ASSERT_SEL);
  delay(1);
  if (loggingEnabled) { Serial.print("Selecting target ");Serial.print(targetId);Serial.print("\n"); }
  // TODO: SCSI spec call for a 250ms timeout for actual selection, so make this wait up to 250ms.
  bool ok = NCR5380_poll_politely(STATUS_REG, SR_BSY, SR_BSY);
  if (!ok) {
    if (loggingEnabled) { Serial.print("Selection problem?\n"); }
    return false;
  }
  delay(1);
  //No less than two deskew delays after the initiator detects the BSY signal is true, it shall release the SEL signal
  //and may change the DATA BUS. -wingel
  NCR5380_write(INITIATOR_COMMAND_REG, ICR_ASSERT_ATN);
  //Since we followed the SCSI spec, and raised ATN while SEL was true but before BSY was false during selection,
  //the information transfer phase should be a MESSAGE OUT phase so that we can send the IDENTIFY message.
  //Wait for start of REQ/ACK handshake
  ok = NCR5380_poll_politely(STATUS_REG, SR_REQ, SR_REQ);
  if (!ok) {
    if (loggingEnabled) { Serial.print("Select: REQ timeout\n"); }
    NCR5380_write(INITIATOR_COMMAND_REG, 0);
    return false;
  }
  if (loggingEnabled) { Serial.print("Target ");Serial.print(targetId);Serial.print(" selected. Going into MESSAGE OUT phase.\n"); }
  byte tmp[3];
  tmp[0] = IDENTIFY(false, 0);
  int len = 1;
  byte phase = PHASE_MSGOUT;
  byte *msgptr = tmp;
  NCR5380_transfer_pio(&phase, &len, &msgptr);
  if (len) {
    NCR5380_write(INITIATOR_COMMAND_REG, 0);
    if (loggingEnabled) { Serial.print("IDENTIFY message transfer failed\n"); }
    return false;
  }
  if (loggingEnabled) { Serial.print("Nexus established.\n"); }
  return true;
}

byte NCR5380::readCurrentScsiDataReg() { return NCR5380_read(CURRENT_SCSI_DATA_REG); }

//Initializes the device, resets the SCSI bus, etc.
void NCR5380::begin() {
    SET_INITIAL_PIN_DIRECTIONS();
    SET_INITIAL_PIN_VALUES();
    PULSE_RESET_PIN();
    RESET_BUS();
    CLEAR_INTERRUPT_CONDITIONS();
}

// int DataIn(BYTE *buf, int length)
// {
//   // フェーズ待ち
//   if (!WaitPhase(BUS::datain)) {
//     return -1;
//   }

//   // データ受信
//   return bus.ReceiveHandShake(buf, length);
// }

bool NCR5380::NCR5380_data_in(byte *buf, int count) {
  // Wait for data in phase
  byte phase = 0;
  do {
    phase = NCR5380_read(STATUS_REG) & PHASE_MASK;
  } while (phase != PHASE_DATAIN);
  //while (NCR5380_read(STATUS_REG) & PHASE_MASK != PHASE_DATAIN) {}
  int c = count;
  //byte phase = NCR5380_read(STATUS_REG) & PHASE_MASK;
  NCR5380_transfer_pio(&phase, &c, &buf);
  return (c == 0);
}

void NCR5380::test() {//NCR5380_main
  bool ok = NCR5380_arbitrate();
  if (!ok) {
    Serial.print("arbitrate()=");Serial.print(ok);Serial.print("\n");
    return;
  }
  ok = NCR5380_select(5);
  if (!ok) {
    Serial.print("select()=");Serial.print(ok);Serial.print("\n");
    return;
  }
  byte mm[8];
  mm[0] = 0x12;  // 0x12 = INQUIRY command
  mm[1] = 0;
  mm[2] = 0;
  mm[3] = 0;
  mm[4] = 0xff;
  mm[5] = 0;
  byte *b = mm;
  ok = NCR5380_command(b, 6);
  if (!ok) {
    Serial.print("NCR5380_command()=");Serial.print(ok);Serial.print("\n");
    return;
  }
  byte buf[256];
  for (int i = 0; i < 256; i++) {
    buf[i] = 0;
  }
  ok = NCR5380_data_in(buf, 256);
  if (!ok) {
    Serial.print("NCR5380_data_in()=");Serial.print(ok);Serial.print("\n");
    //return;
  }
  Serial.print("------------");
  for (int i = 0; i < 256; i++) {
    char x = buf[i];
    Serial.print(x);
    Serial.print(" ");
    if (i % 8 == 0) {
      Serial.print("\n");
    }
  }
  Serial.print("============");
}

bool NCR5380::NCR5380_transfer_pio(byte *phase, int *count, byte **data) {
  byte p = *phase, tmp;
  byte *d = *data;
  int c = *count;
  //The NCR5380 chip will only drive the SCSI bus when the phase specified in the appropriate bits of the TARGET COMMAND
  //REGISTER match the STATUS REGISTER
  NCR5380_write(TARGET_COMMAND_REG, PHASE_SR_TO_TCR(p));
  do {// Wait for assertion of REQ, after which the phase bits will be valid
    if (!NCR5380_poll_politely(STATUS_REG, SR_REQ, SR_REQ)) break;
    if (loggingEnabled) { Serial.print("REQ asserted\n"); }
    byte statusRegPhase = NCR5380_read(STATUS_REG) & PHASE_MASK;
    if (statusRegPhase != p) { //Check for phase mismatch
      if (loggingEnabled) {
        Serial.print("phase mismatch found=");
        Serial.print(statusRegPhase, HEX);
        Serial.print(" expected=");
        Serial.print(p, HEX);
        Serial.print("\n");
      }
      break;
    }
    //Do actual transfer from SCSI bus to / from memory
    if (!(p & SR_IO)) { NCR5380_write(OUTPUT_DATA_REG, *d); } else {*d = NCR5380_read(CURRENT_SCSI_DATA_REG); }
    ++d;
    //The SCSI standard suggests that in MSGOUT phase, the initiator should drop ATN on the last byte of the message
    //phase after REQ has been asserted for the handshake but before the initiator raises ACK.
    if (!(p & SR_IO)) {
      if (!((p & SR_MSG) && c > 1)) {
        NCR5380_write(INITIATOR_COMMAND_REG, ICR_ASSERT_DATA);
        NCR5380_write(INITIATOR_COMMAND_REG, ICR_ASSERT_DATA | ICR_ASSERT_ACK);
      } else {
        NCR5380_write(INITIATOR_COMMAND_REG, ICR_ASSERT_DATA | ICR_ASSERT_ATN);
        NCR5380_write(INITIATOR_COMMAND_REG, ICR_ASSERT_DATA | ICR_ASSERT_ATN | ICR_ASSERT_ACK);
      }
    } else {
      NCR5380_write(INITIATOR_COMMAND_REG, ICR_ASSERT_ACK);
    }
    if (!NCR5380_poll_politely(STATUS_REG, SR_REQ, 0)) break;
    if (loggingEnabled) { Serial.print("REQ negated, handshake complete\n"); }
    //We have several special cases to consider during REQ/ACK handshaking :
    //1.  We were in MSGOUT phase, and we are on the last byte of the message.  ATN must be dropped as ACK is dropped.
    //2.  We are in a MSGIN phase, and we are on the last byte of the message.  We must exit with ACK asserted, so that
    //the calling code may raise ATN before dropping ACK to reject the message.
    //3.  ACK and ATN are clear and the target may proceed as normal.
    if (!(p == PHASE_MSGIN && c == 1)) {
      if (p == PHASE_MSGOUT && c > 1) { NCR5380_write(INITIATOR_COMMAND_REG, ICR_ASSERT_ATN); }
      else { NCR5380_write(INITIATOR_COMMAND_REG, 0); }
    }
  } while (--c);
  if (loggingEnabled) { Serial.print("residual ");Serial.print(c);Serial.print("\n"); }
  *count = c;
  *data = d;
  tmp = NCR5380_read(STATUS_REG);
  //The phase read from the bus is valid if either REQ is (already) asserted or if ACK hasn't been released yet. The
  //latter applies if we're in MSG IN, DATA IN or STATUS and all bytes have been received.
  if ((tmp & SR_REQ) || ((tmp & SR_IO) && c == 0)) { *phase = tmp & PHASE_MASK; }
  else { *phase = PHASE_UNKNOWN; }
  return (!c || (*phase == p));
}

bool NCR5380::NCR5380_command(byte *buf, int count) {
  int c = count;
  byte phase = NCR5380_read(STATUS_REG) & PHASE_MASK;
  NCR5380_transfer_pio(&phase, &c, &buf);
  return (c == 0);
}

// bool NCR5380::NCR5380_inquiry(int id, byte *buf) {

// }
