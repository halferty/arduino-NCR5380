/* Arduino NCR5380 Library
 * Copyright 2020 Edward Halferty
 * Portions of this file copied from Linux source, copyright Linus Torvalds and many others
*/

#include "ncr5380.h"

//Constructor sets the pins to use for the NCR5380 connection
NCR5380::NCR5380(int cs_, int drq, int irq, int ior_, int ready, int dack_, int eop_, int reset_, int iow_, int a0,
                   int a1, int a2, int d0, int d1, int d2, int d3, int d4, int d5, int d6, int d7)
{ SET_PIN_NUMBERS(); }

void NCR5380::setLoggingEnabled(bool x) { loggingEnabled = x; }

void NCR5380::setScsiId(int x) { scsiId = x; }

void NCR5380::write(byte addr, byte data) {
  SET_ADDR(addr);
  SET_DATA(data);
  PULSE_WRITE_PINS();
}

byte NCR5380::read(byte addr) {
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

bool NCR5380::poll(int reg1, byte bit1, byte val1) { return poll2(reg1, bit1, val1, reg1, bit1, val1); }

bool NCR5380::poll2(int reg1, byte bit1, byte val1, int reg2, byte bit2, byte val2) {
  for (int i = NUM_POLL_ITERATIONS; i > 0; i--) {
    if ((NCR5380_read(reg1) & bit1) == val1) return true;
    if ((NCR5380_read(reg2) & bit2) == val2) return true;
  }
  return false;
}

bool NCR5380::arbitrate() {
  if (loggingEnabled) { Serial.print("Trying to arbitrate. ID=");Serial.print(scsiId);Serial.print("\n"); }
  //Set the phase bits to 0, otherwise the NCR5380 won't drive the data bus during SELECTION.
  NCR5380_write(TARGET_COMMAND_REG, 0);
  NCR5380_write(OUTPUT_DATA_REG, ID_MASK);
  NCR5380_write(MODE_REG, MR_ARBITRATE);
  // Wait for BUS FREE phase
  bool ok = poll2(MODE_REG, MR_ARBITRATE, 0, INITIATOR_COMMAND_REG, ICR_ARBITRATION_PROGRESS, ICR_ARBITRATION_PROGRESS);
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
bool NCR5380::select(int targetId) {
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
  bool ok = poll(STATUS_REG, SR_BSY, SR_BSY);
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
  ok = poll(STATUS_REG, SR_REQ, SR_REQ);
  if (!ok) {
    if (loggingEnabled) { Serial.print("Select: REQ timeout\n"); }
    NCR5380_write(INITIATOR_COMMAND_REG, 0);
    return false;
  }
  if (loggingEnabled) { Serial.print("Target");Serial.print(targetId);Serial.print("selected. Going into MESSAGE OUT phase.\n"); }

  return true;
}

byte NCR5380::readCurrentScsiDataReg() { return read(CURRENT_SCSI_DATA_REG); }

//Initializes the device, resets the SCSI bus, etc.
void NCR5380::begin() {
    SET_INITIAL_PIN_DIRECTIONS();
    SET_INITIAL_PIN_VALUES();
    PULSE_RESET_PIN();
    RESET_BUS();
    CLEAR_INTERRUPT_CONDITIONS();
}

void NCR5380::test() {
  bool ok = arbitrate();
  if (!ok) {
    Serial.print("arbitrate()=");Serial.print(ok);Serial.print("\n");
    return;
  }
  ok = select(5);
  if (!ok) {
    Serial.print("select()=");Serial.print(ok);Serial.print("\n");
    return;
  }
}


// //Essentially resets communication to the ADNS2620 module
// void ADNS2620::sync()
// {
//     digitalWrite(_scl, HIGH);
//     delay(1);
// 	digitalWrite(_scl, LOW);
//     delay(1);
// 	digitalWrite(_scl, HIGH);
//     delay(100);
// }

// //Reads a register from the ADNS2620 sensor. Returns the result to the calling function.
// //Example: value = mouse.read(CONFIGURATION_REG);
// char ADNS2620::read(char address)
// {
//     char value=0;
// 	pinMode(_sda, OUTPUT); //Make sure the SDIO pin is set as an output.
//     digitalWrite(_scl, HIGH); //Make sure the clock is high.
//     address &= 0x7F;    //Make sure the highest bit of the address byte is '0' to indicate a read.
 
//     //Send the Address to the ADNS2620
//     for(int address_bit=7; address_bit >=0; address_bit--){
//         digitalWrite(_scl, LOW);  //Lower the clock
//         pinMode(_sda, OUTPUT); //Make sure the SDIO pin is set as an output.
        
//         //If the current bit is a 1, set the SDIO pin. If not, clear the SDIO pin
//         if(address & (1<<address_bit)){
//             digitalWrite(_sda, HIGH);
//         }
//         else{
//             digitalWrite(_sda, LOW);
//         }
//         delayMicroseconds(10);
//         digitalWrite(_scl, HIGH);
//         delayMicroseconds(10);
//     }
    
//     delayMicroseconds(120);   //Allow extra time for ADNS2620 to transition the SDIO pin (per datasheet)
//     //Make SDIO an input on the microcontroller
//     pinMode(_sda, INPUT);	//Make sure the SDIO pin is set as an input.
// 	digitalWrite(_sda, HIGH); //Enable the internal pull-up
        
//     //Send the Value byte to the ADNS2620
//     for(int value_bit=7; value_bit >= 0; value_bit--){
//         digitalWrite(_scl, LOW);  //Lower the clock
//         delayMicroseconds(10); //Allow the ADNS2620 to configure the SDIO pin
//         digitalWrite(_scl, HIGH);  //Raise the clock
//         delayMicroseconds(10);
//         //If the SDIO pin is high, set the current bit in the 'value' variable. If low, leave the value bit default (0).    
// 		//if((ADNS_PIN & (1<<ADNS_sda)) == (1<<ADNS_sda))value|=(1<<value_bit);
// 		if(digitalRead(_sda))value |= (1<<value_bit);

//     }
    
//     return value;
// }	

// //Writes a value to a register on the ADNS2620.
// //Example: mouse.write(CONFIGURATION_REG, 0x01);
// void ADNS2620::write(char address, char value)
// {
// 	pinMode(_sda, OUTPUT);	//Make sure the SDIO pin is set as an output.
//     digitalWrite(_scl, HIGH);          //Make sure the clock is high.
//     address |= 0x80;    //Make sure the highest bit of the address byte is '1' to indicate a write.

//     //Send the Address to the ADNS2620
//     for(int address_bit=7; address_bit >=0; address_bit--){
//         digitalWrite(_scl, LOW); //Lower the clock
        
//         delayMicroseconds(10); //Give a small delay (only needed for the first iteration to ensure that the ADNS2620 relinquishes
//                     //control of SDIO if we are performing this write after a 'read' command.
        
//         //If the current bit is a 1, set the SDIO pin. If not, clear the SDIO pin
//         if(address & (1<<address_bit))digitalWrite(_sda, HIGH);
//         else digitalWrite(_sda, LOW);
//         delayMicroseconds(10);
//         digitalWrite(_scl, HIGH);
//         delayMicroseconds(10);
//     }
    
//     //Send the Value byte to the ADNS2620
//     for(int value_bit=7; value_bit >= 0; value_bit--){
//         digitalWrite(_scl, LOW);  //Lower the clock
//         //If the current bit is a 1, set the SDIO pin. If not, clear the SDIO pin
//         if(value & (1<<value_bit))digitalWrite(_sda, HIGH);
//         else digitalWrite(_sda, LOW);
//         delayMicroseconds(10);
//         digitalWrite(_scl, HIGH);
//         delayMicroseconds(10);
//     }
// }