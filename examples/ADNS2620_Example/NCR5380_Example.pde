#include <linux_ncr5380.h>
#include <ncr5380.h>

// NCR5380 pin mappings
#define CS_     2
#define DRQ     3
#define IRQ     4
#define IOR_    5
#define READY   6
#define DACK_   7
#define EOP_    8
#define RESET_  9
#define IOW_    10
#define A0      11
#define A1      12
#define A2      13
#define D0      14
#define D1      15
#define D2      16
#define D3      17
#define D4      18
#define D5      19
#define D6      20
#define D7      21

NCR5380 *ncr;

void setup() {
  Serial.begin(9600);
  ncr = new NCR5380(CS_, DRQ, IRQ, IOR_, READY, DACK_, EOP_, RESET_, IOW_, A0, A1, A2, D0, D1, D2, D3, D4, D5, D6, D7);
  ncr->begin();
  ncr->setLoggingEnabled(true);
  delay(100);
  ncr->test();
}

void loop() {
  delay(100);
  Serial.write("--------");
  byte res = ncr->readCurrentScsiDataReg();
  Serial.println(res, HEX);
}
