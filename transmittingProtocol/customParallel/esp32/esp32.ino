#include "soc/gpio_reg.h"
#include "RS_FEC.h" // Switched to quotes to match your tab structure

const int pins[8] = {16, 17, 18, 19, 21, 22, 23, 25};
const int CLOCK_PIN = 4;

uint32_t gpio_lookup[256];
const uint32_t DATA_MASK = (1<<16)|(1<<17)|(1<<18)|(1<<19)|(1<<21)|(1<<22)|(1<<23)|(1<<25);
const uint32_t CLOCK_MASK = (1<<4);

// ADJUSTED FOR 8-BIT LIMITS: 245 Data + 10 Parity = 255 Total Block Size
const int msglen = 245;
const int ECC_LEN = 10;
uint8_t raw_msg[msglen];
uint8_t encoded_msg[msglen + ECC_LEN];

RS::ReedSolomon<msglen, ECC_LEN> rs;

void setup() {
for(int i=0; i<8; i++) pinMode(pins[i], OUTPUT);
pinMode(CLOCK_PIN, OUTPUT);
digitalWrite(CLOCK_PIN, LOW);

for(int i = 0; i < 256; i++) {
uint32_t val = 0;
if(i & (1 << 0)) val |= (1 << 16);
if(i & (1 << 1)) val |= (1 << 17);
if(i & (1 << 2)) val |= (1 << 18);
if(i & (1 << 3)) val |= (1 << 19);
if(i & (1 << 4)) val |= (1 << 21);
if(i & (1 << 5)) val |= (1 << 22);
if(i & (1 << 6)) val |= (1 << 23);
if(i & (1 << 7)) val |= (1 << 25);
gpio_lookup[i] = val;
}
}

#define SETTLE_DELAY() { for(volatile int x=0; x<20; x++) __asm__ __volatile__("nop"); }

void sendByteFast(uint8_t data) {
REG_WRITE(GPIO_OUT_W1TC_REG, DATA_MASK);
REG_WRITE(GPIO_OUT_W1TS_REG, gpio_lookup[data]);
SETTLE_DELAY();
REG_WRITE(GPIO_OUT_W1TS_REG, CLOCK_MASK);
SETTLE_DELAY();
REG_WRITE(GPIO_OUT_W1TC_REG, CLOCK_MASK);
SETTLE_DELAY();
}

void loop() {
// Populate our 245 data array positions
for(int i = 0; i < msglen; i++) {
raw_msg[i] = i; // Will cycle up to 244
}

rs.Encode(raw_msg, encoded_msg);

// Send the newly sized 255-byte total burst
for(int i = 0; i < (msglen + ECC_LEN); i++) {
sendByteFast(encoded_msg[i]);
}

delay(2000);
} 
