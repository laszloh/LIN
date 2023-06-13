/*  Copyright (c) 2016 Macchina
 *
 *  Permission is hereby granted, free of charge, to any person obtaining
 *  a copy of this software and associated documentation files (the
 *  "Software"), to deal in the Software without restriction, including
 *  without limitation the rights to use, copy, modify, merge, publish,
 *  distribute, sublicense, and/or sell copies of the Software, and to
 *  permit persons to whom the Software is furnished to do so, subject to
 *  the following conditions:
 *
 *  The above copyright notice and this permission notice shall be included
 *  in all copies or substantial portions of the Software.
 *
 *  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 *  EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 *  MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 *  IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
 *  CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 *  TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
 *  SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 *  LIN STACK for TJA1021
 *  v2.0
 *
 *  Short description:
 *  Comunication stack for LIN and TJA1021 LIN transceiver.
 *  Can be modified for any Arduino board with UART available and any LIN slave.
 *
 *  Author: Blaž Pongrac B.S., RoboSap, Institute of Technology, Ptuj (www.robosap-institut.eu)
 *
 *  Arduino IDE 1.6.9
 *  RoboSap, Institute of Technology, September 2016
 */

#include <lin_stack.h>
#include <avr/sfr_defs.h>

/* LIN PACKET:
   It consist of:
    ___________ __________ _______ ____________ _________
   |           |          |       |            |         |
   |Synch Break|Synch Byte|ID byte| Data Bytes |Checksum |
   |___________|__________|_______|____________|_________|

   Every byte have start bit and stop bit and it is send LSB first.
   Synch Break - min 13 bits of dominant state ("0"), followed by 1 bit recesive state ("1")
   Synch Byte - Byte for Bound rate syncronization, always 0x55
   ID Byte - consist of parity, length and address; parity is determined by LIN standard and depends from
   address and message length Data Bytes - user defined; depend on devices on LIN bus Checksum - inverted 256
   checksum; data bytes are sumed up and then inverted
*/

// CONSTRUCTORS
lin_stack::lin_stack(HardwareSerial &_channel, uint16_t _baud, int8_t _wake_pin, uint8_t _ident)
    : baud(_baud), channel(_channel), ident(_ident), wake_pin(_wake_pin) {
    if(wake_pin >= 0)
        sleep_config(); // Configurating Sleep pin for transceiver
}

// PUBLIC METHODS
// WRITE methods
// Creates a LIN packet and then send it via USART(Serial) interface.
void lin_stack::write(const uint8_t ident, const void *data, size_t len) {
    // Synch Break
    lin_break();
    // Send data via Serial interface
    channel.begin(baud);
    channel.write(0x55);
    channel.write(ident);
    channel.write(static_cast<const char *>(data), len);
    channel.write(calcChecksum(ident, data, len));
    channel.flush();
}

void lin_stack::writeRequest(const uint8_t ident) {
    this->ident = ident;
    // Synch Break
    lin_break();
    // Send data via Serial interface
    channel.begin(baud);
    channel.write(0x55);
    channel.write(ident);
    channel.flush();
}

void lin_stack::writeResponse(const void *data, size_t len) {
    channel.begin(baud);
    channel.write(static_cast<const char *>(data), len);
    channel.write(calcChecksum(this->ident, data, len));
    channel.flush();
}

void lin_stack::writeStream(const void *data, size_t len) {
    // Synch Break
    lin_break();
    // Send data via Serial interface
    channel.begin(baud);
    channel.write(0x55);
    channel.write(ident);
    channel.write(static_cast<const char *>(data), len);
    channel.flush();
}

bool lin_stack::read(uint8_t *data, const size_t len, size_t *read) {
    size_t loc;
    if(read == nullptr)
        read = &loc;
    *read = channel.readBytes(data, len);
    return (validateParity(data[0]) && validateChecksum(this->ident, data, min(len, *read)));
}

void lin_stack::setupSerial() { channel.begin(baud); }

bool lin_stack::waitBreak(uint32_t maxTimeout) {
    const auto enterTime = millis();
    while(bit_is_clear(UCSR0A, FE0)) {
        const auto now = millis();
        if(maxTimeout < UINT32_MAX &&  now - enterTime > maxTimeout) {
            // we timed out
            return false;
        }
    }
    return true;
}

int lin_stack::readStream(uint8_t *data, size_t len) { return channel.readBytes(data, len); }

// PRIVATE METHODS
void lin_stack::lin_break() {
    // send the break field. Since LIN only specifies min 13bit, we'll send 0x00 at half baud
    channel.flush();
    channel.begin(baud / 2);

    // send the break field
    channel.write(0x00);
    channel.flush();
}

void lin_stack::sleep(bool sleep_state) {
    digitalWrite(wake_pin, (sleep_state) ? HIGH : LOW);
    delayMicroseconds(20); // According to TJA1021 datasheet this is needed for proper working
}

void lin_stack::sleep_config() {
    pinMode(wake_pin, OUTPUT);
    digitalWrite(wake_pin, LOW);
}

bool lin_stack::validateParity(uint8_t _ident) { return (_ident == ident); }

uint8_t lin_stack::calcChecksum(uint8_t ident, const void *data, size_t len) {
    const uint8_t *p = static_cast<const uint8_t*>(data);
    uint16_t ret = ident;
    for(size_t i=0;i<len;i++){
        ret += p[i];
        if(ret > 255)
            ret -= 255;
    }
    return ~ret;
}

bool lin_stack::validateChecksum(uint8_t ident, const void *data, size_t len) {
    uint8_t crc = calcChecksum(ident, data, len - 1);
    return (crc == static_cast<const uint8_t *>(data)[len]);
}

void lin_stack::busWakeUp() {
    // generate a wakeup pattern by sending 9 zero bits, we use 19200 baud to generate a 480us pulse
    channel.flush();
    channel.begin(19200);
    channel.write(0x00);
    channel.flush();
    channel.begin(baud);
}

uint8_t lin_stack::generateIdent(const uint8_t addr) const {
    return ((addr << 2) & 0x3f) | calcIdentParity(addr);
}

/* Create the Lin ID parity */
#define BIT(data, shift) ((data & (1 << shift)) >> shift)
uint8_t lin_stack::calcIdentParity(const uint8_t ident) const {
    uint8_t p0 = BIT(ident, 0) ^ BIT(ident, 1) ^ BIT(ident, 2) ^ BIT(ident, 4);
    uint8_t p1 = ~(BIT(ident, 1) ^ BIT(ident, 3) ^ BIT(ident, 4) ^ BIT(ident, 5));
    return (p0 | (p1 << 1)) << 6;
}
