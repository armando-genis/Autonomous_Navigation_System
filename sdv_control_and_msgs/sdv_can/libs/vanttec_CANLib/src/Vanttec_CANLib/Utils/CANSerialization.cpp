//
// Created by abiel on 12/9/21.
//

#include "CANSerialization.h"
#include "ByteOrder/inet.h"
#include <math.h>

uint32_t serialize_float(float input) {
    uint32_t out = 0;
    unsigned int* ptr = (unsigned int*)&input; // Type cast float pointer to unsigned int pointer
    unsigned int value = *ptr; // Get the binary representation of the float

    unsigned int sign = (value >> 31) & 0x1; // Extract the sign bit (MSB)
    unsigned int exponent = (value >> 23) & 0xFF; // Extract the exponent bits (bits 30-23)
    unsigned int mantissa = value & 0x7FFFFF; // Extract the mantissa bits (bits 22-0)
    out |= sign << 31;
    out |= exponent << 23;
    out |= mantissa & 0x7fffff;
    return out;
}

void serialize_short(uint8_t *data, uint16_t in) {
    in = vanttec_htons(in);
    data[0] = in >> 8;
    data[1] = in & 0xFF;
}

void serialize_long(uint8_t *data, uint32_t in) {
    in = vanttec_htonl(in);
    data[0] = (in >> (8 * 3)) & 0xFF;
    data[1] = (in >> (8 * 2)) & 0xFF;
    data[2] = (in >> 8) & 0xFF;
    data[3] = in & 0xFF;
}
