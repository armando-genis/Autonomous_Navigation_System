//
// Created by abiel on 12/9/21.
//

#include "inet.h"

#ifdef __cplusplus
extern "C" {
#endif
uint32_t vanttec_htonl(uint32_t hostlong) {
    return __builtin_bswap32(hostlong);
}

uint16_t vanttec_htons(uint16_t hostshort) {
    return __builtin_bswap16(hostshort);
}

uint32_t vanttec_ntohl(uint32_t netlong) {
    return __builtin_bswap32(netlong);
}

uint16_t vanttec_ntohs(uint16_t netshort) {
    return __builtin_bswap16(netshort);
}
#ifdef __cplusplus
}
#endif
