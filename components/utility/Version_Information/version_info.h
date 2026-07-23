/*
 * Copyright (c) 2022-2025 Rafael Microelectronics Inc. All rights reserved.
 * 
 * SPDX-License-Identifier: LicenseRef-RafaelMicro-Proprietary-1.0
 *
 */

#ifndef __VERSION_INFO_H__
#define __VERSION_INFO_H__


#include "fota_define.h"

typedef struct __attribute__((packed, aligned(4))) {
    char ic_name[8];                // 8B
    uint32_t version;               // 4B
    uint32_t hash;                  // 4B   //16
    uint32_t build_date;            // 4B
    uint32_t start_address;         // 4B
    uint32_t length;                // 4B
    uint32_t mac_fw_ver;            // 4B   //32
    uint32_t ble_fw_ver;            // 4B
    uint32_t multi_fw_ver;          // 4B
    uint32_t app_start_address;     // 4B   //44B
    uint8_t reserved1[4];           // 4B   //48B
    uint8_t sha256[32];             // 32B  //76B
    uint8_t reserved2[48];          // 128 - (76) = 52 
} build_info_t;

#endif
