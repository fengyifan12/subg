/*
 * Copyright (c) 2022-2025 Rafael Microelectronics Inc. All rights reserved.
 * 
 * SPDX-License-Identifier: LicenseRef-RafaelMicro-Proprietary-1.0
 *
 */

#include "version_info.h"

#define STR_HELPER(x) #x
#define STR(x) STR_HELPER(x)

#ifndef BUILD_CHIP_INFO
#define BUILD_CHIP_INFO    RT584_NO
#endif

#ifndef BUILD_VERSION_INFO
#define BUILD_VERSION_INFO  0x02000000
#endif

#ifndef BUILD_HASH_INFO
#define BUILD_HASH_INFO    0x00000000
#endif

#ifndef BUILD_DATE_INFO
#define BUILD_DATE_INFO    0x20251126
#endif

#ifndef BUILD_MAC_FW_INFO
#define BUILD_MAC_FW_INFO  0x0
#endif

#ifndef BUILD_BLE_FW_INFO
#define BUILD_BLE_FW_INFO    0x0
#endif

#ifndef BUILD_MULTI_FW_INFO
#define BUILD_MULTI_FW_INFO    0x0
#endif


/* 放在 .build_info 區段 */
__attribute__((section(".build_info"), used, aligned(4)))
const build_info_t BUILD_INFO = {
    .ic_name    = STR(BUILD_CHIP_INFO),
    .version    = BUILD_VERSION_INFO,
    .hash       = BUILD_HASH_INFO,
    .build_date = BUILD_DATE_INFO,
#if defined CONFIG_BOOTLOADER_APP
    .start_address = BOOTLOADER_START_ADDRESS,
#else
    .start_address = APP_START_ADDRESS,
#endif
    .length = 0,    
    .mac_fw_ver = BUILD_MAC_FW_INFO,
    .ble_fw_ver = BUILD_BLE_FW_INFO,
    .multi_fw_ver = BUILD_MULTI_FW_INFO,
    .app_start_address = APP_START_ADDRESS,
    .reserved1[0] = 0x00,
    .reserved1[1] = 0x00,
    .reserved1[2] = 0x00,
    .reserved1[3] = 0x00,
    .sha256 = 0,
    .reserved2   = { [0 ... (sizeof(((build_info_t*)0)->reserved2) - 1)] = 0 }
};
_Static_assert(sizeof(build_info_t) == 128, "build_info_t must be 128 bytes");

