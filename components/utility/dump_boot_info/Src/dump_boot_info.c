/*
 * Copyright (c) 2022-2025 Rafael Microelectronics Inc. All rights reserved.
 * 
 * SPDX-License-Identifier: LicenseRef-RafaelMicro-Proprietary-1.0
 *
 */

#include <FreeRTOS.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "sysctrl.h"
#include "sysfun.h"
#include "uart_stdio.h"


extern void systemcoreclockupdate(void);
extern void get_set_sys_clk_value(uint32_t* get_value);

void _dump_boot_info(void) {
    uint32_t ver_major, ver_minor, tar_sys_clk, check_pll_times = 0;
    sys_clk_sel_t now_sys_clk;

    puts("\r\n");
    puts("------------------------------------------------------------\r\n");
    printf("ARM %s",
           getotpversion().type == CHIP_TYPE_584 ? "Cortex-M33" : "Cortex-M3");
    puts(" SoC: ");
    puts(CONFIG_CHIP);
    puts("\r\n");
    puts("Target Board: ");
    puts(CONFIG_BOARD);
    puts("\r\n");

#ifdef HOSAL_SOC_TARGET_CUSTOMER
    puts(" Customer: ");
    puts(HOSAL_SOC_TARGET_CUSTOMER);
    puts("\r\n");
#endif

    puts("SDK Version: ");
    puts(RAFAEL_SDK_VER);
    puts("\r\n");
    puts("Build Project: ");
    puts(CONFIG_BUILD_PORJECT);
    puts("\r\n");
    puts("Build Date: ");
    puts(__DATE__);
    puts("\r\n");
    puts("Build Time: ");
    puts(__TIME__);
    puts("\r\n");
    puts("Build Commit SHA: ");
    printf("%.8x", BUILD_HASH_INFO);
    puts("\r\n");
    puts("MAC FW Version: ");
    printf("%.8x", BUILD_MAC_FW_INFO);
    puts("\r\n");
    puts("BLE FW Version: ");
    printf("%.8x", BUILD_BLE_FW_INFO);
    puts("\r\n");
    puts("MULTI FW Version: ");
    printf("%.8x", BUILD_MULTI_FW_INFO);
    puts("\r\n");
    uint32_t bootloader_version = *(uint32_t*)0x100000B0;

    ver_major = (bootloader_version >> 16) & 0xFFFF;
    ver_minor = bootloader_version & 0xFFFF;

    printf("Bootloader Version: %.4d.%.4d\r\n", ver_major, ver_minor);

    printf("System clock: %s\r\n",
           (get_ahb_system_clk() == SYS_CLK_32MHZ)   ? "32MHz"
           : (get_ahb_system_clk() == SYS_CLK_48MHZ) ? "48MHz"
                                                     : "64MHz");

#if defined(CONFIG_RT584H) || defined(CONFIG_RT584L) || defined(CONFIG_RF1301) || defined(CONFIG_RT584HA4)
    printf("PMU_CTRL->soc_bbpll_read: 0x%.8x\r\n", PMU_CTRL->soc_bbpll_read.reg);
    printf("bbpll_vt_bit: %d, bbpll_vco_bank:%d\r\n", PMU_CTRL->soc_bbpll_read.bit.bbpll_vtbit, PMU_CTRL->soc_bbpll_read.bit.bbpll_bank_vco);
    printf("PMU_CTRL->soc_bbpll0: 0x%.8x\r\n", PMU_CTRL->soc_bbpll0.reg);
    printf("PMU_CTRL->soc_bbpll1: 0x%.8x\r\n", PMU_CTRL->soc_bbpll1.reg);
#endif
    systemcoreclockupdate();
    check_pll_times = 1;
    do{
        now_sys_clk = get_ahb_system_clk();
        get_set_sys_clk_value(&tar_sys_clk);
        if (tar_sys_clk != now_sys_clk) {
            change_ahb_system_clk(tar_sys_clk);
            puts(
            "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\r\n");
            printf("retry lock pll times :%d\r\n",check_pll_times);
#if defined(CONFIG_RT584H) || defined(CONFIG_RT584L) || defined(CONFIG_RF1301) || defined(CONFIG_RT584HA4)
            printf("PMU_CTRL->soc_bbpll_read: 0x%.8x\r\n", PMU_CTRL->soc_bbpll_read.reg);
            printf("bbpll_vt_bit: %d, bbpll_vco_bank:%d\r\n", PMU_CTRL->soc_bbpll_read.bit.bbpll_vtbit, PMU_CTRL->soc_bbpll_read.bit.bbpll_bank_vco);
            printf("PMU_CTRL->soc_bbpll0: 0x%.8x\r\n", PMU_CTRL->soc_bbpll0.reg);
            printf("PMU_CTRL->soc_bbpll1: 0x%.8x\r\n", PMU_CTRL->soc_bbpll1.reg);
#endif
            puts(
            "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\r\n\r\n");
            
        }
        check_pll_times ++;
    }while(check_pll_times < 4);

    

    now_sys_clk = get_ahb_system_clk();
    get_set_sys_clk_value(&tar_sys_clk);
    if (tar_sys_clk != now_sys_clk) {
        puts(
            "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\r\n");
        puts(
            "!!!!!!!!!!!!!!!!!!!!!!!!!!WARNING!!!!!!!!!!!!!!!!!!!!!!!!!!!\r\n");
        printf("target system clock %s, now system clock %s\r\n",
            (tar_sys_clk == SYS_CLK_32MHZ)   ? "32MHz"
            : (tar_sys_clk == SYS_CLK_48MHZ) ? "48MHz"
                                                : "64MHz",
            (now_sys_clk == SYS_CLK_32MHZ)   ? "32MHz"
            : (now_sys_clk == SYS_CLK_48MHZ) ? "48MHz"
                                                : "64MHz");
        puts(
            "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\r\n");
    }
    puts("------------------------------------------------------------\r\n");
}
