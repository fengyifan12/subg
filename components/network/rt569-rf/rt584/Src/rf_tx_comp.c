/*
 * Copyright (c) 2022-2025 Rafael Microelectronics Inc. All rights reserved.
 * 
 * SPDX-License-Identifier: LicenseRef-RafaelMicro-Proprietary-1.0
 *
 */

/** @file Compensate.c
 *
 * @brief Compensate driver file.
 *
 */

/**
 * @defgroup SADC Group
 * @{
 * @brief    SAR ADC group.
 */

/**************************************************************************************************
 *    INCLUDES
 *************************************************************************************************/
#include <stdio.h>
#include <string.h>

#include "mcu.h"
#include "rf_common_init.h"
#include "rf_mcu.h"
#include "rf_tx_comp.h"
#include "lpm.h"

#include "FreeRTOS.h"
#include "timers.h"

#if (RF_TX_POWER_COMP)
#include "mp_sector.h"
#endif

/**************************************************************************************************
 *    MACROS
 *************************************************************************************************/


/**************************************************************************************************
 *    CONSTANTS AND DEFINES
 *************************************************************************************************/
#if (RF_MCU_CHIP_MODEL == RF_MCU_CHIP_569S)
#define TX_PWR_COMP_TEMPERATURE_HIGH      2
#define TX_PWR_COMP_TEMPERATURE_NORMAL    1
#define TX_PWR_COMP_TEMPERATURE_LOW       0
#define TX_PWR_COMP_TEMPERATURE_DEFAULT   3

#define TX_PWR_COMP_TEMPERATURE_ELEMENT_PLL   3
#define TX_PWR_COMP_TEMPERATURE_BOUNDARY_PLL  (TX_PWR_COMP_TEMPERATURE_ELEMENT_PLL - 1)
#if (defined(CONFIG_RT584H) || defined(CONFIG_RT584HA4))
#define TX_PWR_COMP_TEMPERATURE_ELEMENT   2
#define TX_PWR_COMP_VBAT_ELEMENT          2
#define TX_PWR_COMP_TEMPERATURE_BOUNDARY  (TX_PWR_COMP_TEMPERATURE_ELEMENT - 1)
#define TX_PWR_COMP_VBAT_BOUNDARY         (TX_PWR_COMP_VBAT_ELEMENT - 1)
#elif (defined(CONFIG_RF1301))
#define TX_PWR_COMP_TEMPERATURE_ELEMENT   3
#define TX_PWR_COMP_VBAT_ELEMENT          3
#define TX_PWR_COMP_TEMPERATURE_BOUNDARY  (TX_PWR_COMP_TEMPERATURE_ELEMENT - 1)
#define TX_PWR_COMP_VBAT_BOUNDARY         (TX_PWR_COMP_VBAT_ELEMENT - 1)
#endif

#define TX_PWR_COMP_DEBUG                 1
#else
#define TX_PWR_COMP_TEMPERATURE_ELEMENT   3
#define TX_PWR_COMP_VBAT_ELEMENT          3
#define TX_PWR_COMP_TEMPERATURE_BOUNDARY  (TX_PWR_COMP_TEMPERATURE_ELEMENT - 1)
#define TX_PWR_COMP_VBAT_BOUNDARY         (TX_PWR_COMP_VBAT_ELEMENT - 1)

#define TX_PWR_COMP_DEBUG                 0
#endif

/**************************************************************************************************
 *    TYPEDEFS
 *************************************************************************************************/


/**************************************************************************************************
 *    GLOBAL VARIABLES
 *************************************************************************************************/
#if defined(CONFIG_FREERTOS)
TimerHandle_t rf_tx_comp_timer_handle = NULL;
#endif
tx_pwr_comp_state_t comp_state = TX_PWR_COMP_TEMPERATURE;
sadc_value_t tx_pwr_comp_value_temperature = 0;
sadc_value_t tx_pwr_comp_value_vbat = 0;

#if (RF_MCU_CHIP_MODEL == RF_MCU_CHIP_569S)
uint8_t tx_pwr_comp_region_pre  = TX_PWR_COMP_TEMPERATURE_DEFAULT;
uint8_t tx_pwr_comp_region      = TX_PWR_COMP_TEMPERATURE_NORMAL;
uint8_t tx_pwr_comp_pre         = TX_PWR_COMP_TEMPERATURE_NORMAL;
#if (defined(CONFIG_RF1301))
uint8_t tx_pwr_comp_pre_915MHz  = TX_PWR_COMP_TEMPERATURE_NORMAL;
#endif

#if (defined(CONFIG_RT584H) || defined(CONFIG_RT584HA4))
sadc_value_t tx_pwr_comp_boundary_temperature[TX_PWR_COMP_TEMPERATURE_BOUNDARY] = {-20};
sadc_value_t tx_pwr_comp_boundary_vbat[TX_PWR_COMP_VBAT_BOUNDARY] = {3450};
#elif (defined(CONFIG_RF1301))
sadc_value_t tx_pwr_comp_boundary_temperature[TX_PWR_COMP_TEMPERATURE_BOUNDARY] = {-20, -20};
sadc_value_t tx_pwr_comp_boundary_vbat[TX_PWR_COMP_VBAT_BOUNDARY] = {3450, 3450};
#endif
#if (defined(CONFIG_RT584HA4) || defined(CONFIG_RF1301))
sadc_value_t tx_pwr_comp_boundary_temperature_pll_H[TX_PWR_COMP_TEMPERATURE_BOUNDARY_PLL] = {0, 45};
sadc_value_t tx_pwr_comp_boundary_temperature_pll_N[TX_PWR_COMP_TEMPERATURE_BOUNDARY_PLL] = {-10, 55};
sadc_value_t tx_pwr_comp_boundary_temperature_pll_L[TX_PWR_COMP_TEMPERATURE_BOUNDARY_PLL] = {0, 55};
#elif (defined(CONFIG_RT584H) || defined(CONFIG_RT584L))
sadc_value_t tx_pwr_comp_boundary_temperature_pll_H[TX_PWR_COMP_TEMPERATURE_BOUNDARY_PLL] = {0, 70};
sadc_value_t tx_pwr_comp_boundary_temperature_pll_N[TX_PWR_COMP_TEMPERATURE_BOUNDARY_PLL] = {-10, 80};
sadc_value_t tx_pwr_comp_boundary_temperature_pll_L[TX_PWR_COMP_TEMPERATURE_BOUNDARY_PLL] = {0, 80};
#endif

#if (defined(CONFIG_RT584H) || defined(CONFIG_RT584HA4))
tx_pwr_comp_element_t tx_pwr_comp_table_h_20dbm[TX_PWR_COMP_TEMPERATURE_ELEMENT][TX_PWR_COMP_VBAT_ELEMENT] =
{
    {{0}, {1}},
    {{2}, {2}}
};
#elif (defined(CONFIG_RF1301))
tx_pwr_comp_element_t tx_pwr_comp_table_s_20dbm[TX_PWR_COMP_TEMPERATURE_ELEMENT][TX_PWR_COMP_VBAT_ELEMENT] =
{
    {{1, 3}, {1, 4}, {1, 4}},
    {{0, 0}, {1, 2}, {1, 2}},
    {{0, 0}, {1, 2}, {1, 2}}
};
#endif

#if (defined(CONFIG_RT584H) || defined(CONFIG_RT584HA4))
tx_pwr_comp_element_t (*ptr_tx_pwr_comp_table)[TX_PWR_COMP_VBAT_ELEMENT] = tx_pwr_comp_table_h_20dbm;
#elif (defined(CONFIG_RF1301))
tx_pwr_comp_element_t (*ptr_tx_pwr_comp_table)[TX_PWR_COMP_VBAT_ELEMENT] = tx_pwr_comp_table_s_20dbm;
#endif
#else
sadc_value_t tx_pwr_comp_boundary_temperature[TX_PWR_COMP_TEMPERATURE_BOUNDARY] = {-10, 60};
sadc_value_t tx_pwr_comp_boundary_vbat[TX_PWR_COMP_VBAT_BOUNDARY] = {3500, 2900};

tx_pwr_comp_element_t tx_pwr_comp_table_10dbm[TX_PWR_COMP_VBAT_ELEMENT][TX_PWR_COMP_TEMPERATURE_ELEMENT] =
{
    {{-4, 63, 0}, {-1, 63, 0}, {0, 63, 0}},
    {{-2, 63, 0}, { 0, 63, 0}, {1, 63, 0}},
    {{ 2,  0, 3}, { 2,  0, 3}, {2,  0, 3}}
};

tx_pwr_comp_element_t tx_pwr_comp_table_4dbm[TX_PWR_COMP_VBAT_ELEMENT][TX_PWR_COMP_TEMPERATURE_ELEMENT] =
{
    {{-6, 63, 0}, {-3, 63, 0}, {0, 63, 0}},
    {{-5, 63, 0}, { 0, 63, 0}, {2, 63, 0}},
    {{-1, 63, 0}, { 2, 63, 1}, {2, 63, 2}}
};

tx_pwr_comp_element_t tx_pwr_comp_table_0dbm[TX_PWR_COMP_VBAT_ELEMENT][TX_PWR_COMP_TEMPERATURE_ELEMENT] =
{
    {{-4, 63, 0}, {-2, 63, 0}, {0, 63, 0}},
    {{-3, 63, 0}, { 0, 63, 0}, {2, 63, 0}},
    {{ 0, 63, 0}, { 3, 63, 0}, {6, 63, 0}}
};

tx_pwr_comp_element_t (*ptr_tx_pwr_comp_table)[TX_PWR_COMP_TEMPERATURE_ELEMENT] = tx_pwr_comp_table_10dbm;
#endif

/**************************************************************************************************
 *    LOCAL FUNCTIONS
 *************************************************************************************************/


/**************************************************************************************************
 *    GLOBAL FUNCTIONS
 *************************************************************************************************/
void Tx_Power_Sadc_Int_Callback_Handler(sadc_cb_t *p_cb)
{

}

#include <inttypes.h>


typedef struct {
    uint32_t pw_vco_auto            : 4;
    uint32_t rf_pll_vtbit           : 2;
    uint32_t rf_vco_amp_ok          : 1;
    uint32_t rf_pll_lock            : 1;
    uint32_t rf_vco_bank            : 6;
    uint32_t reserved               : 2;
    uint32_t rf_read_rvd            : 16;
} rf_pll_reads_t, *rf_pll_reads_ptr_t;

typedef struct
{
    uint32_t chmap_pll_sdm      : 16;   /* [15:0]  SDM fractional value        */
    uint32_t chmap_pll_n        : 6;    /* [21:16] PLL integer divider N        */
    uint32_t chmap_pll_s        : 2;    /* [23:22] PLL S divider                */
    uint32_t vcob_fast_sel_vco  : 6;    /* [29:24] VCO fast selection           */
    uint32_t vcob_fast_error_flag : 1;  /* [30]    VCO fast lock error flag     */
    uint32_t reserved           : 1;    /* [31]    reserved                     */
} pll_auto_reads_t, *pll_auto_reads_ptr_t;



void rf_common_radio_reg_rf_pll_dump (void)
{
    uint32_t reg_val, reg_val2;
    uint16_t reg_addr = 0x328, reg_addr2 = 0x3D4;


    reg_val = RfMcu_RegGet(reg_addr);
    printf("rf_vt_bit: %d, rf_vco_bank:%d\r\n", 
        ((rf_pll_reads_ptr_t)&reg_val)->rf_pll_vtbit, 
        ((rf_pll_reads_ptr_t)&reg_val)->rf_vco_bank);
    printf("Addr: 0x%04"PRIx16", Val: 0x%08"PRIx32" \r\n", reg_addr, reg_val);
    reg_val2 = RfMcu_RegGet(reg_addr2);
    printf("RF vcob_fast_sel_vco: %d, vcob_fast_error_flag:%d\r\n", 
        ((pll_auto_reads_ptr_t)&reg_val2)->vcob_fast_sel_vco, 
        ((pll_auto_reads_ptr_t)&reg_val2)->vcob_fast_error_flag);
    printf("Addr: 0x%04"PRIx16", Val: 0x%08"PRIx32" \r\n", reg_addr2, reg_val2);
}

#if (RF_TX_POWER_COMP)
void Tx_Power_Compensation_Update(sadc_value_t temperature, sadc_value_t vbat)
{
    uint32_t tx_pwr_comp_temperature_index = 0;
    uint32_t tx_pwr_comp_vbat_index = 0;
    tx_pwr_comp_element_t tx_pwr_comp;
    #if 0
    uint32_t vt_bit = 0;

    vt_bit = PLL_VIBIT_STATUS();
    printf("PMU_CTRL->soc_bbpll0: 0x%08x"PRIx32"\r\n", PMU_CTRL->soc_bbpll0);
    printf("PMU_CTRL->soc_bbpll1: 0x%08x"PRIx32"\r\n", PMU_CTRL->soc_bbpll1);
    rf_common_radio_reg_rf_pll_dump();
    printf("vt_bit: %d, vco bank: %d\r\n", vt_bit, PLL_BANK_VCO_STATUS());
    if(vt_bit == 3) {
        printf("!!!!!!!!!!!!");
        printf("!!!!!!!!!!!!");
        printf("!vt_bit = 3!");
        printf("!!!!!!!!!!!!");
        printf("!!!!!!!!!!!!");
    }
    #endif


#if (RF_MCU_CHIP_MODEL == RF_MCU_CHIP_569S)

    for (tx_pwr_comp_temperature_index = 0; tx_pwr_comp_temperature_index < TX_PWR_COMP_TEMPERATURE_BOUNDARY_PLL; tx_pwr_comp_temperature_index++)
    {
        if (tx_pwr_comp_region_pre == TX_PWR_COMP_TEMPERATURE_HIGH)
        {
            if (temperature < tx_pwr_comp_boundary_temperature_pll_H[tx_pwr_comp_temperature_index])
            {
                break;
            }
        }
        else if (tx_pwr_comp_region_pre == TX_PWR_COMP_TEMPERATURE_NORMAL)
        {
            if (temperature < tx_pwr_comp_boundary_temperature_pll_N[tx_pwr_comp_temperature_index])
            {
                break;
            }
        }
        else
        {
            if (temperature < tx_pwr_comp_boundary_temperature_pll_L[tx_pwr_comp_temperature_index])
            {
                break;
            }
        }
    }
#if (TX_PWR_COMP_DEBUG == 1)
    printf("Region Ref : %d", tx_pwr_comp_region_pre);
#endif
    tx_pwr_comp_region = tx_pwr_comp_temperature_index;

#if ((CONFIG_RF_POWER_20DBM == 1) && (defined(CONFIG_RT584H) ||  defined(CONFIG_RT584HA4) || defined(CONFIG_RF1301)))
#if (defined(CONFIG_RT584H) || defined(CONFIG_RT584HA4))
    for (tx_pwr_comp_temperature_index = 0; tx_pwr_comp_temperature_index < TX_PWR_COMP_TEMPERATURE_BOUNDARY; tx_pwr_comp_temperature_index++)
    {
        if (temperature >= tx_pwr_comp_boundary_temperature[tx_pwr_comp_temperature_index])
        {
            break;
        }
    }

    for (tx_pwr_comp_vbat_index = 0; tx_pwr_comp_vbat_index < TX_PWR_COMP_VBAT_BOUNDARY; tx_pwr_comp_vbat_index++)
    {
        if (vbat <= tx_pwr_comp_boundary_vbat[tx_pwr_comp_vbat_index])
        {
            break;
        }
    }

    tx_pwr_comp = ptr_tx_pwr_comp_table[tx_pwr_comp_temperature_index][tx_pwr_comp_vbat_index];
#if (TX_PWR_COMP_DEBUG == 1)
    printf("txp_offset: %d\n", tx_pwr_comp.txp_offset);
#endif
    if ((tx_pwr_comp_region != tx_pwr_comp_region_pre) || (tx_pwr_comp.txp_offset != tx_pwr_comp_pre))
    {
        tx_pwr_comp_region_pre = tx_pwr_comp_region;
        tx_pwr_comp_pre = tx_pwr_comp.txp_offset;
        rf_common_tx_pwr_comp_set(0, tx_pwr_comp_region, tx_pwr_comp.txp_offset, 0);
#if (TX_PWR_COMP_DEBUG == 1)
        printf("Change Region to %d %d", tx_pwr_comp_region, tx_pwr_comp.txp_offset);
#endif
    }
#elif (defined(CONFIG_RF1301))
    for (tx_pwr_comp_temperature_index = 0; tx_pwr_comp_temperature_index < TX_PWR_COMP_TEMPERATURE_BOUNDARY; tx_pwr_comp_temperature_index++)
    {
        if (temperature <= tx_pwr_comp_boundary_temperature[tx_pwr_comp_temperature_index])
        {
            break;
        }
    }

    for (tx_pwr_comp_vbat_index = 0; tx_pwr_comp_vbat_index < TX_PWR_COMP_VBAT_BOUNDARY; tx_pwr_comp_vbat_index++)
    {
        if (vbat <= tx_pwr_comp_boundary_vbat[tx_pwr_comp_vbat_index])
        {
            break;
        }
    }

    tx_pwr_comp = ptr_tx_pwr_comp_table[tx_pwr_comp_temperature_index][tx_pwr_comp_vbat_index];
#if (TX_PWR_COMP_DEBUG == 1)
    printf("txp_offset: %d , txp_offset_915MHz: %d\n", tx_pwr_comp.txp_offset, tx_pwr_comp.txp_offset_915MHz);
#endif
    if ((tx_pwr_comp_region != tx_pwr_comp_region_pre) || (tx_pwr_comp.txp_offset != tx_pwr_comp_pre) || (tx_pwr_comp.txp_offset_915MHz != tx_pwr_comp_pre_915MHz))
    {
        tx_pwr_comp_region_pre = tx_pwr_comp_region;
        tx_pwr_comp_pre = tx_pwr_comp.txp_offset;
        tx_pwr_comp_pre_915MHz = tx_pwr_comp.txp_offset_915MHz;
        rf_common_tx_pwr_comp_set(0, tx_pwr_comp_region, (uint8_t) ((tx_pwr_comp.txp_offset_915MHz << 4) | tx_pwr_comp.txp_offset), 0);
#if (TX_PWR_COMP_DEBUG == 1)
        printf("Change Region to %d %d %d", tx_pwr_comp_region, tx_pwr_comp.txp_offset, tx_pwr_comp.txp_offset_915MHz);
#endif
    }
#endif
#else
    if (tx_pwr_comp_region != tx_pwr_comp_region_pre)
    {
        tx_pwr_comp_region_pre = tx_pwr_comp_region;
        rf_common_tx_pwr_comp_set(0, tx_pwr_comp_region, 0, 0);
#if (TX_PWR_COMP_DEBUG == 1)
        printf("Change Region to %d", tx_pwr_comp_region);
#endif
    }
#endif
#else
    uint32_t tx_pwr_comp_temperature_index = 0;
    uint32_t tx_pwr_comp_vbat_index = 0;
    tx_pwr_comp_element_t tx_pwr_comp;

    for (tx_pwr_comp_temperature_index = 0; tx_pwr_comp_temperature_index < TX_PWR_COMP_TEMPERATURE_BOUNDARY; tx_pwr_comp_temperature_index++)
    {
        if (temperature < tx_pwr_comp_boundary_temperature[tx_pwr_comp_temperature_index])
        {
            break;
        }
    }

    for (tx_pwr_comp_vbat_index = 0; tx_pwr_comp_vbat_index < TX_PWR_COMP_VBAT_BOUNDARY; tx_pwr_comp_vbat_index++)
    {
        if (vbat > tx_pwr_comp_boundary_vbat[tx_pwr_comp_vbat_index])
        {
            break;
        }
    }

    tx_pwr_comp = ptr_tx_pwr_comp_table[tx_pwr_comp_vbat_index][tx_pwr_comp_temperature_index];
    rf_common_tx_pwr_comp_set(tx_pwr_comp.offset, tx_pwr_comp.poly_gain, tx_pwr_comp.pa_pw_pre, 0);

#if 0//(TX_PWR_COMP_DEBUG == 1)
    SYSCTRL->SYS_SCRATCH[4] = temperature;
    SYSCTRL->SYS_SCRATCH[5] = vbat;
#endif
#endif
}

void Tx_Power_Compensation_Sadc_Int_Handler(sadc_cb_t *p_cb)
{
    uint32_t             sadc_comp_input;
    sadc_value_t         sadc_comp_value;

    if (p_cb->type == SADC_CB_SAMPLE)
    {
        sadc_comp_input = p_cb->data.sample.channel;
        sadc_comp_value = p_cb->data.sample.value;

        // #if (TX_PWR_COMP_DEBUG == 1)
        //         printf("\nADC CH%d: adc = %d, comp = %d, cal = %d\n", p_cb->data.sample.channel, p_cb->raw.conversion_value, p_cb->raw.compensation_value, p_cb->raw.calibration_value);
        // #endif

        if (sadc_comp_input == SADC_CH_TEMPERATURE)
        {
            // #if (TX_PWR_COMP_DEBUG == 1)
            //             gpio_pin_toggle(1);
            //             printf("\nTemperature ADC = %d\n", sadc_comp_value);
            // #endif

            tx_pwr_comp_value_temperature = sadc_comp_value;
#if (TX_PWR_COMP_DEBUG == 1)
            printf("Temp:%d\r\n", tx_pwr_comp_value_temperature);
#endif
        }
        else if (sadc_comp_input == SADC_CH_VBAT)
        {
            // #if (TX_PWR_COMP_DEBUG == 1)
            //             gpio_pin_toggle(2);
            //             printf("\nVbat ADC = %d\n", sadc_comp_value);
            // #endif

            tx_pwr_comp_value_vbat = sadc_comp_value;
#if (TX_PWR_COMP_DEBUG == 1)
            printf("Vbat:%d\r\n", tx_pwr_comp_value_vbat);
#endif
        }

        sadc_disable();
        lpm_low_power_unmask(LOW_POWER_MASK_BIT_TASK_ADC);

        // if ((tx_pwr_comp_value_temperature != 0) && (tx_pwr_comp_value_vbat != 0))
        if (tx_pwr_comp_value_temperature != 0)
        {
            Tx_Power_Compensation_Update(tx_pwr_comp_value_temperature, tx_pwr_comp_value_vbat);
        }
    }
}

#if defined(CONFIG_FREERTOS)
void Tx_Power_Compensation_Periodic_Callback(TimerHandle_t pxTimer)
{
    /* Optionally do something if the pxTimer parameter is NULL. */
    configASSERT(pxTimer);

    switch (comp_state)
    {
    case TX_PWR_COMP_VBAT:

        if (sadc_vbat_read() == STATUS_SUCCESS)
        {
            lpm_low_power_mask(LOW_POWER_MASK_BIT_TASK_ADC);
            comp_state = TX_PWR_COMP_TEMPERATURE;
        }
        break;

    case TX_PWR_COMP_TEMPERATURE:

        if (sadc_temp_read() == STATUS_SUCCESS)
        {
            lpm_low_power_mask(LOW_POWER_MASK_BIT_TASK_ADC);
            comp_state = TX_PWR_COMP_VBAT;
        }
        break;

    default:
        break;
    }
}
#endif

void Tx_Power_Compensation_Init(uint32_t xPeriodicTimeInSec)
{
#if !(RF_MCU_CHIP_MODEL == RF_MCU_CHIP_569S)
    mp_tx_power_trim_t                  sTx_power_trim;

    /* Init TX power base from MP sector */
    ptr_tx_pwr_comp_table = tx_pwr_comp_table_10dbm;
    if (mpcalrftrimread(MP_ID_TX_POWER_TRIM, MP_CNT_TX_POWER_TRIM, (uint8_t *)(&sTx_power_trim)) == STATUS_SUCCESS)
    {
        if (((sTx_power_trim.flag == 1) || (sTx_power_trim.flag == 2)) && (sTx_power_trim.mode & RF_BAND_SUPP(RF_BAND_2P4G)))
        {
            switch (sTx_power_trim.tx_gain_idx_2g >> 6)
            {
            case (0):
                ptr_tx_pwr_comp_table = tx_pwr_comp_table_10dbm;
#if (TX_PWR_COMP_DEBUG == 1)
                printf("\nUsing 10dBm power compensation table\n");
#endif
                break;
            case (1):
                ptr_tx_pwr_comp_table = tx_pwr_comp_table_4dbm;
#if (TX_PWR_COMP_DEBUG == 1)
                printf("\nUsing 4dBm power compensation table\n");
#endif
                break;
            case (2):
                ptr_tx_pwr_comp_table = tx_pwr_comp_table_0dbm;
#if (TX_PWR_COMP_DEBUG == 1)
                printf("\nUsing 0dBm power compensation table\n");
#endif
                break;
            }
        }
    }
#endif
    // Sadc_Register_Int_Callback(Tx_Power_Compensation_Sadc_Int_Handler);

#if defined(CONFIG_FREERTOS)
    mp_cal_temp_adc_t mp_cal_temp_adc;

    if (mpcalrftrimread(MP_ID_TEMP_ADC, MP_CNT_TEMPADC, (uint8_t *)(&mp_cal_temp_adc)) == STATUS_SUCCESS)
    {
        if ((mp_cal_temp_adc.flag == 1) || (mp_cal_temp_adc.flag == 2))
        {
            rf_tx_comp_timer_handle = xTimerCreate
                                      ( /* Just a text name, not used by the RTOS kernel. */
                                          "Timer",
                                          /* The timer period in ticks, must be greater than 0. */
                                          pdMS_TO_TICKS(xPeriodicTimeInSec * 1000),
                                          /* The timers will auto-reload themselves when they expire. */
                                          pdTRUE,
                                          /* The ID is used to store a count of the number of times the timer has expired, which is initialised to 0. */
                                          (void *) 0,
                                          /* Each timer calls the same callback when it expires. */
                                          Tx_Power_Compensation_Periodic_Callback
                                      );

            if (rf_tx_comp_timer_handle == NULL)
            {
                /* The timer was not created. */
            }
            else
            {
                /* Start the timer.  No block time is specified, and
                even if one was it would be ignored because the RTOS
                scheduler has not yet been started. */
                if (xTimerStart(rf_tx_comp_timer_handle, 0) != pdPASS)
                {
                    /* The timer could not be set into the Active state. */
                }
            }
        }
    }
#else
    //??
    /* Add periodic callback function for tx power compensation */
#endif
}

void Tx_Power_Compensation_Deinit(void)
{
#if defined(CONFIG_FREERTOS)
    if (rf_tx_comp_timer_handle != NULL)
    {
        if (xTimerDelete(rf_tx_comp_timer_handle, 0) == pdPASS)
        {
            /* The delete command was successfully sent to the timer command queue. */
            rf_tx_comp_timer_handle = NULL;
        }
    }
#endif

    sadc_register_int_callback(NULL);
}
#endif

/** @} */
