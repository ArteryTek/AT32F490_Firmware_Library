/**
  **************************************************************************
  * @file     system_at32f490.c
  * @brief    contains all the functions for cmsis cortex-m4 system source file
  **************************************************************************
  *                       Copyright notice & Disclaimer
  *
  * The software Board Support Package (BSP) that is made available to 
  * download from Artery official website is the copyrighted work of Artery. 
  * Artery authorizes customers to use, copy, and distribute the BSP 
  * software and its related documentation for the purpose of design and 
  * development in conjunction with Artery microcontrollers. Use of the 
  * software is governed by this copyright notice and the following disclaimer.
  *
  * THIS SOFTWARE IS PROVIDED ON "AS IS" BASIS WITHOUT WARRANTIES,
  * GUARANTEES OR REPRESENTATIONS OF ANY KIND. ARTERY EXPRESSLY DISCLAIMS,
  * TO THE FULLEST EXTENT PERMITTED BY LAW, ALL EXPRESS, IMPLIED OR
  * STATUTORY OR OTHER WARRANTIES, GUARANTEES OR REPRESENTATIONS,
  * INCLUDING BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY,
  * FITNESS FOR A PARTICULAR PURPOSE, OR NON-INFRINGEMENT.
  *
  **************************************************************************
  */

/** @addtogroup CMSIS
  * @{
  */

/** @addtogroup AT32F490_system
  * @{
  */
    
#include "at32f490.h"

/** @addtogroup AT32F490_system_private_defines
  * @{
  */
#define VECT_TAB_OFFSET                  0x0 /*!< vector table base offset field. this value must be a multiple of 0x200. */
/**
  * @}
  */

/** @addtogroup AT32F490_system_private_variables
  * @{
  */
unsigned int system_core_clock           = HICK_VALUE; /*!< system clock frequency (core clock) */
/**
  * @}
  */

/** @addtogroup AT32F490_system_private_functions
  * @{
  */

/**
  * @brief  setup the microcontroller system
  *         initialize the flash interface.
  * @note   this function should be used only after reset.
  * @param  none
  * @retval none
  */
void SystemInit (void)
{
#if defined (__FPU_USED) && (__FPU_USED == 1U)
  SCB->CPACR |= ((3U << 10U * 2U) |         /* set cp10 full access */
                 (3U << 11U * 2U)  );       /* set cp11 full access */
#endif

  /* reset the crm clock configuration to the default reset state(for debug purpose) */
  /* enable auto step mode */
  crm_auto_step_mode_enable(TRUE);

  /* set hicken bit */
  CRM->ctrl_bit.hicken = TRUE;

  /* wait hick stable */
  while(CRM->ctrl_bit.hickstbl != SET);

  /* hick used as system clock */
  CRM->cfg_bit.sclksel = CRM_SCLK_HICK;

  /* wait sclk switch status */
  while(CRM->cfg_bit.sclksts != CRM_SCLK_HICK);

  /* reset cfg register, include sclk switch, ahbdiv, apb1div, apb2div, adcdiv, clkout bits */
  CRM->cfg = (0x40000000U);

  /* reset hexten, hextbyps, cfden and pllen bits */
  CRM->ctrl &= ~(0x010D0000U);

  /* reset pllms pllns pllfr pllrcs bits */
  CRM->pllcfg = 0x000007C1U;

  /* reset clkout_sel, clkoutdiv, pllclk_to_adc, hick_to_usb */
  CRM->misc1 &= 0x00005000U;
  CRM->misc1 |= 0x000F0000U;

  /* disable all interrupts enable and clear pending bits  */
  CRM->clkint = 0x009F0000U;

  /* disable auto step mode */
  crm_auto_step_mode_enable(FALSE);

  qspi_psram_init();
  
#ifdef VECT_TAB_SRAM
  SCB->VTOR = SRAM_BASE  | VECT_TAB_OFFSET;  /* vector table relocation in internal sram. */
#else
  SCB->VTOR = FLASH_BASE | VECT_TAB_OFFSET;  /* vector table relocation in internal flash. */
#endif
}

/**
  * @brief  update system_core_clock variable according to clock register values.
  *         the system_core_clock variable contains the core clock (hclk), it can
  *         be used by the user application to setup the systick timer or configure
  *         other parameters.
  * @param  none
  * @retval none
  */
void system_core_clock_update(void)
{
  uint32_t pll_ns = 0, pll_ms = 0, pll_fr = 0, pll_clock_source = 0, pllrcsfreq = 0;
  uint32_t temp = 0, div_value = 0, psc = 0;
  crm_sclk_type sclk_source;

  static const uint8_t sys_ahb_div_table[16] = {0, 0, 0, 0, 0, 0, 0, 0, 1, 2, 3, 4, 6, 7, 8, 9};
  static const uint8_t pll_fp_table[16] = {1, 2, 4, 6, 8, 10, 12, 14, 16, 18, 20, 22, 24, 26, 28, 30};

  /* get sclk source */
  sclk_source = crm_sysclk_switch_status_get();

  switch(sclk_source)
  {
    case CRM_SCLK_HICK:
      if(((CRM->misc1_bit.hick_to_sclk) != RESET) && ((CRM->misc1_bit.hickdiv) != RESET))
        system_core_clock = HICK_VALUE * 6;
      else
        system_core_clock = HICK_VALUE;

      psc = CRM->misc2_bit.hick_to_sclk_div;
      system_core_clock = system_core_clock >> psc;
      break;
    case CRM_SCLK_HEXT:
      system_core_clock = HEXT_VALUE;
      psc = CRM->misc2_bit.hext_to_sclk_div;
      system_core_clock = system_core_clock >> psc;
      break;
    case CRM_SCLK_PLL:
      /* get pll clock source */
      pll_clock_source = CRM->pllcfg_bit.pllrcs;

      /* get multiplication factor */
      pll_ns = CRM->pllcfg_bit.pllns;
      pll_ms = CRM->pllcfg_bit.pllms;
      pll_fr = pll_fp_table[CRM->pllcfg_bit.pllfp];

      if (pll_clock_source == CRM_PLL_SOURCE_HICK)
      {
        /* hick selected as pll clock entry */
        pllrcsfreq = HICK_VALUE;
      }
      else
      {
        /* hext selected as pll clock entry */
        pllrcsfreq = HEXT_VALUE;
      }

      system_core_clock = (uint32_t)(((uint64_t)pllrcsfreq * pll_ns) / (pll_ms * pll_fr));
      break;
    default:
      system_core_clock = HICK_VALUE;
      break;
  }

  /* compute sclk, ahbclk frequency */
  /* get ahb division */
  temp = CRM->cfg_bit.ahbdiv;
  div_value = sys_ahb_div_table[temp];
  /* ahbclk frequency */
  system_core_clock = system_core_clock >> div_value;
}

/**
  * @brief  reduce power consumption initialize
  * @param  none
  * @retval none
  */
void reduce_power_consumption(void)
{
  volatile uint32_t delay = 0x34BC0;
  if(CRM->ctrl_bit.hextstbl)
  {
    *(__IO uint32_t *)0x40023878 = 0x00;
  }
  else if(CRM->ctrl_bit.pllstbl == SET)
  {
    CRM->pllcfg_bit.plluen = TRUE;
    while(CRM->ctrl_bit.pllstbl != SET || CRM->ctrl_bit.pllustbl != SET);
    *(__IO uint32_t *)0x40023878 = 0x10;
  }
  else
  {
    /* the pll or hext need to be enable */
    return;
  }
  CRM->ahben1 |= 1 << 29;
  *(__IO uint32_t *)0x40040038 = 0x210000;
  *(__IO uint32_t *)0x4004000C |= 0x40000000;
  *(__IO uint32_t *)0x40040804 &= ~0x2;
  while(delay --)
  {
    if(*(__IO uint32_t *)0x40040808 & 0x1)
      break;      
  }
  *(__IO uint32_t *)0x40040038 |= 0x400000;
  *(__IO uint32_t *)0x40040E00 |= 0x1;  
  *(__IO uint32_t *)0x40040038 &= ~0x10000;
  *(__IO uint32_t *)0x40023878 = 0x0;
  return;
}

static const qspi_cmd_type psram_qpi_mode_exit = {
FALSE,0,0xF5,QSPI_CMD_INSLEN_1_BYTE,0,QSPI_CMD_ADRLEN_0_BYTE,0,0,QSPI_OPERATE_MODE_444,QSPI_RSTSC_HW_AUTO,FALSE,TRUE};
static const qspi_cmd_type psram_rsten = {
FALSE,0,0x66,QSPI_CMD_INSLEN_1_BYTE,0,QSPI_CMD_ADRLEN_0_BYTE,0,0,QSPI_OPERATE_MODE_111,QSPI_RSTSC_HW_AUTO,FALSE,TRUE};
static const qspi_cmd_type psram_rst = {
FALSE,0,0x99,QSPI_CMD_INSLEN_1_BYTE,0,QSPI_CMD_ADRLEN_0_BYTE,0,0,QSPI_OPERATE_MODE_111,QSPI_RSTSC_HW_AUTO,FALSE,TRUE};
static const qspi_cmd_type psram_qpi_enable = {
FALSE,0,0x35,QSPI_CMD_INSLEN_1_BYTE,0,QSPI_CMD_ADRLEN_0_BYTE,0,0,QSPI_OPERATE_MODE_111,QSPI_RSTSC_HW_AUTO,FALSE,TRUE};
static const qspi_xip_type psram_xip_config = {
0xEB,QSPI_XIP_ADDRLEN_3_BYTE,QSPI_OPERATE_MODE_444,6,0x38,QSPI_XIP_ADDRLEN_3_BYTE,QSPI_OPERATE_MODE_444,0,QSPI_XIPW_SEL_MODED,0x7F,0x1F,QSPI_XIPR_SEL_MODET,0x7F,0x10};

/**
  * @brief  qspi psram initialize
  * @param  none
  * @retval none
  */
void qspi_psram_init(void)
{
  gpio_init_type gpio_init_struct;
 
  /* enable the qspi clock */
  crm_periph_clock_enable(CRM_QSPI1_PERIPH_CLOCK, TRUE);

  /* enable the pin clock */
  crm_periph_clock_enable(CRM_GPIOD_PERIPH_CLOCK, TRUE);

  /* set default parameter */
  gpio_default_para_init(&gpio_init_struct);

  /* configure the io0 gpio */
  gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
  gpio_init_struct.gpio_out_type  = GPIO_OUTPUT_PUSH_PULL;
  gpio_init_struct.gpio_mode = GPIO_MODE_MUX;
  gpio_init_struct.gpio_pins = GPIO_PINS_10 | GPIO_PINS_11 | GPIO_PINS_12 | GPIO_PINS_13 | GPIO_PINS_14 | GPIO_PINS_15;
  gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
  gpio_init(GPIOD, &gpio_init_struct);
  gpio_pin_mux_config(GPIOD, GPIO_PINS_SOURCE10, GPIO_MUX_11);
  gpio_pin_mux_config(GPIOD, GPIO_PINS_SOURCE11, GPIO_MUX_11);
  gpio_pin_mux_config(GPIOD, GPIO_PINS_SOURCE12, GPIO_MUX_11);
  gpio_pin_mux_config(GPIOD, GPIO_PINS_SOURCE13, GPIO_MUX_11);
  gpio_pin_mux_config(GPIOD, GPIO_PINS_SOURCE14, GPIO_MUX_11);
  gpio_pin_mux_config(GPIOD, GPIO_PINS_SOURCE15, GPIO_MUX_11);
  
  /* switch to cmd port */
  qspi_xip_enable(QSPI1, FALSE);

  /* set sclk */
  qspi_clk_division_set(QSPI1, QSPI_CLK_DIV_3);

  /* set sck idle mode 0 */
  qspi_sck_mode_set(QSPI1, QSPI_SCK_MODE_0);

  /* set wip in bit 0 */
  qspi_busy_config(QSPI1, QSPI_BUSY_OFFSET_0);

  qspi_xip_cache_bypass_set(QSPI1, TRUE);
  qspi_auto_ispc_enable(QSPI1);  
  
  /* qspi qpi exit */
  qspi_cmd_operation_kick(QSPI1, (qspi_cmd_type*)&psram_qpi_mode_exit);
  while(qspi_flag_get(QSPI1, QSPI_CMDSTS_FLAG) == RESET);
  qspi_flag_clear(QSPI1, QSPI_CMDSTS_FLAG);
  
  /* qspi system rest */
  qspi_cmd_operation_kick(QSPI1, (qspi_cmd_type*)&psram_rsten);
  while(qspi_flag_get(QSPI1, QSPI_CMDSTS_FLAG) == RESET);
  qspi_flag_clear(QSPI1, QSPI_CMDSTS_FLAG);
  qspi_cmd_operation_kick(QSPI1, (qspi_cmd_type*)&psram_rst);
  while(qspi_flag_get(QSPI1, QSPI_CMDSTS_FLAG) == RESET);
  qspi_flag_clear(QSPI1, QSPI_CMDSTS_FLAG);
  
  /* time between end of rst cmd to next valid cmd min 50ns */
  __NOP();

  /* qspi qpi enable */
  qspi_cmd_operation_kick(QSPI1, (qspi_cmd_type*)&psram_qpi_enable);
  while(qspi_flag_get(QSPI1, QSPI_CMDSTS_FLAG) == RESET);
  qspi_flag_clear(QSPI1, QSPI_CMDSTS_FLAG);
  
  /* qspi xip config,and switch to xip port*/
  qspi_xip_init(QSPI1, (qspi_xip_type*)&psram_xip_config);
  qspi_xip_enable(QSPI1, TRUE);
}

/**
  * @}
  */
  
/**
  * @}
  */

/**
  * @}
  */

