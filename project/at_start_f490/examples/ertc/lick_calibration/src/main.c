/**
  **************************************************************************
  * @file     main.c
  * @brief    main program
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

#include "at32f490_board.h"
#include "at32f490_clock.h"

/** @addtogroup AT32F490_periph_examples
  * @{
  */

/** @addtogroup 490_ERTC_lick_calibration ERTC_lick_calibration
  * @{
  */

__IO uint32_t   lick_freq = 0;
__IO uint32_t   capture_number = 0;
__IO uint32_t   period_value = 0;

void ertc_config(void);
uint32_t lick_frequency_get(void);
void ertc_clock_config(void);

/**
  * @brief  main program
  * @param  none
  * @retval none
  */
int main(void)
{
  ertc_time_type time;
  uint32_t temp = 0;

  /* The maximum frequency of the AHB is 120 MHz while accessing to
     CRM_BPDC and CRM_CTRLSTS registers. */
  ertc_clock_config();

  /* initial system clock */
  system_clock_config();

  /* initial the nvic priority group */
  nvic_priority_group_config(NVIC_PRIORITY_GROUP_4);

  /* at board initial */
  at32_board_init();

  /* initialize uart */
  uart_print_init(115200);

  /* config ertc or other operations of battery powered domain */
  ertc_config();

  /* get the lick frequency: TMR14 is used to measure the lick frequency */
  lick_freq = lick_frequency_get();

  /* configure the ertc divider */
  /* ertc second(1hz) = ertc_clk(lick) / (div_a + 1) * (div_b + 1) */
  ertc_divider_set(127, (lick_freq / 128) - 1);

  printf("lick_freq = %d\r\n", lick_freq);
  printf("div_a     = %d\r\n", 127);
  printf("div_b     = %d\r\n", (lick_freq / 128) - 1);
  printf("\r\n");

  while(1)
  {
    /* get the current time */
    ertc_calendar_get(&time);

    if(temp != time.sec)
    {
      temp = time.sec;

      /* display date format : year-month-day */
      printf("%02d-%02d-%02d ",time.year, time.month, time.day);

      /* display time format : hour:min:sec */
      printf("%02d:%02d:%02d\r\n",time.hour, time.min, time.sec);
    }
  }
}

/**
  * @brief  configure the ertc clock source.
  * @param  none
  * @retval none
  */
void ertc_clock_config(void)
{
  /* enable the pwc clock interface */
  crm_periph_clock_enable(CRM_PWC_PERIPH_CLOCK, TRUE);

  /* allow access to bpr domain */
  pwc_battery_powered_domain_access(TRUE);

  /* reset ertc domain */
  crm_battery_powered_domain_reset(TRUE);
  crm_battery_powered_domain_reset(FALSE);

  /* enable the lick osc */
  crm_clock_source_enable(CRM_CLOCK_SOURCE_LICK, TRUE);

  /* wait till lick is ready */
  while(crm_flag_get(CRM_LICK_STABLE_FLAG) == RESET)
  {
  }

  /* select the ertc clock source */
  crm_ertc_clock_select(CRM_ERTC_CLOCK_LICK);

  /* enable the ertc clock */
  crm_ertc_clock_enable(TRUE);
}

/**
  * @brief  configures the ertc peripheral by selecting the clock source.
  * @param  none
  * @retval none
  */
void ertc_config(void)
{
  /* deinitializes the ertc registers */
  ertc_reset();

  /* wait for ertc apb registers update */
  ertc_wait_update();

  /* configure the ertc divider */
  ertc_divider_set(0x7F, 0xFF);

  /* configure the ertc hour mode */
  ertc_hour_mode_set(ERTC_HOUR_MODE_24);

  /* set date: 2021-05-01 */
  ertc_date_set(21, 5, 1, 5);

  /* set time: 12:00:00 */
  ertc_time_set(12, 0, 0, ERTC_AM);
}

/**
  * @brief  configures TMR14 to measure the lick oscillator frequency.
  * @param  none
  * @retval lick frequency
  */
uint32_t lick_frequency_get(void)
{
  tmr_input_config_type  tmr_input_config_struct;
  crm_clocks_freq_type   crm_clock_freq;

  /* enable TMR14 apb1 clocks */
  crm_periph_clock_enable(CRM_TMR14_PERIPH_CLOCK, TRUE);

  /* connect internally the TMR14_ch4 to the lick clock output */
  tmr_iremap_config(TMR14, TMR14_ERTC_CLK);

  /* configure TMR14 divider */
  tmr_div_value_set(TMR14, 0);
  tmr_event_sw_trigger(TMR14, TMR_OVERFLOW_SWTRIG);

  /* TMR14 channel4 input capture mode configuration */
  tmr_input_config_struct.input_channel_select = TMR_SELECT_CHANNEL_1;
  tmr_input_config_struct.input_mapped_select = TMR_CC_CHANNEL_MAPPED_DIRECT;
  tmr_input_config_struct.input_polarity_select = TMR_INPUT_RISING_EDGE;
  tmr_input_config_struct.input_filter_value = 0;
  tmr_input_channel_init(TMR14, &tmr_input_config_struct, TMR_CHANNEL_INPUT_DIV_1);
  tmr_input_channel_divider_set(TMR14, TMR_SELECT_CHANNEL_1, TMR_CHANNEL_INPUT_DIV_8);

  /* enable TMR14 interrupt channel */
  nvic_irq_enable(TMR14_GLOBAL_IRQn, 0, 0);

  /* enable TMR14 counter */
  tmr_counter_enable(TMR14, TRUE);

  /* reset the flags */
  TMR14->swevt  = 0;

  /* enable the cc4 interrupt request */
  tmr_interrupt_enable(TMR14, TMR_C1_INT, TRUE);

  /* wait the TMR14 measuring operation to be completed */
  while(capture_number != 2);

  /* deinitialize the TMR14 peripheral */
  tmr_reset(TMR14);

  /* get the frequency value */
  crm_clocks_freq_get(&crm_clock_freq);

  /* get pclk1 divider */
  if ((CRM->cfg_bit.apb1div) > 0)
  {
    /* pclk1 divider different from 1 => timclk = 2 * pclk1 */
    return (((2 * crm_clock_freq.apb1_freq) / period_value) * 8) ;
  }
  else
  {
    /* pclk1 divider equal to 1 => timclk = pclk1 */
    return ((crm_clock_freq.apb1_freq / period_value) * 8);
  }
}

/**
  * @}
  */

/**
  * @}
  */
