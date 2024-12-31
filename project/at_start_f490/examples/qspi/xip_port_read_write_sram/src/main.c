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
#include <string.h>

/** @addtogroup AT32F490_periph_examples
  * @{
  */

/** @addtogroup 490_QSPI_xip_port_read_write_sram QSPI_xip_port_read_write_sram
  * @{
  */

#define TEST_SIZE                        4096
uint8_t wbuf[TEST_SIZE];
uint8_t rbuf[TEST_SIZE];

/**
  * @brief  main function.
  * @param  none
  * @retval none
  */
int main(void)
{
  uint16_t i, err = 0;
  uint8_t *qspi1_mem_addr;

  system_clock_config();
  at32_board_init();

  for(i = 0; i < TEST_SIZE; i++)
  {
    wbuf[i] = (uint8_t)i;
    rbuf[i] = 0;
  }

  qspi1_mem_addr = (uint8_t*)QSPI1_MEM_BASE;

  memcpy(qspi1_mem_addr, wbuf, TEST_SIZE);

  memcpy(rbuf, qspi1_mem_addr, TEST_SIZE);

  if(memcmp(rbuf, wbuf, TEST_SIZE))
  {
    err = 1;
  }

  while(1)
  {
    if(err == 0)
    {
      at32_led_toggle(LED3);
      delay_ms(300);
    }
    else
    {
      at32_led_toggle(LED2);
      delay_ms(300);
    }
  }
}

/**
  * @}
  */

/**
  * @}
  */
