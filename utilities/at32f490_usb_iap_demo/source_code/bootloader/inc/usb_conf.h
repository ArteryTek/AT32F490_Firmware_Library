/**
  **************************************************************************
  * @file     usb_conf.h
  * @brief    usb config header file
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

/* define to prevent recursive inclusion -------------------------------------*/
#ifndef __USB_CONF_H
#define __USB_CONF_H

#ifdef __cplusplus
extern "C" {
#endif

#include "at32f490_usb.h"
#include "at32f490.h"
#include "at32f490_board.h"

/** @addtogroup UTILITIES_examples
  * @{
  */

/** @addtogroup USB_iap_bootloader
  * @{
  */

/**
  * @brief enable usb device mode
  */
#define USE_OTG_DEVICE_MODE

/**
  * @brief enable usb host mode
  */
/* #define USE_OTG_HOST_MODE */

/**
  * @brief otgfs define
  */

#ifdef USB_OTG_FS
#define USB_ID                           USB_OTG1_ID
#define OTG_CLOCK                        CRM_OTGFS1_PERIPH_CLOCK
#define OTG_IRQ                          OTGFS1_IRQn
#define OTG_IRQ_HANDLER                  OTGFS1_IRQHandler
#define OTG_WKUP_IRQ                     OTGFS1_WKUP_IRQn
#define OTG_WKUP_HANDLER                 OTGFS1_WKUP_IRQHandler
#define OTG_WKUP_EXINT_LINE              EXINT_LINE_18

#define OTG_PIN_GPIO                     GPIOA
#define OTG_PIN_GPIO_CLOCK               CRM_GPIOA_PERIPH_CLOCK

#define OTG_PIN_VBUS                     GPIO_PINS_9
#define OTG_PIN_VBUS_SOURCE              GPIO_PINS_SOURCE9

#define OTG_PIN_ID                       GPIO_PINS_10
#define OTG_PIN_ID_SOURCE                GPIO_PINS_SOURCE10

#define OTG_PIN_SOF_GPIO                 GPIOA
#define OTG_PIN_SOF_GPIO_CLOCK           CRM_GPIOA_PERIPH_CLOCK
#define OTG_PIN_SOF                      GPIO_PINS_8
#define OTG_PIN_SOF_SOURCE               GPIO_PINS_SOURCE8

#define OTG_PIN_MUX                      GPIO_MUX_10

#define USB_SPEED_CORE_ID                USB_FULL_SPEED_CORE_ID
#endif

/**
  * @brief usb device mode config
  */
#ifdef USE_OTG_DEVICE_MODE
/**
  * @brief usb device mode fifo
  */
/* otg fs device fifo 
   otg fs fifo size is 1280 byte
*/

#define USBD_RX_SIZE                     128  /*this value is in terms of 4 bytes*/
#define USBD_EP0_TX_SIZE                 24   /*this value is in terms of 4 bytes*/
#define USBD_EP1_TX_SIZE                 20   /*this value is in terms of 4 bytes*/
#define USBD_EP2_TX_SIZE                 20   /*this value is in terms of 4 bytes*/
#define USBD_EP3_TX_SIZE                 20   /*this value is in terms of 4 bytes*/
#define USBD_EP4_TX_SIZE                 20   /*this value is in terms of 4 bytes*/
#define USBD_EP5_TX_SIZE                 20   /*this value is in terms of 4 bytes*/
#define USBD_EP6_TX_SIZE                 20   /*this value is in terms of 4 bytes*/
#define USBD_EP7_TX_SIZE                 20   /*this value is in terms of 4 bytes*/


/* otg hs device fifo 
   otg hs fifo size is 4096 byte
*/
#define USBD2_RX_SIZE                    256  /*this value is in terms of 4 bytes*/
#define USBD2_EP0_TX_SIZE                64   /*this value is in terms of 4 bytes*/
#define USBD2_EP1_TX_SIZE                256  /*this value is in terms of 4 bytes*/
#define USBD2_EP2_TX_SIZE                20   /*this value is in terms of 4 bytes*/
#define USBD2_EP3_TX_SIZE                20   /*this value is in terms of 4 bytes*/
#define USBD2_EP4_TX_SIZE                20   /*this value is in terms of 4 bytes*/
#define USBD2_EP5_TX_SIZE                20   /*this value is in terms of 4 bytes*/
#define USBD2_EP6_TX_SIZE                20   /*this value is in terms of 4 bytes*/
#define USBD2_EP7_TX_SIZE                20   /*this value is in terms of 4 bytes*/

/**
  * @brief usb endpoint max num define
  */
#ifndef USB_EPT_MAX_NUM
#define USB_EPT_MAX_NUM                   8
#endif
#endif

/**
  * @brief usb host mode config
  */
#ifdef USE_OTG_HOST_MODE
#ifndef USB_HOST_CHANNEL_NUM
#define USB_HOST_CHANNEL_NUM             16
#endif

/**
  * @brief usb host mode fifo
  */
/* otg1 host fifo 
   otg fs fifo size is 1280 byte
*/
#define USBH_RX_FIFO_SIZE                128  /*this value is in terms of 4 bytes*/
#define USBH_NP_TX_FIFO_SIZE             96   /*this value is in terms of 4 bytes*/
#define USBH_P_TX_FIFO_SIZE              96   /*this value is in terms of 4 bytes*/


/* otg2 host fifo 
   otg hs fifo size is 4096 byte
*/
#define USBH2_RX_FIFO_SIZE               256  /*this value is in terms of 4 bytes*/
#define USBH2_NP_TX_FIFO_SIZE            256  /*this value is in terms of 4 bytes*/
#define USBH2_P_TX_FIFO_SIZE             256  /*this value is in terms of 4 bytes*/
#endif

/**
  * @brief usb sof output enable
  */
/* #define USB_SOF_OUTPUT_ENABLE */

/**
  * @brief usb vbus ignore
  */
#define USB_VBUS_IGNORE

/**
  * @brief usb low power wakeup handler enable
  */
/* #define USB_LOW_POWER_WAKUP */

/**
  * @brief usb high speed support dma mode
  */
/* #define OTG_USE_DMA */

void usb_delay_ms(uint32_t ms);
void usb_delay_us(uint32_t us);

/**
  * @}
  */

/**
  * @}
  */
#ifdef __cplusplus
}
#endif

#endif
