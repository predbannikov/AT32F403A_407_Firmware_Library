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

#include "at32f403a_407_board.h"
#include "at32f403a_407_clock.h"
#include "at32_emac.h"
#include "netconf.h"
#include "tcp_server.h"

/** @addtogroup AT32F407_periph_examples
  * @{
  */

/** @addtogroup 407_EMAC_tcp_server EMAC_tcp_server
  * @{
  */

#define DELAY                            100
#define FAST                             1
#define SLOW                             4
#define RS485_BAUDRATE                   115200
#define RS485_BUFFER_SIZE                8

uint8_t g_speed = FAST;
volatile uint32_t local_time = 0;

uint8_t rs485_buffer_rx[RS485_BUFFER_SIZE];
uint8_t rs485_buffer_rx_cnt = 0; 


static void rs485_config(void)
{
  gpio_init_type gpio_init_struct;

  /* enable the uart2 and gpio clock */
  crm_periph_clock_enable(CRM_USART2_PERIPH_CLOCK, TRUE);
  crm_periph_clock_enable(CRM_GPIOA_PERIPH_CLOCK, TRUE);

  gpio_default_para_init(&gpio_init_struct);

  /* configure the uart2 tx,rx,de pin */
  gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
  gpio_init_struct.gpio_out_type  = GPIO_OUTPUT_PUSH_PULL;
  gpio_init_struct.gpio_mode = GPIO_MODE_MUX;
  gpio_init_struct.gpio_pins = GPIO_PINS_2;
  gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
  gpio_init(GPIOA, &gpio_init_struct);
  
  gpio_init_struct.gpio_pins = GPIO_PINS_3;
  gpio_init_struct.gpio_mode = GPIO_MODE_INPUT;
  gpio_init_struct.gpio_pull = GPIO_PULL_UP;
  gpio_init(GPIOA, &gpio_init_struct);
  
  gpio_init_struct.gpio_pins = GPIO_PINS_1;
  gpio_init_struct.gpio_mode = GPIO_MODE_OUTPUT;
  gpio_init_struct.gpio_out_type  = GPIO_OUTPUT_PUSH_PULL;
  gpio_init(GPIOA, &gpio_init_struct);
  
  gpio_bits_reset(GPIOA, GPIO_PINS_1);
  
  /* configure uart2 param */
  usart_init(USART2, RS485_BAUDRATE, USART_DATA_8BITS, USART_STOP_1_BIT);
  usart_parity_selection_config(USART2, USART_PARITY_NONE);
	//usart_hardware_flow_control_set(USART2, USART_HARDWARE_FLOW_RTS);
  usart_flag_clear(USART2, USART_RDBF_FLAG);
  usart_interrupt_enable(USART2, USART_RDBF_INT, TRUE);
  //usart_interrupt_enable(USART2, USART_IDLE_INT, TRUE);
	//usart_hardware_flow_control_set(USART2, USART_HARDWARE_FLOW_RTS_CTS );
  usart_receiver_enable(USART2, TRUE);
  usart_transmitter_enable(USART2, TRUE);
  usart_enable(USART2, TRUE);
  
  nvic_irq_enable(USART2_IRQn, 2, 0);
}

int main(void)
{
  error_status status;

  system_clock_config();

  at32_board_init();
  
  //uart_print_init(115200);

  
  nvic_priority_group_config(NVIC_PRIORITY_GROUP_4);

  delay_init();

  status = emac_system_init();

  while(status == ERROR);

  tcpip_stack_init();

  tcp_server_init();
	
	rs485_config();

  for(;;)
  {
    /* lwip receive handle */
    lwip_rx_loop_handler();
    
    /*timeout handle*/
    lwip_periodic_handle(local_time);
  }
}

/**
  * @}
  */

/**
  * @}
  */
