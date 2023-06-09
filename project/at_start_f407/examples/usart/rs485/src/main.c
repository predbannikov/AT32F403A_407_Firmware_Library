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

/** @addtogroup AT32F407_periph_examples
  * @{
  */

/** @addtogroup F407_USART_rs485
  * @{
  */

#define RS485_BAUDRATE                       115200
#define RS485_BUFFER_SIZE                    16

uint8_t rs485_buffer_rx[RS485_BUFFER_SIZE];
uint8_t rs485_buffer_rx_cnt = 0; 
uint32_t counter = 0;

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
  
  usart_flag_clear(USART2, USART_RDBF_FLAG);
  usart_interrupt_enable(USART2, USART_RDBF_INT, TRUE);
  //usart_interrupt_enable(USART2, USART_IDLE_INT, TRUE);
	//usart_hardware_flow_control_set(USART2, USART_HARDWARE_FLOW_RTS_CTS );
  usart_receiver_enable(USART2, TRUE);
  usart_transmitter_enable(USART2, TRUE);
  usart_enable(USART2, TRUE);
  
  nvic_irq_enable(USART2_IRQn, 1, 0);
}


static void rs485_send_data(u8* buf, u8 cnt)
{
  gpio_bits_set(GPIOA, GPIO_PINS_1);
  while(cnt--){
    while(usart_flag_get(USART2, USART_TDBE_FLAG) == RESET);
    usart_data_transmit(USART2, *buf++);
  }
  while(usart_flag_get(USART2, USART_TDC_FLAG) == RESET);
  gpio_bits_reset(GPIOA, GPIO_PINS_1);
}

int main(void)
{
  char str[]="start test..\r\n";
  u8 len = 0;
  
  system_clock_config();
  at32_board_init();
  nvic_priority_group_config(NVIC_PRIORITY_GROUP_4);
  
	uart_print_init(115200);

  rs485_config();
  
  len = sizeof(str);
	if (0) {
		while (1) {
			rs485_send_data((u8*)str, len);
			delay_sec(1);
		}	
	} else {
		//gpio_bits_reset(GPIOA, GPIO_PINS_1);

		rs485_send_data((u8*)str, len);
	}

  while(1)
  {
    if(usart_flag_get(USART2, USART_IDLEF_FLAG) != RESET)
    {
      usart_data_receive(USART2);
      usart_interrupt_enable(USART2, USART_RDBF_INT, FALSE);
      rs485_send_data(rs485_buffer_rx, rs485_buffer_rx_cnt);
      rs485_buffer_rx_cnt = 0;
      usart_interrupt_enable(USART2, USART_RDBF_INT, TRUE);
    }  
  }
}

void USART2_IRQHandler(void)
{
  uint16_t tmp;
	//flag = usart_flag_get(USART2, USART_IDLE_INT);

  if(usart_flag_get(USART2, USART_RDBF_FLAG) != RESET)
  {
    tmp = usart_data_receive(USART2);
    if(rs485_buffer_rx_cnt < RS485_BUFFER_SIZE)
    {
      rs485_buffer_rx[rs485_buffer_rx_cnt++] = tmp;
    }
		//if (tmp == 'a') {
		//}
  } 
	if(usart_flag_get(USART2, USART_PERR_FLAG) != RESET)
	{
		printf("PERR\r\n");
	}
  if(usart_flag_get(USART2, USART_ROERR_FLAG) != RESET)
	{
		printf("ROERR\r\n");
	}  
	if(usart_flag_get(USART2, USART_FERR_FLAG) != RESET)
	{
		printf("FERR\r\n");
	}
  if(usart_flag_get(USART2, USART_NERR_FLAG) != RESET)
	{
		printf("NERR\r\n");
	}  
	if(usart_flag_get(USART2, USART_IDLEF_FLAG) != RESET)
	{
		printf("IDLEF\r\n");
	}
	if(usart_flag_get(USART2, USART_TDC_FLAG) != RESET)
	{
		printf("TDC\r\n");
	}
  if(usart_flag_get(USART2, USART_TDBE_FLAG) != RESET)
	{
		printf("TDBE\r\n");
	}
  if(usart_flag_get(USART2, USART_BFF_FLAG) != RESET)
	{
		printf("BFF\r\n");
	}
  if(usart_flag_get(USART2, USART_CTSCF_FLAG) != RESET)
	{
		printf("CTSCF\r\n");
	}


		//printf("%c,\t%i,\t%i", tmp, (uint8_t)tmp & 0x00FF, (uint8_t)((tmp & 0xFFFF)>>8));
	printf("%i\r\n\r\n", counter);
//	if (rs485_buffer_rx_cnt < 4) {
//		int flag = usart_flag_get(USART2, USART_RDBF_FLAG);
//		rs485_buffer_rx[rs485_buffer_rx_cnt] = 0;
//		printf("%i\tRDBF=%i\t%s",counter, flag, rs485_buffer_rx); 
//		printf("\r\n"); 
//     //usart_data_receive(USART2);
//      usart_interrupt_enable(USART2, USART_RDBF_INT, FALSE);
//      rs485_send_data(rs485_buffer_rx, rs485_buffer_rx_cnt);
//      rs485_buffer_rx_cnt = 0;
//      usart_interrupt_enable(USART2, USART_RDBF_INT, TRUE);	
		counter++;
//	}

}
