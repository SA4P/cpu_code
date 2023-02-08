/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  /* USER CODE BEGIN 2 */

  // (1) General setup

  // (1.1) Ringbuffers initialization

  // (1.1.1) LPUART1 ringbuffers initialization
  rb_init(&lp1_rx_rbuf, 'a');
  rb_init(&lp1_tx_rbuf, 'b');

  // (1.1.2) USART2 ringbuffers initialize
  rb_init(&us2_rx_rbuf, 'c');
  rb_init(&us2_tx_rbuf, 'd');

  // (1.2) Flags initialization

  // (1.2.1) Server flags initialization
  s_flags.rx_done = FALSE;
  s_flags.tx_rdy  = FALSE;

  // (1.2.2) Gateway flags initialization
  gw_flags.rx_done = FALSE;
  gw_flags.tx_rdy  = FALSE;

  // (1.3) Currently granting flag initialization
  flag_currently_granting = FALSE;

  // (1.4) Set up device type and capabilities URL
  dev_state.dev_cap		= "https://github.com/SA4P";
  dev_state.dev_type 	= 0;
  dev_state.dev_id		= -1;


  // (1) Initialize the two uarts
  lpuart = LPUART1;
  usart2 = USART2;

  // (1.1) Initialize the aliases which are used when working on a higher abstraction level
  s_uart   = lpuart;
  gw_usart = usart2;


  // (2) Initialize LPUART1

  // (2.1) Enable clock to LPUART1 and GPIO port G
  rcc_enable_lpuart();

  //  ONLY if we use GPIO to USB port: rcc_enable_gpiog();
  rcc_enable_gpioc();

  // (2.2) GPIO port G needs this extra step.
  rcc_set_vddio2();

  // (2.3) Set up GPIO port G such that the LPUART1-specific pins use their alternative function (i.e. LPUART1)

#ifdef LPUART_TO_USB
  // Case: LPUART goes to USB
  gpio_init_uart(lpuart, TRUE);

  // (2.4) Set up control registers of LPUART1
  usart_init_cr1(lpuart);
  usart_init_cr2(lpuart);
  usart_init_cr3(lpuart);
  usart_init_baudrate(lpuart, USART_BAUD_RATE);

  gpio_button_port = GPIOC;
  gpio_button_pin  = GPIO_IDR_ID13;
  gpio_init_button();

#else
  gpio_init_uart(lpuart, FALSE);

  // (2.4) Set up control registers of LPUART1
  usart_init_cr1(lpuart);
  usart_init_cr2(lpuart);
  usart_init_cr3(lpuart);
  usart_init_baudrate(lpuart, LPUART_BAUD_RATE);

  gpio_button_port = GPIOC;
  gpio_button_pin  = GPIO_IDR_ID13;
  gpio_init_button();

#endif

  // (2.5) Enable LPUART1
  usart_enable(lpuart);

  // (2.6) Enable transmission
  usart_enable_transmit(lpuart);

  // (2.7) Enable receiving
  usart_enable_receive(lpuart);

  // (2.8) Enable event generation (RXNE event, i.e. event when RDR register is not empty) inside LPUART1.
  usart_set_RXNEIE(lpuart);

  // (2.9) Enable LPUART1 interrupts. I.e. events generated inside LPUART1 now actually interrupt the CPU (if this flag was not set, events would not reach CPU)
  NVIC_EnableIRQ(LPUART1_IRQn);


  // (3) Initialize USART2
  rcc_enable_usart2();
  rcc_enable_gpiod();

  gpio_init_uart(usart2, FALSE);			// Second argument (to_usb, only used for LPUART1) is unused
  usart_init_cr1(usart2);
  usart_init_cr2(usart2);
  usart_init_cr3(usart2);
  usart_init_baudrate(usart2, USART_BAUD_RATE);

  usart_enable(usart2);
  usart_enable_transmit(usart2);

  usart_enable_receive(usart2);

  usart_set_RXNEIE(usart2);
  NVIC_EnableIRQ(USART2_IRQn);

  req_t req_state;
  send_msg_t o_msg;

//  byte_t rx_byte;
//  bool_t rx_success = FALSE;

  // msg_states, holding
  msg_t s_msg_state;
  msg_t gw_msg_state;

  // Key buffer holding HMAC keys (note: Padded with ZEROs to block size (512 bits == 64 bytes)
  // NOT used by the nonsecure world, but I kept it in here
  byte_t key_buf[64];
  memset(key_buf, 0, sizeof(key_buf));
  key_buf[0] = 0x69;

//  rb_produce(&us2_tx_rbuf, "HELLO WORLD!\r\n", sizeof("HELLO WORLD!\r\n"));
//
//  usart_set_TXEIE(usart2);

  // Set up profiling
  rcc_enable_gpioe();
  gpio_profiling_port_0 = GPIOE;

  rcc_enable_gpiof();
  gpio_profiling_port_1 = GPIOF;

  gpio_init_port_as_output(gpio_profiling_port_0);
  gpio_init_port_as_output(gpio_profiling_port_1);


#ifdef PROFILING_MACRO_SIGNUP_TIME
	  label_profiling_macro_signup_time:
#endif
  gpio_profiling_pin_0.port = gpio_profiling_port_0;     // PE2 on MCU, D56/SAI_A_MCLK on board
  gpio_profiling_pin_0.pin = GPIO_PIN_2;
  PROFILING_RESET_PIN(gpio_profiling_pin_0);

  gpio_profiling_pin_1.port = gpio_profiling_port_0;     // PE4 on MCU, D57/SAI_A_FS   on board
  gpio_profiling_pin_1.pin = GPIO_PIN_4;
  PROFILING_RESET_PIN(gpio_profiling_pin_1);

  gpio_profiling_pin_2.port = gpio_profiling_port_0;     // PE5 on MCU, D58/SAI_A_SCK  on board
  gpio_profiling_pin_2.pin = GPIO_PIN_5;
  PROFILING_RESET_PIN(gpio_profiling_pin_2);

  gpio_profiling_pin_3.port = gpio_profiling_port_0;     // PE6 on MCU, D59/SAI_A_SD   on board
  gpio_profiling_pin_3.pin = GPIO_PIN_6;
  PROFILING_RESET_PIN(gpio_profiling_pin_3);

  gpio_profiling_pin_4.port = gpio_profiling_port_0;     // PE3 on MCU, D60/SAI_B_SD   on board
  gpio_profiling_pin_4.pin = GPIO_PIN_3;
  PROFILING_RESET_PIN(gpio_profiling_pin_4);

  gpio_profiling_pin_5.port = gpio_profiling_port_1;     // PF8 on MCU, D56/SAI_B_SCK on board
  gpio_profiling_pin_5.pin = GPIO_PIN_8;
  PROFILING_RESET_PIN(gpio_profiling_pin_5);

  gpio_profiling_pin_6.port = gpio_profiling_port_1;     // PF7 on MCU, D56/SAI_B_MCLK on board
  gpio_profiling_pin_6.pin = GPIO_PIN_7;
  PROFILING_RESET_PIN(gpio_profiling_pin_6);

  gpio_profiling_pin_7.port = gpio_profiling_port_1;     // PF9 on MCU, D56/SAI_B_FS on board
  gpio_profiling_pin_7.pin = GPIO_PIN_9;
  PROFILING_RESET_PIN(gpio_profiling_pin_7);

//  PROFILING_SET_COUNTER_E(1);
//  HAL_Delay(1000);
//  PROFILING_SET_COUNTER_E(2);
//  HAL_Delay(1000);
//  PROFILING_SET_COUNTER_E(3);
//  HAL_Delay(1000);
//  PROFILING_SET_COUNTER_E(4);
//  HAL_Delay(1000);
//  PROFILING_SET_COUNTER_E(5);
//  HAL_Delay(1000);
//  PROFILING_SET_COUNTER_E(6);
//  HAL_Delay(1000);
//  PROFILING_SET_COUNTER_E(7);
//  HAL_Delay(1000);
//  PROFILING_SET_COUNTER_E(8);
//  HAL_Delay(1000);

  profiling_ctr_usart2 = 1;
  profiling_ctr_lpuart = 1;


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  // (4) Main loop
  while (1)
  {

	  // (1) Handle button press event
	  button_task_check_pressed(&req_state, &o_msg);
//	  if(button_task_check_pressed(&req_state, &o_msg)){
//		  // (1.1) If we reach here, the button was pressed and wasn't pressed before
//		  //	   ==> New request was generated
//	  }

	  // (2) Check if something can be sent to gateway
	  if(gw_flags.tx_rdy){
		  if(!gw_task_send(&o_msg)){

		  }
	  }

	  // (3) Handle gateway message event
	  // (3.1) Try to receive
	  gw_task_receive(&gw_msg_state);

	  // (3.2) If something was received from gateway, process it
	  if(gw_flags.rx_done){
		  if(!gw_task_process_msg(&gw_msg_state, &req_state, key_buf, &o_msg)){
			  __NOP();
		  }
		  // (3.3) If processing returns TRUE, this always means that s_flags.tx_rdy == TRUE. Nonetheless, we explicitly check it here
		  if(s_flags.tx_rdy){
			  s_task_send(&o_msg);
		  }
	  }

	  // (4) Handle server message event
	  // (4.1) Try to receive
	  s_task_receive(&s_msg_state);

	  // (4.2) If something was received from server, process it
	  if(s_flags.rx_done){
		  if(!s_task_process_msg(&s_msg_state, &req_state, key_buf, &o_msg)){
			  __NOP();
		  }

		  // (4.3) If processing returns TRUE, this always means that gw_flags.tx_rdy == TRUE. Nonetheless, we explicitly check it here
		  if(gw_flags.tx_rdy){
			  gw_task_send(&o_msg);
		  }

	  }

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE0) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 55;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
