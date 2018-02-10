

#include "stm32l4xx_hal.h"
#include "stm32l4xx_ll_usart.h"
#include "math.h"

DAC_HandleTypeDef    DacHandle;
UART_HandleTypeDef   huart1;
static DAC_ChannelConfTypeDef sConfig;


void SystemClock_Config(void);
static void TIM6_Config(void);
static void USART1_Init(void);
static void Error_Handler(void);


#define BUFFERSIZE 1024

uint16_t buffer[BUFFERSIZE*2] = {

//0,1000,0,1000,0,1000,0,1000};



0,260,390,520,650,780,910,1040,
1170,1300,1430,1560,1690,1820,1950,2080,
2210,2340,2470,2600,2730,2860,2990,3120,
3250,3380,3510,3640,3770,3900,4030,4095,
4030,3900,3770,3640,3510,3380,3250,3120,
2990,2860,2730,2600,2470,2340,2210,2080,
1950,1820,1690,1560,1430,1300,1170,1040,
910,780,650,520,390,260,130,0};

// 10,500,10,10,10,10,10,10,
// 10,10,10,10,10,10,10,10,
// 10,10,10,10,10,10,10,10,
// 10,10,10,10,10,10,10,10,
// 4000,4000,200,4000,4000,4000,4000,4000,
// 4000,4000,4000,4000,4000,4000,4000,4000,
// 4000,4000,4000,4000,4000,4000,4000,4000,
// 4000,4000,4000,4000,4000,4000,4000,4000};

// 1,2,3,4,5,6,7,8,
// 0,0,0,0,0,0,0,0,
// 0,0,0,0,0,0,0,0,
// 0,0,0,0,0,0,0,0,
// 0,0,0,0,0,0,0,0,
// 0,0,0,0,0,0,0,0,
// 0,0,0,0,0,0,0,0,
// 0,0,0,0,0,0,0,0};







float period = 0;

uint8_t lastnote=0;

void noteOn(uint8_t n) {
  period = pow(2.0, ((float)n -100)/12 );

  //buffer[0]=(uint16_t)n *32;


  lastnote =n;
}

void noteOff(uint8_t n) {
  if (lastnote==n) period = 0.0;
}



void USART1_IRQHandler(void) {
  static uint8_t status=0;
  static uint8_t bytenumber =0;
  static uint8_t bytetwo = 0;

  uint8_t i = LL_USART_ReceiveData8(USART1);

  if (i & 0x80) {
    status = i;
    bytenumber = 1;

  } else {
    if (bytenumber ==1) {
      bytetwo = i;
      bytenumber= 2;
    } else if (bytenumber==2){

      switch (status & 0xF0) {

        case 0x90: //Note on
          if (i == 0) noteOff(bytetwo);
          else noteOn(bytetwo);
        break;

        case 0x80: //Note off
          noteOff(bytetwo);
        break;

      }


      bytenumber =1; //running status
    }
  }


}



#define PI 3.1415926535897

void generateIntoBuffer(uint16_t* buf){

//static uint16_t j=0;
static float phase = 0;

   for (uint16_t i = 0; i<BUFFERSIZE; i++) {

     phase += period;
     buf[i] = phase*500;//(uint16_t)(sin(phase)*1024+1024);
     if (phase>2*PI) phase -= PI;


  //    buf[i]++;
   }

//buf[5]+=++j;

}



void DMA1_Channel3_IRQHandler(void)
{


  DMA_HandleTypeDef *hdma = DacHandle.DMA_Handle1;
  uint32_t flag_it = hdma->DmaBaseAddress->ISR;
  uint32_t source_it = hdma->Instance->CCR;



  // Half Transfer Complete Interrupt
  if ((RESET != (flag_it & (DMA_FLAG_HT1 << hdma->ChannelIndex))) && (RESET != (source_it & DMA_IT_HT))) {
    hdma->DmaBaseAddress->IFCR = (DMA_ISR_HTIF1 << hdma->ChannelIndex);  //Clear Flag

    generateIntoBuffer(&buffer[0]);

  }

  // Transfer Complete Interrupt
  else if ((RESET != (flag_it & (DMA_FLAG_TC1 << hdma->ChannelIndex))) && (RESET != (source_it & DMA_IT_TC))) {

    hdma->DmaBaseAddress->IFCR = (DMA_ISR_TCIF1 << hdma->ChannelIndex);  //Clear Flag

    generateIntoBuffer(&buffer[BUFFERSIZE]);
// static uint16_t j=0;
// buffer[BUFFERSIZE+3]+=j++;

    __HAL_UNLOCK(hdma);

  }


  return;

}





int main(void)
{

  HAL_Init();
  SystemClock_Config();

   __HAL_RCC_GPIOA_CLK_ENABLE();


  // __HAL_RCC_GPIOC_CLK_ENABLE();
  // GPIO_InitTypeDef GPIO_InitStruct;
  // GPIO_InitStruct.Pin = GPIO_PIN_0;
  // GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  // GPIO_InitStruct.Pull = GPIO_NOPULL;
  // GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  // HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  // HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_0);

  
  USART1_Init();

  DacHandle.Instance = DAC1;
  TIM6_Config();




  if (HAL_DAC_Init(&DacHandle) != HAL_OK)
    Error_Handler();
  
  sConfig.DAC_Trigger = DAC_TRIGGER_T6_TRGO;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;

  if (HAL_DAC_ConfigChannel(&DacHandle, &sConfig, DAC_CHANNEL_1) != HAL_OK)
    Error_Handler();

  if (HAL_DAC_Start_DMA(&DacHandle, DAC_CHANNEL_1, (uint32_t *)buffer, BUFFERSIZE*2, DAC_ALIGN_12B_R) != HAL_OK)
    Error_Handler();
  



  /* Infinite loop */
  while (1) {


    if (period!=0) {
      

      for (uint16_t i =0; i<period; i++){
        asm volatile("nop");

      }
    }
  }
}

void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 8;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    Error_Handler();
  
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;  
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;  
  if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
    Error_Handler();

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
    Error_Handler();

}


void USART1_Init(void) {

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 31250;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;

  if (HAL_UART_Init(&huart1) != HAL_OK)
    Error_Handler();

  NVIC_SetPriority(USART1_IRQn, 0);
  NVIC_EnableIRQ(USART1_IRQn);
  LL_USART_Enable(USART1);
  LL_USART_EnableIT_RXNE(USART1);

}


/**
  * @brief  TIM6 Configuration
  * @note   TIM6 configuration is based on APB1 frequency
  * @note   TIM6 Update event occurs each TIM6CLK/256
  * @param  None
  * @retval None
  */
void TIM6_Config(void)
{
  static TIM_HandleTypeDef  htim;
  TIM_MasterConfigTypeDef sMasterConfig;

  /*##-1- Configure the TIM peripheral #######################################*/
  /* Time base configuration */
  htim.Instance = TIM6;

  htim.Init.Period            = 1684; // (64MHz / 38kHz) -> 19kHz nyquist
  htim.Init.Prescaler         = 0;
  htim.Init.ClockDivision     = 0;
  htim.Init.CounterMode       = TIM_COUNTERMODE_UP;
  htim.Init.RepetitionCounter = 0;
  HAL_TIM_Base_Init(&htim);

  /* TIM6 TRGO selection */
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;

  HAL_TIMEx_MasterConfigSynchronization(&htim, &sMasterConfig);

  /*##-2- Enable TIM peripheral counter ######################################*/
  HAL_TIM_Base_Start(&htim);
}




/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
static void Error_Handler(void)
{
  while(1){} 
}

