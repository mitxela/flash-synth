

#include "stm32l4xx_hal.h"
#include "stm32l4xx_ll_usart.h"
#include "math.h"

#define PI 3.14159265358979323846

DAC_HandleTypeDef    DacHandle;
UART_HandleTypeDef   huart1;
static DAC_ChannelConfTypeDef sConfig;

void SystemClock_Config(void);
static void TIM6_Config(void);
static void USART1_Init(void);
static void Error_Handler(void);


#define BUFFERSIZE 512
#define POLYPHONY 12

uint16_t buffer[BUFFERSIZE*2] = {0};


struct channel {
  uint8_t volume;
  uint16_t bend;
} channels[8];

struct oscillator {
  struct channel * channel;
  uint8_t notenumber;
  float frequency;
  float phase;
  unsigned alive:1;
} oscillators[8];





float freq = 0;


uint8_t lastnote=0;

void noteOn(uint8_t n) {

// find a free oscillator
// set it up
  uint8_t i;

  for (i=POLYPHONY; i--;) {
    if (0==oscillators[i].alive) break;
  }

  oscillators[i].alive = 1;
  oscillators[i].notenumber=n;
  //oscillators[i].frequency=;



  lastnote =n;
}

void noteOff(uint8_t n) {
// find oscillator with same channel and note number
// kill it

  for (uint8_t i=POLYPHONY; i--;) {
    if (oscillators[i].alive && oscillators[i].notenumber==n) {
      oscillators[i].alive=0;
      //oscillators[i].frequency=0;
      break;
    }
  }

  //if (lastnote==n) oscillators[0].frequency= 0.0;
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


void doOscillator(struct oscillator* osc, uint16_t* buf){

  float f = pow(2.0, ((float)osc->notenumber -100)/12 );

  for (uint16_t i = 0; i<BUFFERSIZE; i++) {
    osc->phase += f;

    if (osc->phase>4.0f) osc->phase-=4.0f;

    if (osc->phase>2.0f) {
      buf[i] += (4.0f - osc->phase -1.0f)*350;
    } else {
      buf[i] += (osc->phase -1.0f)*350;
    }

  }


}


void generateIntoBuffer(uint16_t* buf){

  //memset(buf, 0, BUFFERSIZE*sizeof(uint16_t));
  for (uint16_t i = 0; i<BUFFERSIZE; i++) {buf[i]=2048;}

  for (uint8_t i=POLYPHONY; i--;) {
    if (oscillators[i].alive)
      doOscillator(&oscillators[i], buf);

  }





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


    // if (period!=0) {
      

      // for (uint16_t i =0; i<period; i++){
        // asm volatile("nop");

      // }
    // }
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

  htim.Init.Period            = 1451; // (64MHz / 44.1kHz)
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

