

#include "stm32l4xx_hal.h"
#include "stm32l4xx_ll_usart.h"
#include "math.h"

#define PI 3.14159265358979323846

#include "lookupTables.h"

DAC_HandleTypeDef    DacHandle;
UART_HandleTypeDef   huart1;
static DAC_ChannelConfTypeDef sConfig;

void SystemClock_Config(void);
static void TIM6_Config(void);
static void USART1_Init(void);
static void Error_Handler(void);


#define BUFFERSIZE 256
#define POLYPHONY 16
// To be absolutely sure it isn't going to clip, wave amplitude should be less than 2048/polyphony = 128...
#define WAVE_AMPLITUDE 128

uint16_t buffer[BUFFERSIZE*2] = {0};

  float fm_freq = 0.1;
  float fm_depth = 0.5;
  float fm_decay = 0.9999;

struct channel {
  uint8_t volume;
  int32_t bend;
  uint8_t RPNstack[4];
  uint8_t pbSensitivity;
  unsigned sustain:1;
} channels[16];

struct oscillator {
  uint8_t channel;
  uint8_t notenumber;
  float phase;
  float amplitude;
  float fm_phase;
  float fm_amplitude;
  uint32_t starttime;
  unsigned alive:1;
  unsigned released:1;
  unsigned sustained:1;
} oscillators[POLYPHONY];

uint32_t timestamp = 0;

void noteOn(uint8_t n, uint8_t chan) {

// find a free oscillator
// set it up
  uint8_t i, oldest;
  uint32_t mintime=0xffffffff;

  for (i=POLYPHONY; i--;) {
    if (0==oscillators[i].alive) break;
    if (oscillators[i].starttime < mintime) {
      oldest = i;
      mintime = oscillators[i].starttime;
    }
  }

  if (oscillators[i].alive!=0) {
    i = oldest;
  }

  oscillators[i].alive = 1;
  oscillators[i].starttime = timestamp++;
  oscillators[i].released = 0;
  oscillators[i].sustained = 0;
  oscillators[i].notenumber=n;
  oscillators[i].channel=chan;

  oscillators[i].fm_amplitude=fm_depth;
  //oscillators[i].phase = 1.0f;


}

void noteOff(uint8_t n, uint8_t chan) {
// find oscillator with same channel and note number
// kill it

  for (uint8_t i=POLYPHONY; i--;) {
    if (oscillators[i].alive && !oscillators[i].released && oscillators[i].notenumber==n && oscillators[i].channel==chan && !oscillators[i].sustained) {
      if (channels[chan].sustain) {
        oscillators[i].sustained=1;
      } else {
        oscillators[i].released=1;
      }
      break;
    }
  }

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
    if (bytenumber == 1) {
      bytetwo = i;
      bytenumber = 2;
    } else if (bytenumber == 2){

      uint8_t chan = status&0x0F;

      switch (status & 0xF0) {

        case 0x90: //Note on
          if (i == 0) noteOff(bytetwo, chan); //running status uses velocity=0 for noteoff
          else noteOn(bytetwo, chan);
        break;

        case 0x80: //Note off
          noteOff(bytetwo, chan);
        break;

        case 0xE0: //Pitch bend
          //channels[chan].bend = channels[chan].pbSensitivity * ( 1.0f -(float)((i<<7) + bytetwo) / (1<<13) ) ;
          channels[chan].bend = channels[chan].pbSensitivity * (((i<<7) + bytetwo) - 0x2000);
        break;


        case 0xB0: // Continuous controller

          switch (bytetwo) {

            case 20:
              fm_freq=(float)(i)/32;
            break;
            case 21:
              fm_depth=(float)(i)/127;
            break;
            case 22:
              fm_decay=0.999 + ((float)(i)/127000);
            break;

            case 64: //sustain pedal
              channels[chan].sustain = (i>63);
              if (channels[chan].sustain == 0) {//pedal released
                //loop through oscillators and release any with sustained=true
                for (uint8_t i=POLYPHONY; i--;) {
                  if (oscillators[i].sustained && oscillators[i].alive && !oscillators[i].released && oscillators[i].channel==chan) {
                    oscillators[i].released=1;
                    oscillators[i].sustained=0;
                  }
                }
              }
            break;

            case 101: channels[chan].RPNstack[0]=i; break;
            case 100: channels[chan].RPNstack[1]=i; break;
            case 38:  channels[chan].RPNstack[3]=i; break; //not used yet
            case 6:   channels[chan].RPNstack[2]=i; 
              if (channels[chan].RPNstack[0]==0 && channels[chan].RPNstack[1]==0) {//Pitch bend sensitivity
                // should this be limited to 24 ?
                channels[chan].pbSensitivity = i;
              }
            break;

          }
        break;

      }


      bytenumber =1; //running status
    }
  }


}


void doOscillator(struct oscillator* osc, uint16_t* buf){

/*
  with timer period 1451, sysclk 64MHz, real samplerate is 44107.51206

  f_a = (4*440) / fs = 4*440/(64M/1451) = 4*440*1451/64e6 = 0.0399025

*/

/*
  float f = 0.0399025 * pow(2.0, (
      (float)osc->notenumber
      - 69
      - channels[osc->channel].bend
    )/12 );
*/

  //int32_t b = channels[osc->channel].bend * channels[osc->channel].pbSensitivity ;

  float f = fEqualLookup[ osc->notenumber + (channels[osc->channel].bend/0x2000) ] 
          * bLookup14bit1semitone[ (channels[osc->channel].bend%0x2000) +0x2000 ];

  for (uint16_t i = 0; i<BUFFERSIZE; i++) {

    //Simple envelope
    if (osc->released) {
      osc->amplitude-=0.25;
      if (osc->amplitude <= 0.0) {osc->alive =0;osc->amplitude=0;}
    } else if (osc->amplitude<WAVE_AMPLITUDE) osc->amplitude+=0.25;


    osc->fm_phase += fm_freq*f;
    if (osc->fm_phase>4.0f) osc->fm_phase-=4.0f;


    osc->fm_amplitude *= fm_decay;

    osc->phase += f  + sinLut[(int)(osc->fm_phase *2048)]*osc->fm_amplitude;
    while (osc->phase>4.0f) osc->phase-=4.0f;
    while (osc->phase<0.0f) osc->phase+=4.0f;


    // if (osc->phase>2.0f) {
      // buf[i] += (4.0f - osc->phase -1.0f)*osc->amplitude;
    // } else {
      // buf[i] += (osc->phase -1.0f)*osc->amplitude;
    // }

    //sinLut is 8192 
    //phase is 0 to 4 -> *2048
    buf[i] += sinLut[(int)(osc->phase *2048)] * osc->amplitude;

  }


}


void generateIntoBuffer(uint16_t* buf){

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





int main(void) {

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
  

  for (uint8_t i=16;i--;) {
  //  channels[i].bend=0x2000;
    channels[i].pbSensitivity = 2;
  };


  /* Infinite loop */
  while (1);
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



static void Error_Handler(void)
{
  while(1){} 
}

