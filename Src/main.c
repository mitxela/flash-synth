

#include "stm32l4xx_hal.h"
#include "stm32l4xx_ll_usart.h"
#include "math.h"
#include "string.h"

#define PI 3.14159265358979323846


// one of 32, 48, 64, 80
#define CPU_FREQ 64

#define DAC_TIMER_PERIOD ((CPU_FREQ*1000000)/(44100))

#include "lookupTables.h"
#include "defaultPatches.h"

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

#define DEFAULT_PB_RANGE 2

#define ARPEG_SIZE 32
#define ARPEG_RELEASE_CATCH 10

uint16_t buffer[BUFFERSIZE*2] = {0};
uint16_t buffer2[BUFFERSIZE*2] = {0};

  float fm_freq;
  float fm_depth;
  float fm_decay;
  float lfo_freq;
  float detuneUp;
  float detuneDown;

float releaseRate = -0.0025;
float attackRate = 0.25;
// float decayRate = 0.25;
// float sustainLevel = 0.5;
float fm_attack = 2.0;

uint8_t aa = 16;

struct channel {
  int32_t bend;
  uint8_t volume;
  uint8_t RPNstack[4];
  uint8_t pbSensitivity;
  uint8_t mod;
  float lfo_depth;
  const float* tuning;
  unsigned sustain:1;
} channels[16];

struct oscillator {
  uint8_t channel;
  uint8_t notenumber;
  uint8_t velocity;

  uint8_t stolen;
  uint8_t stolenChannel;

  float phase;
  float amplitude;
  float fm_phase;
  float fm_amplitude;
  float fm_depth_cache;
  float lfo_phase;
  uint32_t starttime;
  unsigned alive:1;
  unsigned released:1;
  unsigned sustained:1;
  unsigned intAttack:1;
} oscillators[POLYPHONY];

// TODO: unionize this with oscillator memory (if we get short of ram)
uint8_t monoNoteStack[ARPEG_SIZE*2];
uint8_t monoNoteEnd=0;
uint8_t monoNoteReleaseStack[ARPEG_SIZE*2];
uint8_t monoNoteReleaseEnd=0;
uint8_t monoNoteNow=0;
uint8_t monoNoteTimer=0;
uint8_t monoNoteReleaseTimer=0;
uint8_t arpegSpeed=10;
float portamento = 1.0;

uint32_t timestamp = 0;
uint8_t targetWave = 0;

float mainLut[8192]={0};

typedef void doOsc_t(struct oscillator* osc, uint16_t* buf);
doOsc_t oscAlgo1;
doOsc_t oscAlgo2;
doOsc_t *doOscillator = &oscAlgo1;

typedef void doOscStereo_t(struct oscillator* osc, struct oscillator* osc2, uint16_t* buf, uint16_t* buf2);
doOscStereo_t oscAlgo1Stereo;
doOscStereo_t oscAlgo2Stereo;
doOscStereo_t *doOscillatorStereo = &oscAlgo1Stereo;

typedef void monophonicAlgo_t(float f, uint16_t* buf, uint16_t* buf2);
monophonicAlgo_t algoMonophonic1;
monophonicAlgo_t algoMonophonic2;
monophonicAlgo_t *monophonicAlgo = &algoMonophonic2;

typedef void generateIntoBuffer_t(uint16_t* buf, uint16_t* buf2);
generateIntoBuffer_t generateIntoBufferFullPoly;
generateIntoBuffer_t generateIntoBufferDualOsc;
generateIntoBuffer_t generateIntoBufferMonophonic;
generateIntoBuffer_t *generateIntoBuffer = &generateIntoBufferFullPoly;

typedef void noteOn_t(uint8_t n, uint8_t vel, uint8_t chan);
noteOn_t noteOnFullPoly;
noteOn_t noteOnDualOsc;
noteOn_t noteOnMonophonic;
noteOn_t *noteOn = &noteOnDualOsc;

typedef void noteOff_t(uint8_t n, uint8_t chan);
noteOff_t noteOffFullPoly;
noteOff_t noteOffDualOsc;
noteOff_t noteOffMonophonic;
noteOff_t *noteOff = &noteOffDualOsc;

// For ease of re-ordering:
enum {
  wave_Sine = 0,
  wave_Hammondish,
  wave_SineCubed,
  wave_HalfSine,
  wave_AbsSine,
  wave_HardSquare,
  wave_SoftSquare,
  wave_FifthSquare,
  wave_HardSaw,
  wave_SoftSaw,
  wave_SineEven,
  wave_AbsSineEven,
  wave_HardPulse25,
  wave_HardPulse33
};

void setStereo(int stereo){

  // static bool previousState=0;
  // if (stereo == previousState) return;
  // previousState = stereo;

  // Completely stop and reinit the timer, otherwise there is a risk of 
  // desyncing and missing the first/last sample in each buffer
  TIM6_Config(); 

  if (HAL_DAC_Stop_DMA(&DacHandle, DAC_CHANNEL_1) != HAL_OK)
    Error_Handler();
  if (HAL_DAC_Stop_DMA(&DacHandle, DAC_CHANNEL_2) != HAL_OK)
    Error_Handler();

  if (HAL_DAC_ConfigChannel(&DacHandle, &sConfig, DAC_CHANNEL_1) != HAL_OK)
    Error_Handler();
  if (HAL_DAC_Start_DMA(&DacHandle, DAC_CHANNEL_1, (uint32_t *)buffer, BUFFERSIZE*2, DAC_ALIGN_12B_R) != HAL_OK)
    Error_Handler();
  if (HAL_DAC_ConfigChannel(&DacHandle, &sConfig, DAC_CHANNEL_2) != HAL_OK)
    Error_Handler();
  if (HAL_DAC_Start_DMA(&DacHandle, DAC_CHANNEL_2, 
        stereo ? ((uint32_t *)buffer2) : ((uint32_t *)buffer), 
        BUFFERSIZE*2, DAC_ALIGN_12B_R) != HAL_OK)
    Error_Handler();

}



void antialias(unsigned int radius){
// Must be symmetric

  float b[1024]={};
  float sum = 0;
  float scale = 1.0/((float)radius);
  unsigned int idx = 0;
  
  for (int i=0; i<radius; i++) {
    b[i] = mainLut[8192 - radius + i];
  }

  for (int i=0;i<8192;i++) {
    sum=0;
    for (int j=0; j<radius; j++) {
      sum+=b[j];
    }
    b[idx++] = mainLut[i];
    mainLut[i] = sum*scale;
    
    if (idx==radius) idx=0;
  }

}

void setWaveform(uint8_t id) {
  if (id==255) return;
// TODO: normalize waveforms

  // %8192 == &8191

  switch (id) {
  default:
  case wave_Sine:
    for (uint16_t i=0;i<8192;i++) {
      mainLut[i]=sinLut[i];
    }
    break;

  case wave_Hammondish:
    for (uint16_t i=0;i<8192;i++) {
      mainLut[i]=sinLut[(i)&8191]*0.25 + sinLut[(i*2)&8191]*0.25 + sinLut[(i*3) &8191]*0.25 + sinLut[(i*4) &8191]*0.25;
    }
    break;

  case wave_SineCubed:
    for (uint16_t i=0;i<8192;i++) {
      mainLut[i]=sinLut[i]*sinLut[i]*sinLut[i];
    }
    break;

  case wave_HalfSine:
    for (uint16_t i=0;i<4096;i++) {
      mainLut[i]=sinLut[i];
    }
    for (uint16_t i=4096;i<8192;i++) {
      mainLut[i]=0;
    }
    break;
  case wave_AbsSine:
    for (uint16_t i=0;i<8192;i++) {
      mainLut[i]= sinLut[i] > 0 ? sinLut[i] : -sinLut[i];
    }
    break;
  case wave_HardSquare:
    for (uint16_t i=0;i<4096;i++) {
      mainLut[i]=0.5;
    }
    for (uint16_t i=4096;i<8192;i++) {
      mainLut[i]=-0.5;
    }
    break;
  case wave_SoftSquare:
    { 
      uint16_t i=0;
      while (i<512) {
        mainLut[i]= (i/512.0) -0.5;
        i++;
      }
      while (i<4096) {
        mainLut[i]=0.5;
        i++;
      }
      while (i<4096+512) {
        mainLut[i]= ((512-i+4096)/512.0) -0.5;
        i++;
      }
      while (i<8192) {
        mainLut[i]=-0.5;
        i++;
      }
    }
    break;
  case wave_FifthSquare:
    for (uint16_t i=0;i<8192;i++) {
      mainLut[i]= (((i*2)&8191) >4192? 0.25:-0.25) + (((i*3)&8191) >4192? 0.25:-0.25);
    }
    break;
  case wave_HardSaw:
    {
      float j=-0.5;
      for (uint16_t i=0;i<8192;i++) {
        mainLut[i]= j+=1.0/8192.0;
      }
    }
    break;
  case wave_SoftSaw:
    {
      float j=0.5;
      for (uint16_t i=0;i<512;i++) {
        mainLut[i]= j-=1.0/512.0;
      }

      for (uint16_t i=512;i<8192;i++) {
        mainLut[i]= j+=1.0/8192.0;
      }
    }
    break;
  case wave_SineEven: // sine-even
    for (uint16_t i=0;i<4096;i++) {
      mainLut[i]= sinLut[i*2];
    }
    for (uint16_t i=4096;i<8192;i++) {
      mainLut[i]= 0.0;
    }
    break;
  case wave_AbsSineEven: // abs sine-even
    for (uint16_t i=0;i<2048;i++) {
      mainLut[i]= sinLut[i*2];
    }
    for (uint16_t i=2048;i<4096;i++) {
      mainLut[i]= sinLut[i*2-4096];
    }
    for (uint16_t i=4096;i<8192;i++) {
      mainLut[i]= 0.0;
    }
    break;
  case wave_HardPulse25: // hard pulse 25%
    for (uint16_t i=0;i<2048;i++) {
      mainLut[i]=0.5;
    }
    for (uint16_t i=2048;i<8192;i++) {
      mainLut[i]=-0.5;
    }
    break;
  case wave_HardPulse33: // hard pulse 33%
    for (uint16_t i=0;i<2731;i++) {
      mainLut[i]=0.5;
    }
    for (uint16_t i=2731;i<8192;i++) {
      mainLut[i]=-0.5;
    }
    break;


  }
  antialias((aa<<3)+8);
  antialias((aa<<3)+8);

  if (id==targetWave) targetWave=255;
}

void parameterChange(uint8_t chan, uint8_t cc, uint8_t i){
  static uint8_t fm_freq_cc[] = {0,0};

  switch (cc) {
    case 1: //modulation
      channels[chan].mod = i;
      channels[chan].lfo_depth = (float)(i*8);
    break;
    case 5: //portamento
      portamento = (float)(128-i) /128.0;
    break;

    case 76:
      lfo_freq = 204.8 + (float)(i*4);
    break;

    case 73: //Attack time
      attackRate = (0.128/(float)(i+1));
    break;
     case 75: //Decay time
      fm_attack = (0.01/((float)i+0.5));
     break;
    // case 74: //Sustain time
      // sustainLevel = (float)((i+1)*0.001);
    // break;
    case 72: //Release time
      releaseRate = -(0.128/(float)(i+1));
    break;

    case 15:{
        float detune= ((float)(i)/10160.0);
        detuneUp = 1.0 + detune;
        detuneDown = 1.0 - detune;
      }
    break;

    case 20:
      fm_freq_cc[0] = i;
      fm_freq=(float)((fm_freq_cc[0]<<7) + fm_freq_cc[1])/1024;
    break;
    case 21:
      fm_freq_cc[1] = i;
      fm_freq=(float)((fm_freq_cc[0]<<7) + fm_freq_cc[1])/1024;
    break;
    case 22:
      {
        // float factor = fm_depth;
        fm_depth=(float)(i*25)/127;
        // factor = fm_depth/factor; 
        // for (uint8_t j=POLYPHONY; j--;) {
          // oscillators[j].fm_amplitude *= factor;
        // }
      }
    break;
    case 23:
      fm_decay=1.0 - ((float)(i*i)/25400000);
    break;
    case 24:
      {
        for (int i=POLYPHONY;i--;) oscillators[i].alive=0;
        switch (i){
          case 1:
            doOscillatorStereo = &oscAlgo1Stereo;
            generateIntoBuffer = &generateIntoBufferDualOsc;
            noteOn = &noteOnDualOsc;
            noteOff = &noteOffDualOsc;
            setStereo(1);
            break;
          case 2:
            doOscillator = &oscAlgo2;
            generateIntoBuffer = &generateIntoBufferFullPoly;
            noteOn = &noteOnFullPoly;
            noteOff = &noteOffFullPoly;
            setStereo(0);
            break;

          case 3:
            doOscillatorStereo = &oscAlgo2Stereo;
            generateIntoBuffer = &generateIntoBufferDualOsc;
            noteOn = &noteOnDualOsc;
            noteOff = &noteOffDualOsc;
            setStereo(1);
            break;

          case 4:
            monophonicAlgo = &algoMonophonic1;
            generateIntoBuffer = &generateIntoBufferMonophonic;
            noteOn = &noteOnMonophonic;
            noteOff = &noteOffMonophonic;
            setStereo(1);
            break;


          case 5:
            monophonicAlgo = &algoMonophonic2;
            generateIntoBuffer = &generateIntoBufferMonophonic;
            noteOn = &noteOnMonophonic;
            noteOff = &noteOffMonophonic;
            setStereo(1);
            break;


          default:
            doOscillator = &oscAlgo1;
            generateIntoBuffer = &generateIntoBufferFullPoly;
            noteOn = &noteOnFullPoly;
            noteOff = &noteOffFullPoly;
            setStereo(0);
        }
      }
    break;

    case 25:
      targetWave = i;
    break;
    case 26:
      //aa = i;
      arpegSpeed=(i>>2);

      





      
    break;




    case 64: //sustain pedal
      channels[chan].sustain = (i>63);
      if (channels[chan].sustain == 0) {//pedal released
        if (noteOff == &noteOffMonophonic) {
          if (oscillators[0].released) break;

          for (int i=monoNoteEnd-2;i>=0;i-=2) {
            if (monoNoteStack[i]&0x80 && monoNoteStack[i+1]==chan) {
              for (int j=i; j<monoNoteEnd;j++) {
                monoNoteStack[j]=monoNoteStack[j+2];
              }
              monoNoteEnd-=2;
            }
          }

          if (monoNoteEnd) {
            monoNoteReleaseTimer=ARPEG_RELEASE_CATCH;
            if (monoNoteNow>=monoNoteEnd) monoNoteNow=monoNoteEnd-2;
            oscillators[0].notenumber=monoNoteStack[monoNoteNow]&0x7f;
          } else {
            oscillators[0].released=1;
            memcpy(monoNoteStack, monoNoteReleaseStack, monoNoteReleaseEnd);
            monoNoteEnd=monoNoteReleaseEnd;
          }

        } else {
          //loop through oscillators and release any with sustained=true
          for (uint8_t i=POLYPHONY; i--;) {
            if (oscillators[i].sustained && oscillators[i].alive && !oscillators[i].released && oscillators[i].channel==chan) {
              oscillators[i].released=1;
              oscillators[i].sustained=0;
            }
          }
        }
      }
    break;

    case 101: channels[chan].RPNstack[0]=i; break;
    case 100: channels[chan].RPNstack[1]=i; break;
    case 38:  channels[chan].RPNstack[3]=i; break; //not used yet
    case 6:   channels[chan].RPNstack[2]=i; 
      if (channels[chan].RPNstack[0]==0 && channels[chan].RPNstack[1]==0) {//Pitch bend sensitivity

        // For now, only allow changing bend range in 12ET mode
        if (channels[chan].tuning==&fEqualLookup[0]) {
          channels[chan].pbSensitivity = i; // should this be limited to 24 ?
        }
      }
    break;

    case 3: // Per-channel tuning
      if (i == 0) {
          channels[chan].pbSensitivity=DEFAULT_PB_RANGE;
          channels[chan].tuning = &fEqualLookup[0];
      } else if (i<=16) {
          channels[chan].pbSensitivity=1;
          channels[chan].tuning = &fTuningTables[i-1][0];
      }
    break;

    case 9: // All channels tuning
      if (i == 0) {
        for (int j=0;j<16;j++) {
          channels[j].pbSensitivity=DEFAULT_PB_RANGE;
          channels[j].tuning = &fEqualLookup[0];
        }
      } else {
        for (int j=0;j<16;j++) {
          channels[j].pbSensitivity=1;
          channels[j].tuning = &fTuningTables[j][0];
        }
      }
    break;
  }
}

void loadPatch(uint8_t p){
  // Link between byte position in the patch file and CC number
  const uint8_t paramMap[64] = {
    20,21,22,23,24,25,76,15 
  };
  for (uint8_t i=0;i<64;i++)
    parameterChange(0, paramMap[i], bPatches[p][i]);
}

inline void trigger_int_envelope(struct oscillator* osc, uint8_t vel, uint8_t n){

  osc->intAttack=1;
  osc->fm_amplitude=0;
  osc->fm_depth_cache = (vel/127.0)*fm_depth;// * fEqualLookup[ n ];

}

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmaybe-uninitialized"
void noteOnFullPoly(uint8_t n, uint8_t vel, uint8_t chan) {

// find a free oscillator
// set it up
  uint8_t i, oldest, similar = 255;
  uint32_t mintime=0xffffffff;

  for (i=POLYPHONY; i--;) {
    if (0==oscillators[i].alive) break;
    if (oscillators[i].notenumber == n) {
      similar = i;
    }
    if (oscillators[i].starttime < mintime) {
      oldest = i;
      mintime = oscillators[i].starttime;
    }
  }


  if (oscillators[i].alive!=0) {
    i = (similar==255)?oldest:similar;
    oscillators[i].stolen=n;
    oscillators[i].stolenChannel=chan;
  } else {
    oscillators[i].notenumber=n;
    oscillators[i].channel=chan;

    trigger_int_envelope(&oscillators[i], vel, n);
  }

  oscillators[i].alive = 1;
  oscillators[i].starttime = timestamp++;
  oscillators[i].released = 0;
  oscillators[i].sustained = 0;
  oscillators[i].velocity=vel;

  

}
#pragma GCC diagnostic pop

void noteOffFullPoly(uint8_t n, uint8_t chan) {
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

void noteOnDualOsc(uint8_t n, uint8_t vel, uint8_t chan) {

// find a free oscillator
// set it up
  uint8_t i, oldest, similar=255;
  uint32_t mintime=0xffffffff;

  for (i=0; i<POLYPHONY; i+=2) {
    if (0==oscillators[i].alive) break;
    if (oscillators[i].notenumber == n) {
      similar = i;
    }
    if (oscillators[i].starttime < mintime) {
      oldest = i;
      mintime = oscillators[i].starttime;
    }
  }

  if (i==POLYPHONY) {
    i = (similar==255)?oldest:similar;
    oscillators[i].stolen=n;
    oscillators[i].stolenChannel=chan;
  } else {
    oscillators[i].notenumber=n;
    oscillators[i].channel=chan;

    oscillators[i].stolen=0;

    trigger_int_envelope(&oscillators[i], vel, n);

  }

  oscillators[i].alive = 1;
  oscillators[i].starttime = timestamp++;
  oscillators[i].released = 0;
  oscillators[i].sustained = 0;
  oscillators[i].velocity=vel;



}

void noteOffDualOsc(uint8_t n, uint8_t chan) {
// find oscillator with same channel and note number
// kill it

  for (uint8_t i=0; i<POLYPHONY; i+=2) {
    if ( oscillators[i].alive 
     && !oscillators[i].released 
     && oscillators[i].notenumber==n 
     && oscillators[i].channel==chan 
     && !oscillators[i].sustained
     ){
      if (channels[chan].sustain) {
        oscillators[i].sustained=1;
      } else {
        oscillators[i].released=1;
      }
      break;
    }
  }

}

void noteOnMonophonic(uint8_t n, uint8_t vel, uint8_t chan) {

  if (oscillators[0].released) {
    monoNoteEnd=0;
    //oscillators[0].fm_amplitude = (vel/127.0)*fm_depth * fEqualLookup[ n ];
    trigger_int_envelope(&oscillators[0], vel, n);
  }

  for (int i=0; i<monoNoteEnd; i+=2){
    if ((monoNoteStack[i]&0x7F) == n && monoNoteStack[i+1]==chan) {
      monoNoteStack[i] = n;
      monoNoteNow=i;
      oscillators[0].notenumber=n;
      oscillators[0].velocity=vel;
      oscillators[0].channel=chan;
      return;
    }
  }
  if (monoNoteEnd==ARPEG_SIZE) return;

  monoNoteNow = monoNoteEnd;
  monoNoteStack[monoNoteEnd] = n;
  monoNoteStack[monoNoteEnd+1] = chan;
  monoNoteEnd+=2;
  monoNoteStack[monoNoteEnd]=0;
  monoNoteTimer=0;

  oscillators[0].notenumber=n;
  oscillators[0].velocity=vel;
  oscillators[0].channel=chan;
  oscillators[0].released=0;

  memcpy(monoNoteReleaseStack, monoNoteStack, monoNoteEnd+1);
  monoNoteReleaseEnd=monoNoteEnd;
  monoNoteReleaseTimer=0;
}
void noteOffMonophonic(uint8_t n, uint8_t chan) {

  if (channels[chan].sustain) {
    for (int i=0; i<monoNoteEnd; i+=2){
      if (monoNoteStack[i] == n) monoNoteStack[i]|=0x80;
    }
    return;
  }

  monoNoteReleaseTimer=ARPEG_RELEASE_CATCH;

  for (int i=0; i<monoNoteEnd; i+=2){
    if (monoNoteStack[i] == n && monoNoteStack[i+1]==chan) {
      while (i<monoNoteEnd+1) {
        monoNoteStack[i]=monoNoteStack[i+2];
        i++;
      }
      monoNoteEnd-=2;
    }
  }
  if (monoNoteEnd>0) {
    if (monoNoteNow>=monoNoteEnd) monoNoteNow=monoNoteEnd-2;
    oscillators[0].notenumber=monoNoteStack[monoNoteNow]&0x7f;
    oscillators[0].channel=monoNoteStack[monoNoteNow+1];
  } else {
    oscillators[0].released=1;
    memcpy(monoNoteStack, monoNoteReleaseStack, monoNoteReleaseEnd+1);
    monoNoteEnd=monoNoteReleaseEnd;
  }
}


uint32_t random(void) {
  static uint32_t seed = 123456;
  return seed = seed * 16807 % 0x7FFFFFFF;
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
    uint8_t chan = status&0x0F;

    if (bytenumber == 1) { // Check two-byte messages
      switch (status&0xF0) {

        case 0xD0: //After-touch
          if (i>=channels[chan].mod)
            channels[chan].lfo_depth = (float)(i*8);
          break;

        case 0xC0: //Program Change
          loadPatch(i);
          break;

        default:
          bytetwo = i;
          bytenumber = 2;
      }
    } else if (bytenumber == 2){

      switch (status&0xF0) {

        case 0x90: //Note on
          if (i == 0) noteOff(bytetwo, chan); //running status uses velocity=0 for noteoff
          else noteOn(bytetwo, i, chan);
        break;

        case 0x80: //Note off
          noteOff(bytetwo, chan);
        break;

        case 0xE0: //Pitch bend
          channels[chan].bend = channels[chan].pbSensitivity * (((i<<7) + bytetwo) - 0x2000);
        break;

        case 0xB0: // Continuous controller
          parameterChange(chan, bytetwo, i);
        break;
      }

      bytenumber =1; //running status
    }
  }


}

inline float calculateFrequency(struct oscillator* osc){

  osc->lfo_phase += lfo_freq;
  if (osc->lfo_phase>8192.0f) osc->lfo_phase-=8192.0f;

  float f;

  if (channels[osc->channel].pbSensitivity == 1) {

    // We should probably enforce that LFO depth is never more than a semitone
    f = channels[osc->channel].tuning[ osc->notenumber]
      * bLookup14bit1semitone[ channels[osc->channel].bend +0x2000 ]
      * bLookup14bit1semitone[ (int)(sinLut[(int)(osc->lfo_phase)] * channels[osc->channel].lfo_depth) +0x2000 ];

  } else {
    int32_t bend = channels[osc->channel].bend + (int)(sinLut[(int)(osc->lfo_phase)] * channels[osc->channel].lfo_depth);

    f = channels[osc->channel].tuning[ osc->notenumber + (bend>>13) ]
      * bLookup14bit1semitone[ (bend&0x1FFF) +0x2000 ];
  }

  return f;
}

// Generate a cachable envelope delta. Warp the edges so that state transitions always happen at the buffer boundaries
inline float envelope(struct oscillator* osc){
  float d = 0.0;

  if (osc->stolen) {
    return -osc->amplitude /BUFFERSIZE;
  }

  if (osc->released) {
    d = releaseRate;
    if (osc->amplitude < -releaseRate*BUFFERSIZE) {
      d = - osc->amplitude / BUFFERSIZE;
      osc->alive =0;
    }
  } else if (osc->amplitude < osc->velocity){
    d = attackRate;
    if (osc->amplitude > osc->velocity - attackRate*BUFFERSIZE) {
      d = (osc->velocity - osc->amplitude)/BUFFERSIZE;
    }
  }

  return d;
}

inline float intEnvelope(struct oscillator* osc){
  float d;
  if (osc->intAttack) {
    d = fm_attack;
    if (osc->fm_amplitude +fm_attack*BUFFERSIZE>=osc->fm_depth_cache) {
      d = (osc->fm_depth_cache - osc->fm_amplitude) / BUFFERSIZE;
      osc->intAttack=0;
    }
  } else {
    d = - osc->fm_amplitude*(1-fm_decay);
  }

  return d;
}

inline void graceful_theft(struct oscillator* osc){
  if (osc->stolen) {
    osc->amplitude=0.0;

    osc->channel = osc->stolenChannel;
    osc->notenumber = osc->stolen;
    osc->stolen = 0;

    trigger_int_envelope(osc, osc->velocity, osc->notenumber);
  }
}


void oscAlgo1(struct oscillator* osc, uint16_t* buf){

  float f = calculateFrequency(osc);
  float preAmpDiff = intEnvelope(osc);
  float ampDiff = envelope(osc);

  for (uint16_t i = 0; i<BUFFERSIZE; i++) {

    osc->amplitude += ampDiff;

    osc->fm_phase += fm_freq*f;
    if (osc->fm_phase>8192.0f) osc->fm_phase-=8192.0f;


    osc->fm_amplitude += preAmpDiff;

    osc->phase += f  + sinLut[(int)(osc->fm_phase)]*osc->fm_amplitude *f;
    while (osc->phase>8192.0f) osc->phase-=8192.0f;
    while (osc->phase<0.0f) osc->phase+=8192.0f;

    buf[i] += mainLut[(int)(osc->phase)] * osc->amplitude;

  }

  graceful_theft(osc);
}

void oscAlgo2(struct oscillator* osc, uint16_t* buf){

  float f = calculateFrequency(osc);

  float f2 = 8192.0/2.0;

  float preAmpDiff = intEnvelope(osc);
  float ampDiff = envelope(osc);

  for (uint16_t i = 0; i<BUFFERSIZE; i++) {

    osc->amplitude += ampDiff;


    osc->phase += f ;
    if (osc->phase>8192.0f) osc->phase-=8192.0f;

    osc->fm_amplitude += preAmpDiff;

    osc->fm_phase = mainLut[(int)(osc->phase)] * osc->fm_amplitude *f2;
    while (osc->fm_phase<0.0f) osc->fm_phase+=8192.0f;
    while (osc->fm_phase>=8192.0f) osc->fm_phase-=8192.0f;

    buf[i] += sinLut[(int)(osc->fm_phase)] * osc->amplitude;

  }

  graceful_theft(osc);
}


void oscAlgo1Stereo(struct oscillator* osc, struct oscillator* osc2, uint16_t* buf, uint16_t* buf2) {

  float f = calculateFrequency(osc);

  float fr = f*detuneUp;
  float fl = f*detuneDown;

  float preAmpDiff = intEnvelope(osc);
  float ampDiff = envelope(osc);

  for (uint16_t i = 0; i<BUFFERSIZE; i++) {

    osc->amplitude += ampDiff;

    osc->fm_phase += fm_freq*fl;
    if (osc->fm_phase>8192.0f) osc->fm_phase-=8192.0f;
    osc2->fm_phase += fm_freq*fr;
    if (osc2->fm_phase>8192.0f) osc2->fm_phase-=8192.0f;

    osc->fm_amplitude += preAmpDiff;

    osc->phase  += fl + sinLut[(int)( osc->fm_phase)]*osc->fm_amplitude *fl;
    osc2->phase += fr + sinLut[(int)(osc2->fm_phase)]*osc->fm_amplitude *fr;
    while (osc->phase>8192.0f) osc->phase-=8192.0f;
    while (osc->phase<0.0f) osc->phase+=8192.0f;
    while (osc2->phase>8192.0f) osc2->phase-=8192.0f;
    while (osc2->phase<0.0f) osc2->phase+=8192.0f;

    buf[i] += mainLut[(int)(osc->phase)] * osc->amplitude;
    buf2[i]+= mainLut[(int)(osc2->phase)]* osc->amplitude;

  }

  graceful_theft(osc);
}


void oscAlgo2Stereo(struct oscillator* osc, struct oscillator* osc2, uint16_t* buf, uint16_t* buf2){

  float f = calculateFrequency(osc);
  float f2 = 8192.0/2.0;

  float fr = f*detuneUp;
  float fl = f*detuneDown;

  float preAmpDiff = intEnvelope(osc);
  float ampDiff = envelope(osc);

  for (uint16_t i = 0; i<BUFFERSIZE; i++) {

    osc->amplitude += ampDiff;


    osc->phase += fl ;
    if (osc->phase>8192.0f) osc->phase-=8192.0f;
    osc2->phase += fr ;
    if (osc2->phase>8192.0f) osc2->phase-=8192.0f;

    osc->fm_amplitude += preAmpDiff;

    osc->fm_phase  = mainLut[(int)( osc->phase)] * osc->fm_amplitude *f2;
    osc2->fm_phase = mainLut[(int)(osc2->phase)] * osc->fm_amplitude *f2;
    while ( osc->fm_phase<0.0f) osc->fm_phase+=8192.0f;
    while ( osc->fm_phase>=8192.0f) osc->fm_phase-=8192.0f;
    while (osc2->fm_phase<0.0f) osc2->fm_phase+=8192.0f;
    while (osc2->fm_phase>=8192.0f) osc2->fm_phase-=8192.0f;

    buf[i] += sinLut[(int)( osc->fm_phase)] * osc->amplitude;
    buf2[i]+= sinLut[(int)(osc2->fm_phase)] * osc->amplitude;

  }

  graceful_theft(osc);
}


void algoMonophonic1(float f, uint16_t* buf, uint16_t* buf2) {

  struct oscillator* osc= &oscillators[0];
  struct oscillator* osc2= &oscillators[1];

  float fr = f*detuneUp;
  float fl = f*detuneDown;

  float preAmpDiff = intEnvelope(osc);
  float ampDiff = envelope(osc);

  for (uint16_t i = 0; i<BUFFERSIZE; i++) {
    osc->amplitude += ampDiff;

    osc->fm_phase += fm_freq*fl;
    if (osc->fm_phase>8192.0f) osc->fm_phase-=8192.0f;
    osc2->fm_phase += fm_freq*fr;
    if (osc2->fm_phase>8192.0f) osc2->fm_phase-=8192.0f;

    osc->fm_amplitude += preAmpDiff;

    osc->phase  += fl + sinLut[(int)( osc->fm_phase)]*osc->fm_amplitude;
    osc2->phase += fr + sinLut[(int)(osc2->fm_phase)]*osc->fm_amplitude;
    while (osc->phase>8192.0f) osc->phase-=8192.0f;
    while (osc->phase<0.0f) osc->phase+=8192.0f;
    while (osc2->phase>8192.0f) osc2->phase-=8192.0f;
    while (osc2->phase<0.0f) osc2->phase+=8192.0f;

    buf[i] = 2048+mainLut[(int)(osc->phase)] * osc->amplitude;
    buf2[i]= 2048+mainLut[(int)(osc2->phase)]* osc->amplitude;
  }

}
void algoMonophonic2(float f, uint16_t* buf, uint16_t* buf2) {

  struct oscillator* osc= &oscillators[0];
  struct oscillator* osc2= &oscillators[1];

  float fr = f*detuneUp;
  float fl = f*detuneDown;

  float ampDiff = envelope(osc);

  for (uint16_t i = 0; i<BUFFERSIZE; i++) {
    osc->phase += fl ;
    if (osc->phase>8192.0f) osc->phase-=8192.0f;
    osc2->phase += fr ;
    if (osc2->phase>8192.0f) osc2->phase-=8192.0f;

    osc->amplitude += ampDiff;

    buf[i]  = 2048+mainLut[(int)(osc->phase)] * osc->amplitude;
    buf2[i] = 2048+mainLut[(int)(osc2->phase)] * osc->amplitude;
  }

}


void generateIntoBufferFullPoly(uint16_t* buf, uint16_t* buf2){

  for (uint16_t i = 0; i<BUFFERSIZE; i++) {buf[i]=2048;}

  for (uint8_t i=POLYPHONY; i--;) {
    if (oscillators[i].alive)
      doOscillator(&oscillators[i], buf);
  }
}

void generateIntoBufferDualOsc(uint16_t* buf, uint16_t* buf2){

  for (uint16_t i = 0; i<BUFFERSIZE; i++) {buf[i]=2048; buf2[i]=2048;}

  for (uint8_t i=0; i<POLYPHONY; i+=2) {
    if (oscillators[i].alive)
      doOscillatorStereo(&oscillators[i], &oscillators[i+1], buf, buf2);
  }
}

void generateIntoBufferMonophonic(uint16_t* buf, uint16_t* buf2){

  static float f=0.0;
  struct oscillator* osc= &oscillators[0];

  float ft = calculateFrequency(osc);
  f += (ft-f)*portamento;

  if (arpegSpeed!=31 && ++monoNoteTimer>arpegSpeed) {
    monoNoteNow+=2;
    if (monoNoteNow>=monoNoteEnd) monoNoteNow=0;
    osc->notenumber = monoNoteStack[monoNoteNow]&0x7f;
    osc->channel = monoNoteStack[monoNoteNow+1];
    monoNoteTimer=0;
  }
  if (monoNoteReleaseTimer && osc->released==0 && --monoNoteReleaseTimer==0){
    memcpy(monoNoteReleaseStack, monoNoteStack, monoNoteEnd+1);
    monoNoteReleaseEnd=monoNoteEnd;
  }

  monophonicAlgo(f, buf, buf2);

}


void DMA1_Channel3_IRQHandler(void)
{
  DMA_HandleTypeDef *hdma = DacHandle.DMA_Handle1;
  uint32_t flag_it = hdma->DmaBaseAddress->ISR;
  uint32_t source_it = hdma->Instance->CCR;

  // Half Transfer Complete Interrupt
  if ((RESET != (flag_it & (DMA_FLAG_HT1 << hdma->ChannelIndex))) && (RESET != (source_it & DMA_IT_HT))) {
    hdma->DmaBaseAddress->IFCR = (DMA_ISR_HTIF1 << hdma->ChannelIndex);  //Clear Flag
    generateIntoBuffer(&buffer[0], &buffer2[0]);
  }

  // Transfer Complete Interrupt
  else if ((RESET != (flag_it & (DMA_FLAG_TC1 << hdma->ChannelIndex))) && (RESET != (source_it & DMA_IT_TC))) {
    hdma->DmaBaseAddress->IFCR = (DMA_ISR_TCIF1 << hdma->ChannelIndex);  //Clear Flag
    generateIntoBuffer(&buffer[BUFFERSIZE], &buffer2[BUFFERSIZE]);
  }

  return;
}





int main(void) {

  HAL_Init();
  SystemClock_Config();

   __HAL_RCC_GPIOA_CLK_ENABLE();

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
  
  if (HAL_DAC_ConfigChannel(&DacHandle, &sConfig, DAC_CHANNEL_2) != HAL_OK)
    Error_Handler();

  if (HAL_DAC_Start_DMA(&DacHandle, DAC_CHANNEL_2, (uint32_t *)buffer, BUFFERSIZE*2, DAC_ALIGN_12B_R) != HAL_OK)
    Error_Handler();
  
/*
fm_freq=1.0;
fm_depth=(float)(64*25)/127;
fm_decay = 0.9995 + ((float)(121)/254000);
lfo_freq = (0.1+(float)(48 )/512)* 2048;

setWaveform(0);
*/
  for (uint8_t i=16;i--;) {
    channels[i].pbSensitivity = DEFAULT_PB_RANGE;
    channels[i].tuning = &fEqualLookup[0];
  };
  
  loadPatch(0);

  while (1) {

    setWaveform(targetWave);

  };
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
#if CPU_FREQ == 32
  RCC_OscInitStruct.PLL.PLLN = 12;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV6;
#elif CPU_FREQ == 48
  RCC_OscInitStruct.PLL.PLLN = 12;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV4;
#elif CPU_FREQ == 64
  RCC_OscInitStruct.PLL.PLLN = 8;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
#else
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
#endif
  //not used
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

  htim.Init.Period            = DAC_TIMER_PERIOD;
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

