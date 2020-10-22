#ifdef SQUARE
  #define FN_NAME oscAlgo3_square

  #define SETUP() \
    float duty= (waveParam+1)*28 +512; \
    if (duty<2*f) duty=2*f; \
    float pulseNorm=1.0-duty/4096.0; \
    blepSquareRecalc(osc,&state,f,idt,duty,pulseNorm);

  #define PROCESS() blepSquare(osc,&state,f,idt,duty,pulseNorm)
#endif

#ifdef SAW
  #define FN_NAME oscAlgo3_saw

  #define SETUP() \
    float incr = f/4096.0; \
    blepSawRecalc(osc,&state,f,idt,incr);

  #define PROCESS() blepSaw(osc,&state,f,idt, incr)
#endif

void FN_NAME(struct oscillator* osc, uint16_t* buf){

  phase_incr(osc->lfo_phase, lfo_freq)
  float f = calculateFrequency(osc);
  if (f>=2048.0) return;
  float idt=256.0/f;
  uint8_t state;

  envelopeZero_setup()
  float ampDiff = envelopeZero(osc);

  SETUP()

  for (uint16_t i = 0; i<BUFFERSIZE; i++) {
    osc->amplitude += ampDiff;
    buf[i] += PROCESS() * osc->amplitude;
  }

  graceful_theft(osc);
}

#undef FN_NAME
#undef SETUP
#undef PROCESS
