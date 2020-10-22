#ifdef SQUARE
  #define FN_NAME algoMonophonic3_square

  #define SETUP() \
    float duty=(waveParam+1)*28 +512; \
    if (duty<2*fr) duty=2*fr; \
    if (duty<2*f) duty=2*f; \
    if (duty<2*fl) duty=2*fl; \
    float pulseNorm=1.0-duty/4096.0; \
    \
    blepSquareRecalc(osc, &statel,fl,idtl,duty,pulseNorm); \
    blepSquareRecalc(osc2,&stater,fr,idtr,duty,pulseNorm); \
    blepSquareRecalc(osc3,&statew,f, idtw,duty,pulseNorm);

  #define PROCESS() \
      blepSquare(osc, &statel,fl,idtl,duty,pulseNorm) \
    + blepSquare(osc2,&stater,fr,idtr,duty,pulseNorm) \
    + blepSquare(osc3,&statew,f ,idtw,duty,pulseNorm)

#endif

#ifdef SAW
  #define FN_NAME algoMonophonic3_saw

  #define SETUP() \
    float incrl = fl/4096.0; \
    float incrr = fr/4096.0; \
    float incr = f/4096.0; \
    blepSawRecalc(osc, &statel,fl,idtl,incrl); \
    blepSawRecalc(osc2,&stater,fr,idtr,incrr); \
    blepSawRecalc(osc3,&statew,f, idtw,incr);

  #define PROCESS() \
      blepSaw(osc, &statel,fl,idtl,incrl) \
    + blepSaw(osc2,&stater,fr,idtr,incrr) \
    + blepSaw(osc3,&statew,f ,idtw,incr)

#endif



void FN_NAME(float f, uint16_t* buf, uint16_t* buf2) {

  float fr = f*detuneUp;
  float fl = f*detuneDown;

  struct oscillator* osc= &oscillators[0];
  struct oscillator* osc2= &oscillators[1];
  struct oscillator* osc3= &oscillators[7];

/*  if (fl>=2084.0 || fr>=2048.0 || osc->released) {
    for (uint16_t i = 0; i<BUFFERSIZE; i++) {buf[i] = buf2[i] = 2048;}
    return;
  }
*/

  #define s1 oscillators[2].fm_amplitude
  #define s2 oscillators[3].fm_amplitude
  #define s3 oscillators[4].fm_amplitude
  #define s4 oscillators[5].fm_amplitude
  #define cut oscillators[6].fm_amplitude
  #define preamp oscillators[0].fm_amplitude
  #define rclip 2.0

  float ampDiff = envelope(osc);

  if (osc->intAttack) {
    preamp += 1.0/((float)fm_attack_cc+0.5);
    if (preamp >= 1.0) {
      preamp = 1.0;
      osc->intAttack=0;
    }
  } else {
    preamp *=1.0-BUFFERSIZE*(1.0-fm_decay);
  }


  uint8_t statel, stater, statew;

  if ((targetWave&0x7f)==1)
    f*=0.75;
  else if ((targetWave&0x7f)==2)
    f*=0.5;
  else if ((targetWave&0x7f)==3)
    f*=0.25;
//  else if ((targetWave&0x7f)==4)
//    f=0.0;
//  else if ((targetWave&0x7f)==5)
//    { fr=0;fl=0; }

  float idtr=256.0/fr;
  float idtl=256.0/fl;
  float idtw=256.0/f;

  SETUP()

  float input;
  float res = fm_freq_cc[1]/21.167; // 0..6
  float tcut = fm_freq_cc[0]/127.0f;// 0..1

  //float track=fm_depth*0.0005;

  tcut += (f - 75.093) * 0.001968503937007874;

  if (tcut>1.0) tcut=1.0;
  else if (tcut<0.0) tcut=0.0;

  tcut *= preamp;

  res-=tcut*tcut*8; if(res<0)res=0;


  float cdiff = (tcut-cut)/((float)BUFFERSIZE);

  for (uint16_t i = 0; i<BUFFERSIZE; i++) {
    osc->amplitude += ampDiff;
    cut += cdiff;

    input = PROCESS();

    float resoclip = (s4-input)*res;
    if (resoclip > rclip) resoclip = rclip; else if (resoclip<-rclip) resoclip=-rclip;
    input = input - resoclip ;

    s1 = ((input - s1) * cut) + s1;
    s2 = ((s1 - s2) * cut) + s2;
    s3 = ((s2 - s3) * cut) + s3;
    s4 = ((s3 - s4) * cut) + s4;

    buf2[i] = buf[i]  = 2048 +s4 * osc->amplitude;

  }

  #undef s1
  #undef s2
  #undef s3
  #undef s4
  #undef cut
  #undef preamp
  #undef rclip
}

#undef FN_NAME
#undef SETUP
#undef PROCESS