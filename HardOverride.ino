//Main portion of this code was adapted from https://github.com/povlhp/iBus2PPM/
//Special Thanks to Povl H. Pedersen

//Radio Tx and Rx were prepared and hacked with the method given at https://github.com/povlhp/iBus2PPM/blob/master/iBus2PPM-instructions.md

//Code for Arduino Pro Micro (Leonardo)

#define PPM_CHANS 8           // The number of iBus channels to send as PPM.
#define IBUS_BUFFSIZE 32      // Max iBus packet size (2 byte header, 14 channels x 2 bytes, 2 byte checksum)
#define OVERRIDE_BUFFSIZE 16

//////////////////////PPM CONFIGURATION///////////////////////////////
///// PPM_FrLen might be lowered a bit for higher refresh rates, as all channels
///// will rarely be at max at the same time. For 8 channel normal is 22.5ms.
#define PPM_PulseLen 400          //set the pulse length
#define PPM_Pause 3500            // Pause between PPM frames in microseconds (1ms = 1000µs) - Standard is 6500
#define PPM_FrLen (((1700+PPM_PulseLen) * PPM_CHANS)  + PPM_Pause)  //set the PPM frame length in microseconds 
#define PPM_offset -7             // How much are the channels offset  ? Compensate for timer difference, CPU spent elsewhere

//Definition of I/O pin
#define sigPin  2

#define onState 0  //set polarity: 1 is positive, 0 is negative
  
static uint16_t rcValIBus[PPM_CHANS];     //rc value reading from IBus stream
static uint16_t rcValPC[PPM_CHANS];       //rc value sending form PC
static uint16_t rcValTimer[PPM_CHANS];    //rc value for timer

static uint8_t ibusIndex = 0;
static uint8_t overrideIndex = 0;
static uint8_t ibus[IBUS_BUFFSIZE] = {0};
static uint8_t over[OVERRIDE_BUFFSIZE] = {0};
static uint8_t PrevTimeStamp = 0;
static uint8_t CurrentTimeStamp = 0;    //Time stamp of PC data package
static boolean IsOverride  = false;
static boolean IsPCReady   = false;    //Flag indicate new PC (override) data is ready
static boolean IsIbusReady = false;

void setupPPM();
void readIBus();
void readPC();

void setup()
{
  uint8_t i;
  //Setup serial for communicate with PC
  Serial.begin(115200);

  //Setup serial1 for communicate with radio receiver
  Serial1.begin(115200);
  for (i = 0; i<PPM_CHANS; i++)  rcValIBus[i] = 1000;

  IsOverride = false;
  setupPPM();
}

void loop() 
{
  //Read RC value from IBUS
  readIBus();
  readPC();

  //Read command
  if (IsIbusReady)
  {
    IsIbusReady = false;
    if (rcValTimer[7]>1500) IsOverride=0;
    
    cli();    //Disable interrupt while update RC value
    if (!IsOverride)  //When overriding command is still active, do not receive command from RC transmitter
    {
      rcValTimer[0] = rcValIBus[0];
      rcValTimer[1] = rcValIBus[1];
      rcValTimer[2] = rcValIBus[2];
      rcValTimer[3] = rcValIBus[3];
      rcValTimer[4] = rcValIBus[4];
    }
    //Except for channel 6-8
    rcValTimer[5] = rcValIBus[5];
    rcValTimer[6] = rcValIBus[6];
    rcValTimer[7] = rcValIBus[7];   
    sei();    //Enable interrupt
  }

  //Override RC value only channel 1-5, i.e. roll, pitch, throttle, yaw and flight mode
  if (IsPCReady && IsOverride)
  {
    IsPCReady = false;    
    cli();    //Disable interrupt while update RC value
    rcValTimer[0] = rcValPC[0];
    rcValTimer[1] = rcValPC[1];
    rcValTimer[2] = rcValPC[2];
    rcValTimer[3] = rcValPC[3];
    rcValTimer[4] = rcValPC[4];
    rcValTimer[5] = rcValIBus[5];
    rcValTimer[6] = rcValIBus[6];
    rcValTimer[7] = rcValIBus[7];  
    sei();    //Enable interrupt           
  }
  
}

void readPC()
{
  uint8_t i;  
  uint16_t chksum, rxsum;  
  uint8_t avail = Serial.available();
  
  if (avail)
  {
    uint8_t val = Serial.read();
    
    // Look for 0x2040 as start of packet
    if (overrideIndex == 0 && val != 0x20) 
    {
      return;
    }
    if (ibusIndex == 1 && val != 0x40) 
    {
      overrideIndex = 0;
      return;
    }     

    if (overrideIndex<OVERRIDE_BUFFSIZE) over[overrideIndex] = val;
    overrideIndex++;    

    //Unpack override data
    if (overrideIndex==OVERRIDE_BUFFSIZE)
    {
      overrideIndex = 0;
      chksum = 0xFFFF;
      for (i=0; i<OVERRIDE_BUFFSIZE-2; i++) chksum -= over[i];

      rxsum = over[OVERRIDE_BUFFSIZE-2] + (over[OVERRIDE_BUFFSIZE-1] << 8);

      //Do not copy value if the check sum is invalid
      if (chksum != rxsum)
      {
          return; 
      }

      //CurrentTimeStamp  = over[2];
      //if (PrevTimeStamp != CurrentTimeStamp)
      {
        PrevTimeStamp = CurrentTimeStamp;
        IsOverride = over[3];
        rcValPC[0] = (over[ 5] << 8) + over[ 4];
        rcValPC[1] = (over[ 7] << 8) + over[ 6];
        rcValPC[2] = (over[ 9] << 8) + over[ 8];
        rcValPC[3] = (over[11] << 8) + over[10];
        rcValPC[4] = (over[13] << 8) + over[12];
        IsPCReady = true;
      }
    }
  }
}


void readIBus()
{
  uint8_t i;
  uint16_t chksum, rxsum;

  uint8_t avail = Serial1.available();
  
  if (avail)
  {
    uint8_t val = Serial1.read();
    // Look for 0x2040 as start of packet
    if (ibusIndex == 0 && val != 0x20) 
    {
      return;
    }
    if (ibusIndex == 1 && val != 0x40) 
    {
      ibusIndex = 0;
      return;
    }
 
    if (ibusIndex<IBUS_BUFFSIZE) ibus[ibusIndex] = val;
    ibusIndex++;

    if (ibusIndex==IBUS_BUFFSIZE)
    {
      ibusIndex = 0;
      chksum = 0xFFFF;
      for (i=0; i<30; i++) chksum -= ibus[i];

      rxsum = ibus[30] + (ibus[31] << 8);

      //Do not copy value if the check sum is invalid
      if (chksum != rxsum)
      {
          return; 
      }
      
      //Unpack RC value from IBUS data
      rcValIBus[0] = (ibus[ 3] << 8) + ibus[ 2];
      rcValIBus[1] = (ibus[ 5] << 8) + ibus[ 4];
      rcValIBus[2] = (ibus[ 7] << 8) + ibus[ 6];
      rcValIBus[3] = (ibus[ 9] << 8) + ibus[ 8];
      rcValIBus[4] = (ibus[11] << 8) + ibus[10];
      rcValIBus[5] = (ibus[13] << 8) + ibus[12];
      rcValIBus[6] = (ibus[15] << 8) + ibus[14];
      rcValIBus[7] = (ibus[17] << 8) + ibus[16];

      IsIbusReady = true;
    }
  }
}

static byte cur_chan_numb;
void setupPPM()
{  

  pinMode(sigPin, OUTPUT);
  digitalWrite(sigPin, onState);  //set the PPM signal pin to the default state (off)
  cur_chan_numb = 0;

  rcValTimer[0] = 1500;
  rcValTimer[1] = 1500;
  rcValTimer[2] = 1000;
  rcValTimer[3] = 1500;
  rcValTimer[4] = 1000;
  rcValTimer[5] = 1000;
  rcValTimer[6] = 1000;
  rcValTimer[7] = 1000;
  
  cli();
  TCCR1A = 0; // set entire TCCR1 register to 0
  TCCR1B = 0;
  
  OCR1A = 100;  // compare match register, changed on the fly. First call in 100*0.5us.
  TCCR1B |= (1 << WGM12);  // turn on CTC mode
  TCCR1B |= (1 << CS11);  // 8 prescaler: 0,5 microseconds at 16mhz
  TIMSK1 |= (1 << OCIE1A); // enable timer compare interrupt
  sei();
}


// PPM sum is a signal consisting of a pulse of PPM_PulseLen, followed by a non-pulse 
// defining the signal value. So a 300us PPM-pulse and 1200us pause means the value 
// of the channel is 1500. You might get away with pulses down to 100us depending on
// receiving end.
// After the channels, a long channel is sent to fill out the frame.
// Normal frame length is 22500 us.
// 1 channel at full length is 2000us. 10 channels = 20000 us
// So even at 10 full channels, we have 2500us left for pause.
//
// Most Flight Controller trigger on level shift rather than timing.
// They measure from pulse start to pulse start, so PPM_PulseLen not critical
// If you want more channels, increase the framelen, to say 12*2000 + 3000 = 27000.

ISR(TIMER1_COMPA_vect) //leave this alone
{  
  static boolean state = true;

  cli();
  TCNT1 = 0;

  if(state) //start pulse
  {  
    digitalWrite(sigPin, onState);
    // Set timer that determines when interrupt is called again
    // This is the initial PPM_PulseLen marker
    OCR1A = PPM_PulseLen * 2;
    state = false;
  }
  else  //end pulse and calculate when to start the next pulse
  {  
    static unsigned int calc_signal; 

    // marker ended, so now pause for the channel value us
    digitalWrite(sigPin, !onState);
    // Make sure next interrupt causes a marker
    state = true;

    // Last channel, so set time to wait for full frame
    if(cur_chan_numb >= PPM_CHANS)
    {
      cur_chan_numb = 0;
      //calc_signal = calc_signal + PPM_PulseLen; //Compute time spent
      //OCR1A = (PPM_FrLen - calc_signal) * 2;    // Wait until complete frame has passed      
      OCR1A = PPM_Pause * 2;  // Wait for PPM_Pause
      calc_signal = 0;
    }
    else
    {                                    
      OCR1A = (rcValTimer[cur_chan_numb] - PPM_PulseLen) * 2 + (2*PPM_offset); // Set interrupt timer for the spacing = channel value                                                                     
      calc_signal +=  rcValTimer[cur_chan_numb];
      cur_chan_numb++;
    }     
  }
  sei();
}



