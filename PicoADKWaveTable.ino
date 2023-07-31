/*Need to rewrite with non-downsampled wavetables
* PicoWaveTable
 *  Jim MacArthur, Harvard University, 2021-08-26.
 *  Mashes M0ChirpInterp with M0SamplePlayer
 *  For extra fun, put pots on the A1 and A2 lines and uncomment the analogRead instructions at the end of Loop.
 * 
    bool setBCLK(pin_size_t pin);
    - This assigns two adjacent pins - the pin after this one (one greater)
      is the WS (word select) signal, which toggles before the sample for
      each channel is sent

    bool setDATA(pin_size_t pin);
    - Sets the DOUT pin, can be any valid GPIO pin
*/

#include <I2S.h>

// Create the I2S port using a PIO state machine
I2S i2s(OUTPUT);


// GPIO pin numbers
#define pBCLK 17
#define pWS (pBCLK+1)
#define pDOUT 16 //for PicoADK
#define DEMP 23  //DAC de-emphasis signal
#define NMUTE 25  //DAC !mute signal


#define FLAGOUT1 21 //output flag for diagnostic timing
#define FLAGOUT1_MASK 0x00200000
#define FLAGOUT2 22 //another output flag
#define FLAGOUT2_MASK 0x00400000

#define LED1 2
#define LED2 3
#define LED3 4
#define LED4 5

#define POT1 A0  //first potentiometer
#define POT2 A1  //second potentiometer



// max volume for 32 bit data. Used to generate wave table
#define VOLUME 65535

const int sampleRate = 44100;
int16_t LSample,RSample;

#include <math.h>

#define COMPRESSED true

extern const unsigned int WaveTable[32768];

#include "AudioSampleMadness.cpp"  //wavetable with 64 2048 -sample tables.  each uint has 4 samples
//#include "AudioSampleGenEd1080.cpp"  //wavetable with 64 2048 -sample tables.  each uint has 4 samples
//#include "AudioSampleAcousticGuitar.cpp"  //uncompressed wavetable
//#include "AudioSamplePing16.cpp" //uncompressed

#include "data_ulaw.c"

// Wavetable generated from Vital, then downsampled to 22K in Audacity.
// Then run PJRC's Wav2Sketch from a command prompt using default u-law coding.
// Then edit the resulting .cpp file as described in AudioSampleMadness.cpp

int Wave1[2048];  //this stores a single uncompressed cycle of a wave from WaveTable.
//when dealing with 22K undersampled data, we only use 1024 samples



uint32_t tmp32;

uint32_t WavePtr;
uint32_t WaveTableOffset;
//int S0, S1, S2, S3, S4, Samp;  //temp sample storage
int S1;
int S11, S12, S13, S14;
int S21, S22, S23, S24;
uint32_t InterpFrom, InterpTo;
int16_t Accumulator16;

uint32_t Speed;

double PitchF = 2000000.00;  
int32_t PitchInt = 0x01111111;  //this is PitchF cast into an int

//PitchF gets added to WavePtr every sample,
//so if PitchF = 2^32 each sample would be an entire wave cycle, or 44100 Hz.
//Therefore, PitchF = (frequency in Hz * 2^32) / 44100
//So for A440, PitchF = (440 * 2^32) / 44100 = 42852281.41


uint16_t Pot1Val = 0;  //Pot 1 reading, normalized to 15 bits
uint16_t Pot1Avg = 0;  //Pot1 reading, low-pass filtered
uint16_t Pot2Val = 0;  //Pot2 reading, normalized to 15 bits
uint16_t Pot2Avg = 0;  //Pot2 reading, low-pass filtered


void setup() {

//initialize diagnostic outputs
  pinMode(FLAGOUT1, OUTPUT);
  pinMode(FLAGOUT2, OUTPUT);
  pinMode(LED1,OUTPUT);
  digitalWrite(LED1,LOW);



  pinMode(DEMP,OUTPUT);
  digitalWrite(DEMP,LOW); //turn off D/A de-emphasis

  i2s.setBCLK(pBCLK);
  i2s.setDATA(pDOUT);
  i2s.setBitsPerSample(16);

  i2s.setBuffers(4, 1024);


  // start I2S at the sample rate with 16-bits per sample
  if (!i2s.begin(sampleRate)) 
  {
    while(1)     //i2s didn't initialize properly so stay here flashing the LED
    {
      digitalWrite(LED1,HIGH);  //flash an error indicator
      delay(300);
      digitalWrite(LED1,LOW);
      delay(300);
    }  
  }
 
  WaveTableOffset = 0;
  PitchF = 3000000.00; 
  PitchInt = uint32_t(PitchF);
  Speed = 14000000;

  pinMode(NMUTE, OUTPUT);   
  digitalWrite(NMUTE, 1);    //unmute the D/A converter
  digitalWrite(LED1,HIGH);  //Turn LED on to indicate that setup completed without error



}



void setup1()
{
  //nothing to set up for second processor
}

void loop() 
{

         InterpTo = (WaveTableOffset>>10)&0x0000FFFF;          
        InterpFrom = 65535 - InterpTo;  //65535 or 6?


// compressed 

//wait for waveptr to clear the first half of the wave1 buffer (0 to 512)

  if((WavePtr>>22) >= 512 )
  {
      for(int i=0; i<128; i++)   //512 for non-undersampled
      {
        tmp32 = WaveTable[((WaveTableOffset>>17)&0x00007E00) +i];
        S11 = int(ulaw_decode_table[(tmp32 >> 0) & 255]) << 16; 
        S12 = int(ulaw_decode_table[(tmp32 >> 8) & 255]) << 16;
        S13 = int(ulaw_decode_table[(tmp32 >> 16) & 255]) << 16;
        S14 = int(ulaw_decode_table[(tmp32 >> 24) & 255]) << 16; 

        tmp32 = WaveTable[(((WaveTableOffset>>17)+0x00000100)&0x00007E00) +i];  //+200 for non-undersampled
        S21 = int(ulaw_decode_table[(tmp32 >> 0) & 255]) << 16; 
        S22 = int(ulaw_decode_table[(tmp32 >> 8) & 255]) << 16;
        S23 = int(ulaw_decode_table[(tmp32 >> 16) & 255]) << 16;
        S24 = int(ulaw_decode_table[(tmp32 >> 24) & 255]) << 16; 

        Wave1[i<<2] = ((S11>>16)*InterpFrom) + ((S21>>16)*InterpTo); 
        Wave1[(i<<2)+1] = ((S12>>16)*InterpFrom) + ((S22>>16)*InterpTo); 
        Wave1[(i<<2)+2] = ((S13>>16)*InterpFrom) + ((S23>>16)*InterpTo); 
        Wave1[(i<<2)+3] = ((S14>>16)*InterpFrom) + ((S24>>16)*InterpTo);                                      
      }
  }
  else
  {
      for(int i=128; i<256; i++)   //512 for non-undersampled
      {
        tmp32 = WaveTable[((WaveTableOffset>>17)&0x00007E00) +i];
        S11 = int(ulaw_decode_table[(tmp32 >> 0) & 255]) << 16; 
        S12 = int(ulaw_decode_table[(tmp32 >> 8) & 255]) << 16;
        S13 = int(ulaw_decode_table[(tmp32 >> 16) & 255]) << 16;
        S14 = int(ulaw_decode_table[(tmp32 >> 24) & 255]) << 16; 

        tmp32 = WaveTable[(((WaveTableOffset>>17)+0x00000100)&0x00007E00) +i];  //+200 for non-undersampled
        S21 = int(ulaw_decode_table[(tmp32 >> 0) & 255]) << 16; 
        S22 = int(ulaw_decode_table[(tmp32 >> 8) & 255]) << 16;
        S23 = int(ulaw_decode_table[(tmp32 >> 16) & 255]) << 16;
        S24 = int(ulaw_decode_table[(tmp32 >> 24) & 255]) << 16; 

        Wave1[i<<2] = ((S11>>16)*InterpFrom) + ((S21>>16)*InterpTo); 
        Wave1[(i<<2)+1] = ((S12>>16)*InterpFrom) + ((S22>>16)*InterpTo); 
        Wave1[(i<<2)+2] = ((S13>>16)*InterpFrom) + ((S23>>16)*InterpTo); 
        Wave1[(i<<2)+3] = ((S14>>16)*InterpFrom) + ((S24>>16)*InterpTo);                                      
      }

  }


/*
//uncompressed


//wait for waveptr to clear the first half of the wave1 buffer (0 to 512)

  if((WavePtr>>22) >= 512 )
  {

      for(int i=0; i<256; i++)   //512 for no ulaw compression
      {

        tmp32 = WaveTable[((WaveTableOffset>>17)&0x00007E00) +i];
        S11 = (tmp32 & 0xFFFF) << 16; 
        S12 = tmp32;

        tmp32 = WaveTable[(((WaveTableOffset>>17)+0x00000200)&0x00007E00) +i];  //+200 for non-undersampled
        S21 = (tmp32 & 0xFFFF) << 16; 
        S22 = tmp32;

        Wave1[i<<1] = ((S11>>16)*InterpFrom) + ((S21>>16)*InterpTo); 
        Wave1[(i<<1)+1] = ((S12>>16)*InterpFrom) + ((S22>>16)*InterpTo); 
      }
  }
  else
  {
      for(int i=256; i<512; i++)   //512 for no ulaw compression
      {

        tmp32 = WaveTable[((WaveTableOffset>>17)&0x00007E00) +i];
        S11 = (tmp32 & 0xFFFF) << 16; 
        S12 = tmp32;

        tmp32 = WaveTable[(((WaveTableOffset>>17)+0x00000200)&0x00007E00) +i];  //+200 for non-undersampled
        S21 = (tmp32 & 0xFFFF) << 16; 
        S22 = tmp32;

        Wave1[i<<1] = ((S11>>16)*InterpFrom) + ((S21>>16)*InterpTo); 
        Wave1[(i<<1)+1] = ((S12>>16)*InterpFrom) + ((S22>>16)*InterpTo); 
      }
  }
*/

    Pot1Val = analogRead(POT1) << 5;  //normalize 10-bit ADC to 0-32767 range
    Pot2Val = analogRead(POT2) << 5;  //normalize 10-bit ADC to 0-32767 range

//smooth pots with low-pass filters
//This is a simple-to-calculate 1-pole IIR filter, made by summing the existing filtered signal with the input.
//You change the responsiveness by changing the shift value from 4.  Increasing slows down the filter.
    Pot1Avg = Pot1Avg - (Pot1Avg >> 4) + (Pot1Val >> 4);
    Pot2Avg = Pot2Avg - (Pot2Avg >> 4) + (Pot2Val >> 4);


  // PitchInt = Pot1Avg <<8;
   // Speed = ((Pot2Avg - 16384) <<11);


}

void loop1() 
{


    WaveTableOffset += Speed;

    for(int i=0; i<256; i++)
    {
      S1 = (Wave1[WavePtr  >> 22] >> 16) * (65535 - ((WavePtr & 0x003FFFC0) >>6))  + (Wave1[(WavePtr+0x00400000)>>22] >> 16) * ((WavePtr & 0x003FFFC0) >> 6);  //for 1024-point wavetables
 //should it be 65535 or 65536?
       Accumulator16 = S1>>16;
      i2s.write16(Accumulator16,Accumulator16); 

      WavePtr += PitchInt; 
    }           
}




