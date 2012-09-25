
/*
SolarPowerControllerNoVoltage.ino

Stuart Pittaway, Sept 2012.

Attribution-NonCommercial-ShareAlike 3.0 Unported (CC BY-NC-SA 3.0)
http://creativecommons.org/licenses/by-nc-sa/3.0/
You are free:
to Share — to copy, distribute and transmit the work
to Remix — to adapt the work
Under the following conditions:
Attribution — You must attribute the work in the manner specified by the author or licensor (but not in any way that suggests that they endorse you or your use of the work).
Noncommercial — You may not use this work for commercial purposes.
Share Alike — If you alter, transform, or build upon this work, you may distribute the resulting work only under the same or similar license to this one.
*/

/*

This is an attempt to test out a theory that a synthetic sine wave can be used for solar power diversion
in a similar fashion to Robin Emley's design (calypso_rae on Open Energy Monitor Forum July 2012).

Inspiration by pmcalli (Open Energy Monitor Forum Sept 2012).

The circuit is using a zero cross detector (trigger on positive volt crossing) on
the voltage waveform (connected to INT1, PIN 4 on CPU).

If this design works, solar PV diversion could be made without expensive (or accurate) transformers.

You can ignore the OPAMP GAIN sections from the code below if your not using any Opamp circuts.
*/

#define enable_serial_debug

/*
Joules per ADC-unit squared. Used for converting the product of voltage and current samples into Joules.
To determine this value, note the rate that the energy bucket's level increases when a known load is being measured at a convenient
test location (e.g using a mains extention with the outer cover removed so that
the current-clamp can fit around just one core. Adjust POWERCAL so that 'measured value' = 'expected value' for various loads. The value of
POWERCAL is not critical as any absolute error will cancel out when import and export flows are balanced.
*/
#define POWERCAL 0.025F

//Added in digitalWriteFast from http://code.google.com/p/digitalwritefast
#include "digitalWriteFast.h"

#include "TimerOne.h"


// define the input and output pins
//#define GAINSTAGEPIN1 9
//#define GAINSTAGEPIN2 A0
//#define GAINSTAGEPIN3 A5

#define outputPinForTrigger 4

//eMonTX uses A2 and A3 for voltage and CT1 sockets
//#define voltageSensorPin A2
#define currentSensorPin A3

// the external trigger device Triac is active low  SSR is active high
#define TRIAC_ON 1
#define TRIAC_OFF 0

#define LED_ON HIGH
#define LED_OFF LOW

// use float to ensure accurate maths, mains frequency (Hz)
#define cyclesPerSecond 50.0F
//#define safetyMargin_watts 0.0F // <<<------- Safety Margin in Watts (increase for more export)

//Wait 5 seconds before enabling TRIAC to allow readings to settle.
#define STARTUP_DELAY 5*cyclesPerSecond

//How many times the interrupt will fire during a full mains cycle (microseconds)
//In this case 100 samples per full AC wave, if you notice the serial output going really slow, reduce this number.
#define NUMBER_OF_SAMPLES_PER_FULLWAVE 99 //timer throws phantom interrupt when set so you only need 99

//Time in microseconds between each interrupt
//Deduction made to allow for processing at begining of cycle
#define INTERRUPTDELAY ((1000000/cyclesPerSecond)-300)/NUMBER_OF_SAMPLES_PER_FULLWAVE 
// 0.001 kWh = 3600 Joules
#define capacityOfEnergyBucket 3600

// defines for setting and clearing register bits
#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

static volatile int32_t SUPPLYVOLTAGE;
//static volatile int16_t lastSampleV;
static volatile int16_t lastSampleI;
static volatile double sumP,sumI;
static volatile double filteredI=511;

static volatile double lastFilteredI,pre;

static volatile double filteredV,lastFilteredV;
static volatile double divert_realPower,divert_realEnergy;
static volatile uint32_t waveformSampledCount = 0;
static volatile int16_t voltageAtZeroCross=0;
static volatile int16_t currentAtZeroCross=0;
static volatile int16_t sampleV,sampleI; // voltage & current samples are integers in the ADC's input range 0 - 1023

//Just for statistics
static volatile uint16_t samplesDuringThisMainsCycle = 0;

//For debug
static volatile uint16_t samplesDuringLastMainsCycle = 0;
static volatile double sumILastMainsCycle,sumPLastMainsCycle;


//Total number of samples taken over several AC wave forms
//static volatile uint16_t numberOfSamples=0;
static volatile bool beyondStartUpPhase=false;

static volatile unsigned long interrupt_timing=0;

// the 'energy bucket' mimics the operation of a digital supply meter at the grid connection point.
static volatile double divert_energyInBucket = 0;

static uint8_t page=0;
static uint8_t counter=8;

//static volatile double prevDCoffset; // <<--- for LPF to quantify the DC offset
static volatile double DCoffset; // <<--- for LPF
//static volatile double cumVdeltasThisCycle; // <<--- for LPF

//#define DCoffset 512

//uint16_t v[100];
int vreadingindexoffset=102; // 100 centre of voltage table units of 200us
int vreadingindex= vreadingindexoffset;


//Fake sinewave spread over 1023 scale/100 readings 3 cycles to allow phase shift  +/- one cycle
PROGMEM prog_uint16_t sinetab[] = {	
512,	544,	576,	608,	639,    670,	700,	730,	759,	786,
813,	838,	862,	885,    906,	926,	944,	960,	975,	988,
999,	1007,	1014,   1019,	1022,	1023,	1022,	1019,	1014,	1006,
997,	986,    973,	958,	942,	924,	904,	882,	859,	835,
809,    783,	755,	726,	696,	666,	635,	603,	572,	539,
507,	475,	443,	411,	380,	349,	319,	289,	261,    233,
207,	181,	157,	135,	114,	94,	77,	61,     46,	34,
23,	15,	8,	3,	1,	0,	1,      5,	10,	18,
27,	39,	52,	67,	84,	102,    122,	144,	167,	191,
217,	244,	272,	301,	331,    361,	392,	424,	456,	488,

512,	544,	576,	608,	639,    670,	700,	730,	759,	786,
813,	838,	862,	885,    906,	926,	944,	960,	975,	988,
999,	1007,	1014,   1019,	1022,	1023,	1022,	1019,	1014,	1006,
997,	986,    973,	958,	942,	924,	904,	882,	859,	835,
809,    783,	755,	726,	696,	666,	635,	603,	572,	539,
507,	475,	443,	411,	380,	349,	319,	289,	261,    233,
207,	181,	157,	135,	114,	94,	77,	61,     46,	34,
23,	15,	8,	3,	1,	0,	1,      5,	10,	18,
27,	39,	52,	67,	84,	102,    122,	144,	167,	191,
217,	244,	272,	301,	331,    361,	392,	424,	456,	488,   

512,	544,	576,	608,	639,    670,	700,	730,	759,	786,
813,	838,	862,	885,    906,	926,	944,	960,	975,	988,
999,	1007,	1014,   1019,	1022,	1023,	1022,	1019,	1014,	1006,
997,	986,    973,	958,	942,	924,	904,	882,	859,	835,
809,    783,	755,	726,	696,	666,	635,	603,	572,	539,
507,	475,	443,	411,	380,	349,	319,	289,	261,    233,
207,	181,	157,	135,	114,	94,	77,	61,     46,	34,
23,	15,	8,	3,	1,	0,	1,      5,	10,	18,
27,	39,	52,	67,	84,	102,    122,	144,	167,	191,
217,	244,	272,	301,	331,    361,	392,	424,	456,	488,   

};

/*static long readVcc() {
  long result;
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  delay(2);
  ADCSRA |= _BV(ADSC);
  while (bit_is_set(ADCSRA,ADSC));
  result = ADCL;
  result |= ADCH<<8;
  result = 1126400L / result;
  return result;
}
*/
 // static volatile double filteredV;
  static volatile double instP;
//


// interrupt routines


//measurement of current at sample point
static void takesinglereading() {
  //This function takes about 80 microseconds to run.
  digitalWriteFast(7,HIGH); //set digital 6 high to indicate reading started
  interrupt_timing=micros();

  //lastSampleI=sampleI; // save previous voltage values for digital high-pass filter
  sampleI = analogRead(currentSensorPin); //Read in raw current signal
 // lastFilteredI = filteredI;
 // filteredI = 0.996*(lastFilteredI+(sampleI-lastSampleI));

    sumI +=sampleI;
 
  sampleV=pgm_read_word_near(sinetab+vreadingindex);
  vreadingindex++;

  filteredV = DCoffset-sampleV;
  instP = filteredV * (sampleI-filteredI); // power contribution for this pair of V&I samples
  sumP +=instP; // cumulative power values for this mains cycle

  samplesDuringThisMainsCycle++; // for power calculation of a single AC wave
  //if (samplesDuringThisMainsCycle=100) Timer1.detachInterrupt();
  interrupt_timing=micros()-interrupt_timing;
  digitalWriteFast(7,LOW); //set digital 6 low to indicate reading finished
}




static void positivezerocrossing()
{
  //This is the start of a new mains cycle just before going zero cross point. Timing of this
  //depends upon the opamp comparitor circut.
  //To keep the timing constant as possible try to avoid putting anything in large if...then clauses
  //so that the path through the code is a similar as possible...
  Timer1.detachInterrupt();

  //Start of the solar power divert code...
  waveformSampledCount++;

  //prevDCoffset = DCoffset;
  //DCoffset = prevDCoffset + (0.01 * cumVdeltasThisCycle);

  // Calculate the real power of all instantaneous measurements taken during the
  // previous mains cycle, and determine the gain (or loss) in energy.
  divert_realPower = POWERCAL * sumP / (float)samplesDuringThisMainsCycle;
  divert_realEnergy = divert_realPower / cyclesPerSecond;
  
  //lastFilteredI = filteredI;
  filteredI=sumI/samplesDuringThisMainsCycle;
  
  //filteredI = lastFilteredI + (0.01 * ((sumI/samplesDuringThisMainsCycle)-lastFilteredI));// low pass filter to get current offset
  //filteredI = lastFilteredI + (0.01 * (filteredI-lastFilteredI));
  
  
  //Warm up for 5 seconds before enabling TRIAC
  if (waveformSampledCount>STARTUP_DELAY) beyondStartUpPhase = true;

  if (beyondStartUpPhase == true)
  {
    // Reduce the level in the energy bucket by the specified safety margin.
    // This allows the system to be positively biassed towards export or import
    divert_energyInBucket += divert_realEnergy; //-(safetyMargin_watts / cyclesPerSecond);
   
  }

  // Apply max and min limits to bucket's level
  if (divert_energyInBucket > capacityOfEnergyBucket)
    divert_energyInBucket = capacityOfEnergyBucket;

  if (divert_energyInBucket <= 0)
    divert_energyInBucket = 0;


  if (divert_energyInBucket > (capacityOfEnergyBucket / 2))
  {
    digitalWriteFast(outputPinForTrigger, TRIAC_ON);
   // digitalWriteFast(outputPinForLed, LED_ON);
  }
  else
  {
    digitalWriteFast(outputPinForTrigger, TRIAC_OFF);
    //digitalWriteFast(outputPinForLed, LED_OFF);
  }
  
  //stored value for debug mode (debug breaks into cycle values of power energy bucket etc are last cycle but samples sumI and sumP are not)
 
  samplesDuringLastMainsCycle=samplesDuringThisMainsCycle;
  sumILastMainsCycle=sumI;
  sumPLastMainsCycle=sumP;
 
  
  sumI = 0;
  sumP = 0;
  samplesDuringThisMainsCycle = 0;
  vreadingindex= vreadingindexoffset;
  //cumVdeltasThisCycle = 0;

  //currentAtZeroCross=analogRead(currentSensorPin);
  //voltageAtZeroCross=analogRead(voltageSensorPin);
  //Get the first reading near the zero cross...
 // takesinglereading();


 
  // attaches callback() as a timer overflow interrupt, called every X microseconds
  Timer1.start();
  Timer1.attachInterrupt(takesinglereading,INTERRUPTDELAY );
}



void setup()
{
#ifdef enable_serial_debug
  Serial.begin(115200); //Faster baud rate to reduce timing of serial.print commands
  Serial.println(F("SolarPowerController - No Voltage Samples"));
  Serial.print(F("Time between interrupts (microseconds)="));
  Serial.println(INTERRUPTDELAY);
#endif

  //These are only needed if you are using a variable gain opamp circuit
  //default to the highest amplification
 /* pinModeFast(GAINSTAGEPIN1, OUTPUT);
  pinModeFast(GAINSTAGEPIN2, OUTPUT);
  pinModeFast(GAINSTAGEPIN3, OUTPUT);

  digitalWriteFast(GAINSTAGEPIN1,HIGH);
  digitalWriteFast(GAINSTAGEPIN2,HIGH);
  digitalWriteFast(GAINSTAGEPIN3,HIGH);
*/
  pinModeFast(outputPinForTrigger, OUTPUT);
  //pinModeFast(outputPinForLed, OUTPUT);

 // analogReference(DEFAULT); //5v

 // SUPPLYVOLTAGE = readVcc();


Serial.println("Sine table in use=");
  for(int i=0;i<100;i++) {
    if (i % 20==0) Serial.println("");
    Serial.print(pgm_read_word_near(sinetab+i));
    Serial.print(", ");
  }
  Serial.println("");


  Timer1.initialize();

  //FASTER ANALOGUE READ ON ARDUINO
  //set prescale as per forum on http://www.arduino.cc/cgi-bin/yabb2/YaBB.pl?num=1208715493/11
  //table of prescale values... http://i187.photobucket.com/albums/x269/jmknapp/adc_prescale.png
  //Prescale 32
  sbi(ADCSRA,ADPS2); //ON
  cbi(ADCSRA,ADPS1); //OFF
  sbi(ADCSRA,ADPS0); //ON

  //Start the interrupt and wait for the first zero crossing...
  attachInterrupt(1, positivezerocrossing, RISING);
}

void loop()
{
  //Do what we want here, no time critical code to worry about
  //the interrupts do all the hard work!

#ifdef enable_serial_debug
  Serial.print(" cyc# ");
  Serial.print(waveformSampledCount);

  //This is a timing of the interrupt routine in microseconds
  //Serial.print(",iT ");
  //Serial.print(interrupt_timing);
  //Serial.print(", Vzero ");
  //Serial.print(voltageAtZeroCross);
 // Serial.print(", Izero ");
 // Serial.print(currentAtZeroCross);

  //Serial.print(",SupVol ");
  //Serial.print(SUPPLYVOLTAGE);

 // Serial.print(",lastSampleI ");
 // Serial.print(lastSampleI);
 Serial.print(",filteredI ");
 Serial.print(filteredI);
 Serial.print(",samplesDuringThisMainsCycle");
  Serial.print(samplesDuringThisMainsCycle);
  Serial.print(",samplesDuringLastMainsCycle ");
  Serial.print(samplesDuringLastMainsCycle);
  
  
  //Serial.print(",sumI ");
  //Serial.print(sumI);
  
//  Serial.print(",sumI=");
 // Serial.print(sumILastMainsCycle);
 // Serial.print(",sumP=");
 // Serial.print(sumPLastMainsCycle);
 
  Serial.print(",StartUpPhase=");
  Serial.print(beyondStartUpPhase);
  
  //Serial.print(",sampleI ");
  //Serial.print(sampleI);

  Serial.print(",realPower=");
  Serial.print(divert_realPower);
  Serial.print("W,enInBkt=");
  Serial.print(divert_energyInBucket);
  Serial.print(",realEnergy=");
  Serial.println(divert_realEnergy);
#endif

  delay(500);

} // end of loop()



