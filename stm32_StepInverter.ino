
#include <Streaming.h>
#include <PString.h>
#include <STM32ADC.h>

STM32ADC adcV(ADC1);
STM32ADC adcI(ADC2);

uint8 adcPinV = PB0;
uint8 adcPinI = PB1;

byte  outputDrive = 0;    // Output Driver : High Byte of PORT B 
uint16 cycleIndex;           // position within the cycle : between 0 to 1023 , 
uint8  lastVoltageLookupIndex;
uint16  batteryVoltage = 26500;
int32 inputFactor = 1000;
uint32 nominalBatteryMicroVolts;
uint8 voltSteps;
uint16 quarterSteps;

volatile uint16 adcValueV;
volatile uint16 adcValueI;

HardwareTimer inverterTimer(4);

#define NOMINAL_VAC  240
#define NOMINAL_BATTERY_VOLTAGE 26500    // in milliVolts
#define TRANSFORMER_COUNT 4
#define STEPS 1024
#define VOLTSTEPS   int(pow(3,TRANSFORMER_COUNT))

uint16 Vtransformer[TRANSFORMER_COUNT];   // in Volts * 10 ie 2350 = 235.0 volts

int16 sineLookup[STEPS];   
int16 outputVoltageLookup[VOLTSTEPS];   // in Volts * 10 ie 2350 = 235.0 volts
int8 transformerMask[VOLTSTEPS][TRANSFORMER_COUNT];
uint8 outputMaskTable[VOLTSTEPS];

char message[50];
PString pMessage(message,sizeof(message));

#define LED      PC13

void setup() 
{
  Serial.begin(250000);
  pinMode(PA4,OUTPUT);
  pinMode(PC14,OUTPUT);
  pinMode(PC15,OUTPUT);
  pinMode(LED,OUTPUT);
  //slow way
  pinMode(PB8,OUTPUT);
  pinMode(PB9,OUTPUT);
  pinMode(PB10,OUTPUT);
  pinMode(PB11,OUTPUT);
  pinMode(PB12,OUTPUT);
  pinMode(PB13,OUTPUT);
  pinMode(PB14,OUTPUT);
  pinMode(PB15,OUTPUT);
  digitalWrite(LED,LOW);
  delay(1000);
  Serial.flush();
  // quick way, the upper 8 bits all set to output, pushpull
  GPIOB->regs->CRH = 0x33333333;
  GPIOB->regs->ODR = 0;

  
  voltSteps = int(pow(3,TRANSFORMER_COUNT));//voltSteps;
  quarterSteps = STEPS/4;
  nominalBatteryMicroVolts = NOMINAL_BATTERY_VOLTAGE * 1000;
  setTransformerVoltage();
  fill_TransformerMask();
  fill_outputDriveTable();
  fill_outputVoltageTable();
  fill_sineLookup();

//  cycleIndex = 79;
//  proc_nextOutputValue();

//*******  ADC setup

//  adcV.calibrate();
//  adcV.setSampleRate(ADC_SMPR_239_5);
//  adcV.attachInterrupt(adcConvertedInt, ADC_EOC);
//  adcV.setPins(&adcPinV, 1);
//
//  adcI.calibrate();
//  adcI.setSampleRate(ADC_SMPR_239_5);
//  //adcI.attachInterrupt(int_ADCI,, ADC2_EOC);
//  adcI.setPins(&adcPinI, 1);
//
//    // -- sets up the adc control registers
//    //    for Dual ADC, simultaneous sample
//  ADC1->regs->CR1 |= 0x60000;
//  ADC1->regs->CR2 |= 0x100;
  
  //*******  Timer setup  ( 19.52777777777777777778 microseconds, 51209.10384068278805121 Hz)
  //---  output Frquency = 50.00889046941678520626 Hz
  //---  inverterTimer frequency = 51200Hz (50Hz x 1024 steps)
  //---  inverterTimer period = 19.53125 uSec
  //---  overflow = 72MHz / 51200Hz = 1406.25
  
  inverterTimer.setMode(TIMER_CH4, TIMER_OUTPUT_COMPARE);
  inverterTimer.pause();
  inverterTimer.setPrescaleFactor(1);
  inverterTimer.setCount(0);
  inverterTimer.setOverflow(1406);//1406
  inverterTimer.setCompare(TIMER_CH4, 1);  
  inverterTimer.attachCompare4Interrupt(timer_Handler);
  inverterTimer.refresh();
  inverterTimer.resume();
  delay(1000);
  Serial << "Setup done" << endl;
  Serial.flush();
  digitalWrite(LED,HIGH);
}

bool junk;

void timer_Handler()
{
  gpio_write_bit(GPIOA, 4, HIGH);
  if (junk)
  {
    gpio_write_bit(GPIOC, 14, LOW);
    junk=false;
  }
  else
  {
    gpio_write_bit(GPIOC, 14, HIGH);
    junk=true;
  }
  
  uint16 mask=0;
  static int8 cnt=0;
  byte portBlow = GPIOB->regs->ODR & 0x00ff;
  mask = outputDrive<<8 | portBlow;
  //--- write PORTB 
  GPIOB->regs->ODR = mask;

  cnt++;
  if (cnt>100)
  {
    cnt=0;
    //adcV.startConversion();
  }
  //--- now work out the next driver value
  
  proc_nextOutputValue();
  
}

void adcConvertedInt()
{
  static uint16 lastADC = 0;
  //-- read adc 32bit data register
  uint32 data = ADC1->regs->DR;
  adcValueV = data & 0xFFFF;
  adcValueI = data >> 16;
}

void proc_nextOutputValue()
{
  cycleIndex ++;
  if (cycleIndex >= STEPS) 
  {
    cycleIndex = 0;
    //proc_NewCycle();
    outputDrive = outputMaskTable[0];
    gpio_write_bit(GPIOA, 4, LOW);
    return;
  }
  
  int32 desiredVolts = sineLookup[cycleIndex];
  findMask(cycleIndex,desiredVolts);
  
//  Bin2Char(message,outputDrive);
//  Serial << "# " << cycleIndex << " v= " << desiredVolts << 
//          " lastIdx= " << lastVoltageLookupIndex << 
//          " Drv= " << _BIN(outputDrive) << "  bin2char= " << message << endl;
  gpio_write_bit(GPIOA, 4, LOW);
}

void proc_NewCycle()
{
  //inputFactor = (int32) ( batteryVoltage * 100u) ;
  //inputFactor = inputFactor / NOMINAL_BATTERY_VOLTAGE;
  inputFactor = nominalBatteryMicroVolts / batteryVoltage;
}


void findMask(uint currentStep,int32 matchVolts)
{
  static int32 lastVolts=0;
  int16 index;
  
  if (matchVolts == lastVolts) return;
  int32 diff = matchVolts - lastVolts;
  if (diff>0)
  {
    index = findMaskUp(lastVoltageLookupIndex,matchVolts);
  }
  else
  {
    index = findMaskDn(lastVoltageLookupIndex,matchVolts);
  }
  lastVolts = matchVolts;
  if (lastVoltageLookupIndex != index && index >-1)
  {
    lastVoltageLookupIndex = index;
    outputDrive = outputMaskTable[index];
  }
}




int16 findMaskUp(uint8 lookupindex, int32 matchVolts)
{
  if (lookupindex < 1) lookupindex = 1;
  for (int i=lookupindex;i<voltSteps;i++)
  {
    if (matchVolts == outputVoltageLookup[i])
    {
      return (i);
    }
    if (matchVolts < outputVoltageLookup[i])
    {
      if (matchVolts > outputVoltageLookup[i-1])
      {
        return (i);
      }
    }
  }
  return -1;
}

int16 findMaskDn(uint8 lookupindex, int32 matchVolts)
{
  for (int i=lookupindex;i>0;i--)
  {
    if (matchVolts > outputVoltageLookup[voltSteps-1])return (voltSteps-1);
    if (matchVolts == outputVoltageLookup[i])
    {
      return (i);
    }
    if (matchVolts < outputVoltageLookup[i])
    {
      if (matchVolts > outputVoltageLookup[i-1])
      {
        return (i);
      }
    }
  }
  return -1;
}

void setTransformerVoltage()
{
  
  float Vpeak =  NOMINAL_VAC * sqrt(2) * 10;
  int B=0;
  for (int i=0;i<TRANSFORMER_COUNT;i++)
  {
    B += int(pow(3,i));
  }
  for (int i=0;i<TRANSFORMER_COUNT;i++)
  {
    int x = TRANSFORMER_COUNT - i - 1;
    float F =  pow(3,x) / B ;
    Vtransformer[x] = (uint16) (Vpeak * F);
  }

  for (int i=0;i<TRANSFORMER_COUNT;i++)
  {
    delay(100);
    Serial << "Tx " << i << " : " << Vtransformer[i] << endl;
    Serial.flush();
    delay(50);
  }
}

/*--------------------------------------------------------------------------------
 * 
 *  Fills  array transformeMask with an incremented ternary (base 3) mask
 *  
 *  in relation to the transformer  0 = -1, 1 = 0, 2 = +1 
 * 
 --------------------------------------------------------------------------------*/

void fill_TransformerMask()
{
  for (int i=0;i<TRANSFORMER_COUNT;i++)
  {
    transformerMask[0][i] = 0;
  }
  for (int i=1;i<voltSteps;i++)
  {
    for (int j=0;j<TRANSFORMER_COUNT;j++)
    {
      if (j==0)
      {
        transformerMask[i][j] = 0;
        if (transformerMask[i-1][j] < 2)  transformerMask[i][j] = transformerMask[i-1][j] +1;
      }
      if (j>0) transformerMask[i][j] = transformerMask[i-1][j];
      if (j>0 && transformerMask[i-1][j-1] > transformerMask[i][j-1])
      {
        transformerMask[i][j] = 0;
        if (transformerMask[i-1][j] < 2) transformerMask[i][j] = transformerMask[i-1][j] +1;
      }
    }
  }
}

void fill_outputDriveTable()
{
  for (int i=0;i<voltSteps;i++)
  {
    outputMaskTable[i] = 0 ;
    for (int j=0;j<TRANSFORMER_COUNT;j++)
    {
      int mult =int(pow(4,j));
      int state = (int) transformerMask[i][j]-1;
      if (state < 0 )state = 2;
      outputMaskTable[i] += state * mult;
    }
  }
}

void fill_outputVoltageTable()
{
  for (int i=0;i<voltSteps;i++)
  {
    int32 sumVolts = 0;
    for (int j=0;j<TRANSFORMER_COUNT;j++)
    {
      int8 p = (int8) transformerMask[i][j]-1;
      int32 v = (int32) Vtransformer[j];
      sumVolts += p * v;
    }
    outputVoltageLookup[i] = sumVolts;
  }
}




void fill_sineLookup()
{
  double Vpeak =  NOMINAL_VAC * sqrt(2) ;
  for (int i=0;i< STEPS; i++)
  {
    double rad = 2 * PI / STEPS * i;
    double volts = sin(rad) * Vpeak;
    sineLookup[i] = int(volts * 10 + 0.5);
  }
}

uint16 MAKE_WORD( const byte Byte_hi, const byte Byte_lo)
{
     return   (( Byte_hi << 8  ) | Byte_lo & 0x00FF );
}


void Bin2Char(char *buff, uint8 value)
{
    buff[8] = 0;
    int i=0;
    for (uint8 mask = 0x80; mask; mask >>= 1)
    {
        buff[i] = ((mask & value) ? '1' : '0');
        i++;
        buff[i] = 0;
    }
    
}

void loop() {
  // put your main code here, to run repeatedly:

}
