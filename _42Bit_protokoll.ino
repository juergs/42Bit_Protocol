#define DEBUG_1_PIN   11
#define DEBUG_2_PIN   10
#define DEBUG_3_PIN   9
#define DEBUG_4_PIN   8
#define DEBUG_5_PIN   7

// Arduino sketch to receive KW9010 temperature/humidity RF sensor telegrams
// Written by 'jurs' for German Arduino Forum
#include <bitArray.h> //byte-wide only!

#define RX433DATA 2       // receive pin for hardware interrupts
#define RX433INTERRUPT 0  // interrupt number for receive pin

#define KW9010_SYNC   8000    // length in µs of starting pulse
#define KW9010_ONE    4000     // length in µs of ONE pulse
#define KW9010_ZERO   2000    // length in µs of ZERO pulse
#define KW9010_GLITCH 200     // pulse length variation for ONE and ZERO pulses
#define KW9010_MESSAGELEN 42 //36  // number of bits in one message

#define FIFOSIZE 8  // Fifo Buffer size 8 can hold up to 7 items

volatile unsigned long fifoBuf[FIFOSIZE]; // ring buffer, war long 

volatile byte fifoReadIndex,fifoWriteIndex;  // read and write index into ring buffer

FILE      serial_stdout;          // needed for printf 

bitArray  proto1;
bitArray  proto2;
bitArray  proto3;
bitArray  proto4;
bitArray  proto5;
bitArray  proto6;

void printResults(unsigned long value);
boolean crcValid(unsigned long value, byte checksum);
void rx433Handler();
void rx433Handler2(); 
void fifoWrite(long item);
unsigned long fifoRead();
boolean fifoAvailable();
int freeRam ();
void printBits(size_t const size, void const * const ptr); 
int serial_putchar(char c, FILE* f);
void populateBitArray(byte pos, byte value);
byte readBitArray(byte pos);

void setup()
{
  //--- set up stdout for printf 
  fdev_setup_stream(&serial_stdout, serial_putchar, NULL, _FDEV_SETUP_WRITE);
  stdout = &serial_stdout;  
  
  Serial.begin(57600);
  pinMode(RX433DATA, INPUT);  
  attachInterrupt(RX433INTERRUPT, rx433Handler2, CHANGE);
  pinMode(12, OUTPUT);
  digitalWrite(12,LOW);   

  
  pinMode(DEBUG_1_PIN, OUTPUT);
  digitalWrite(DEBUG_1_PIN,LOW);   

  pinMode(DEBUG_2_PIN, OUTPUT);
  digitalWrite(DEBUG_2_PIN,LOW);   
    
  pinMode(DEBUG_3_PIN, OUTPUT);
  digitalWrite(DEBUG_3_PIN,LOW);   
  
  pinMode(DEBUG_4_PIN, OUTPUT);
  digitalWrite(DEBUG_4_PIN,LOW); 
  
  pinMode(DEBUG_5_PIN, OUTPUT);
  digitalWrite(DEBUG_5_PIN,LOW);   
  
  Serial.println();
  Serial.print(F("Free RAM: "));Serial.println(freeRam());
  Serial.println();
  Serial.println(F("Seconds\tCRC\tID\tChannel\tTemp C\tTrend\trH %\tLowBat\tForced send"));  
}

void loop()
{
/*
  if (fifoAvailable())
  {
 
    unsigned long dataReceived=fifoRead();
    Serial.print(millis()/1000);
    if (dataReceived!=0)
    {
      Serial.print(F("\tOK\t"));
      printBits(sizeof(dataReceived),&dataReceived);
      //Serial.println(""); 
      printResults(dataReceived);
    } 
    else 
      Serial.print(F("\tFAIL"));
    Serial.println(); 
  } 
*/
}

void rx433Handler2()
{
  static long           rx433LineUp, rx433LineDown;  
  static unsigned long  rxBits=0;
  static byte           crcBits=0;
  volatile static byte  bitsCounted=0;
  long                  LowVal, HighVal;
  unsigned long         dauer, timestamp; 

  boolean               isPulseForHigh = false;
  boolean               isPulseForLow = false;
  boolean               isPulseSync = false;
  boolean               isPulseUndef = false;
  
  byte rx433State = digitalRead(RX433DATA); // current pin state
  
  if (rx433State)  // pin is now HIGH -> fallende Flanke 
  {    
    rx433LineUp = micros(); // line went HIGH after being LOW at this time  
    LowVal = rx433LineUp - rx433LineDown; // calculate the LOW pulse time    
    //dauer = micros();
    isPulseSync = (LowVal > KW9010_SYNC - KW9010_GLITCH && LowVal < KW9010_SYNC + KW9010_GLITCH);
    isPulseForHigh = (LowVal > KW9010_ONE  - KW9010_GLITCH && LowVal < KW9010_ONE + KW9010_GLITCH); 
    isPulseForLow  = (LowVal > KW9010_ZERO - KW9010_GLITCH && LowVal < KW9010_ZERO + KW9010_GLITCH);
    isPulseUndef = !(isPulseForHigh || isPulseForLow || isPulseSync) ;
    //--- insgesamt 51uS fuer diese Berechnungen

    if (isPulseSync)
    {
      bitsCounted = 0;   
      digitalWrite(DEBUG_4_PIN,HIGH);       
    }                          
    else if (isPulseForHigh)
    {       
      //if (bitsCounted <= 42)
      populateBitArray(bitsCounted,HIGH);                          
      bitsCounted++;
      digitalWrite(DEBUG_1_PIN,HIGH);       
    }          
    else if (isPulseForLow)
    {       
      populateBitArray(bitsCounted,LOW);          
      bitsCounted++;
      digitalWrite(DEBUG_2_PIN,HIGH); 
    }        
    else if(isPulseUndef)  
    {      
      bitsCounted = 0;
      digitalWrite(DEBUG_5_PIN,HIGH);
    }
    
    if (bitsCounted >= KW9010_MESSAGELEN) // all bits received
    {
      digitalWrite(12,HIGH);       
      //Serial.print("BITS: " );            
      for (int i=0; i<42; i++)      
         Serial.print(readBitArray(i)); 
      Serial.println("");       
      bitsCounted = 0;
      digitalWrite(DEBUG_3_PIN,HIGH); 
    }
  }
  else
  { // High values have no information with them
    digitalWrite(12,LOW);
    rx433LineDown = micros();               // line went LOW after being HIGH
    HighVal = rx433LineDown - rx433LineUp; // calculate the HIGH pulse time
  }

  volatile unsigned long start_time = micros();
  
  while ( micros() - start_time < 10);   
  {
    // wait 10 uS 
    ; 
  } 
  
  digitalWrite(DEBUG_1_PIN,LOW); 
  digitalWrite(DEBUG_2_PIN,LOW); 
  digitalWrite(DEBUG_3_PIN,LOW); 
  digitalWrite(DEBUG_4_PIN,LOW); 
  digitalWrite(DEBUG_5_PIN,LOW);
}

void populateBitArray(byte pos, byte value)
{  
    if (pos <=7)
      proto1.writeBit(pos,value);       
    else if (pos > 7 && pos <= 15) 
      proto2.writeBit(pos,value);       
    else if (pos> 15 && pos <= 23)
      proto3.writeBit(pos,value);       
    else if (pos >23 && pos <= 31) 
      proto4.writeBit(pos,value);       
    else if (pos >31 && pos <= 39)
      proto5.writeBit(pos,value);       
    else 
      proto6.writeBit(pos,value);         
}

byte readBitArray(byte pos)
{
  byte val = LOW; 
  
    if (pos <=7)
      val = proto1.readBit(pos);       
    else if (pos > 7 && pos <= 15)
      val = proto2.readBit(pos);       
    else if (pos> 15 && pos <= 23)
      val = proto3.readBit(pos);       
    else if (pos >23 && pos <= 31)
      val = proto4.readBit(pos);       
    else if (pos >31 && pos <= 39)
      val = proto5.readBit(pos);       
    else
      val = proto6.readBit(pos);       
  return val ; 
}


///////////////////////////////////////////////////////////////////////////////////////
/// dont touch!
void rx433Handler()
{
  static long           rx433LineUp, rx433LineDown;
  static unsigned long  rxBits=0;
  static unsigned long  rxBits_2=0;
  static byte           crcBits=0;
  static byte           bitsCounted=0;
  long                  LowVal, HighVal;
  
  byte rx433State = digitalRead(RX433DATA); // current pin state
  if (rx433State) // pin is now HIGH
  {    
    rx433LineUp = micros(); // line went HIGH after being LOW at this time
  
    LowVal = rx433LineUp - rx433LineDown; // calculate the LOW pulse time
    
    if (LowVal>KW9010_SYNC-2*KW9010_GLITCH && LowVal<KW9010_SYNC+2*KW9010_GLITCH)
    {
      rxBits=0;
      rxBits_2 = 0;
      crcBits=0;
      bitsCounted=0;
    }
    else if (LowVal>KW9010_ONE-KW9010_GLITCH && LowVal<KW9010_ONE+KW9010_GLITCH)
    { 
      // set the one bits, war 32  = ulong size !
      if (bitsCounted<32)
      {
        bitSet(rxBits,bitsCounted);   
        printf("bitsCounted %d - rxBits: %d \n",bitsCounted,rxBits);      
      }
      else 
      {
        bitSet(crcBits, bitsCounted-32);
        printf("bitsCounted %d - crcBits: %d \n",bitsCounted,crcBits);      
      } 
      if (bitsCounted >= 32)
      {
        bitSet(rxBits_2,bitsCounted);              
        printf("bitsCounted %d - rxBits_2: %d \n",bitsCounted,rxBits_2);      
      }          
      bitsCounted++;
    }
    else if (LowVal>KW9010_ZERO-KW9010_GLITCH && LowVal<KW9010_ZERO+KW9010_GLITCH)
    { // setting zero bits is not necessary, but count them
      bitsCounted++;
    }
    else // received bit is not a SYNC, ONE or ZERO bit, so restart
    {
      rxBits=0;
      rxBits_2=0;
      crcBits=0;
      bitsCounted=0;
    }
    if (bitsCounted>=KW9010_MESSAGELEN) // all bits received
    {
      digitalWrite(12,HIGH); 
      printf("Bit counted: %d\n",bitsCounted); 

      fifoWrite(rxBits); // write valid value to FIFO buffer
/*      
      if (crcValid(rxBits,crcBits)) 
        fifoWrite(rxBits); // write valid value to FIFO buffer
      else 
        fifoWrite(0);  // write 0 to FIFO buffer (0 = invalid value received)
*/         
      rxBits=0;
      crcBits=0;
      bitsCounted=0;
      
 
    }
  }
  else
  { // High values have no information with them
    digitalWrite(12,LOW);
    rx433LineDown = micros(); // line went LOW after being HIGH
    HighVal = rx433LineDown - rx433LineUp; // calculate the HIGH pulse time
  }
}




void fifoWrite(long item)
// write item into ring buffer
{
  fifoBuf[fifoWriteIndex]=item; // store the item
  if (!(fifoWriteIndex+1==fifoReadIndex || (fifoWriteIndex+1>=FIFOSIZE && fifoReadIndex==0)))
    fifoWriteIndex++;  // advance write pointer in ringbuffer
  if (fifoWriteIndex>=FIFOSIZE) fifoWriteIndex=0; // ring buffer is at its end
}


unsigned long fifoRead()
// always check first if item is available with fifoAvailable()
// before reading the ring buffer using this function
{
  unsigned long item;
  item=fifoBuf[fifoReadIndex];
  cli(); // Interrupts off while changing the read pointer for the ringbuffer
  fifoReadIndex++;
  if (fifoReadIndex>=FIFOSIZE) fifoReadIndex=0;
  sei(); // Interrupts on again
  return(item);
} 

boolean fifoAvailable()
// item is available for reading if (fifoReadIndex!=fifoWriteIndex)
{
  return (fifoReadIndex!=fifoWriteIndex);
}

int freeRam () {
  extern int __heap_start, *__brkval;
  int v;
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval);
}

void printBits(size_t const size, void const * const ptr)
{
    unsigned char *b = (unsigned char*)ptr;
    unsigned char byte;
    int i, j;

    for (i = size - 1; i >= 0; i--)
    {
        for (j = 7; j >= 0; j--)
        {
            byte = (b[i] >> j) & 1;
            printf("%u", byte);
        }
    }
    puts("");
}

//--- function that printf and related will use to print
int serial_putchar(char c, FILE* f)
{
    if (c == '\n') serial_putchar('\r', f);
    return Serial.write(c) == 1 ? 0 : 1;
}
void printResults(unsigned long value)
{
  // Sensor ID
  byte id = value & 0b11001111; // bit 0, 1, 2, 3, 6, 7, random change bits when battery is changed
  Serial.print('\t');Serial.print(id);
  // Channel (as set on sensor)
  byte channel = 2*bitRead(value,4)+bitRead(value,5); // bit 4, 5 are channel number
  Serial.print('\t');Serial.print(channel);
  // Temperature
  int temperature = value>>12 & 0b111111111111;  // bit 12..23
  // if sign bit is set, adjust two's complement to fit a 16-bit number
  if (bitRead(temperature,11)) temperature= temperature | 0b1111000000000000;
  Serial.print('\t');Serial.print(temperature/10.0,1);
  // temperature tendency
  byte trend = value>>9 &0b11; // bit 9, 10
  Serial.print('\t');
  if (trend==0) Serial.print('=');       // temp tendency steady
  else if (trend==1) Serial.print('-');  // temp tendency falling
  else if (trend==2) Serial.print('+');  // temp tendency rising
  // Humidity
  byte humidity = (value>>24 & 0b11111111) - 156; // bit 24..31
  Serial.print('\t');Serial.print(humidity);
  // Battery State
  bool lowBat = value>>8 & 0b1;      // bit 8 is set if battery voltage is low
  Serial.print('\t');Serial.print(lowBat);
  // Trigger
  bool forcedSend = value>>11 &0b1;  // bit 11 is set if manual send button was pressed
  Serial.print('\t');Serial.print(forcedSend);
}

boolean crcValid(unsigned long value, byte checksum)
// check if received crc is correct for received value
{
  byte calculatedChecksum = 0;
  for (int i = 0 ; i < 8 ; i++) calculatedChecksum +=(byte)(value >> (i*4));
  calculatedChecksum &= 0xF;
  return calculatedChecksum == checksum;
}
