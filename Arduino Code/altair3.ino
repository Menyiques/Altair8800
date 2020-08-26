
extern "C" {
  #include "i8080.h"
}
#include <LedControl.h>
int DIN = 11;
int CS =  10;
int CLK = 13;
//byte RAM [128] = {58,128,0,71,58,129,0,128,50,130,195,0,0};
byte RAM [1024] = {33,0,0,22,128,1,255,10,26,26,26,26,9,210,8,0,219,255,170,15,87,195,8,0};
LedControl lc=LedControl(DIN,CLK,CS,0);

enum State1 { 
  HLDA, WAIT, WO , STACK , MEMR, INP, M1, OUT, HLTA, PROT, INTE, INT };

enum Control { STOP, SINGLE_STEP, EXAMINE, DEPOSIT, RUN, SINGLE_STEP_,EXAMINE_NEXT,DEPOSIT_NEXT,AUX1_UP,AUX2_UP,PROTECT,RESET,CLR};
bool protect=false;
struct {
  int address;
  byte data;
  byte state;
} bus;

struct {
  int address;
  int control,prev_control;
} switches;

byte term_in() {
  return Serial.available() ? Serial.read() : 0; 
}

byte term_out(char c) {
  Serial.write(c & 0x7f);
}

int input(int port) {
  bitSet(bus.state,INP);
  
  static uint8_t character = 0;

  switch (port) {
    case 0x00:
      return 0;
    case 0x01: //serial read
      return term_in();
    case 0x10: //2SIO port 1 status
      if (!character) {
        character = term_in();
      }
      return (character ? 0b11 : 0b10); 
    case 0x11: 
      if (character) {
        int tmp = character; 
        character = 0; 
        return tmp; 
      } else {
        return term_in();
      }
    case 0xff: //sense switches
      return switches.address >> 8;
    default:
  return 0xff;
}
}

void output(int port, byte value) {
  bitSet(bus.state,OUT);

  switch (port) {
    case 0x01: 
      term_out(value);
      break;
    case 0x10: // 2SIO port 1 control
      //nothing
      break;
    case 0x11: // 2SIO port 1 write  
      term_out(value);
      break;
    case 0x12: // ????
      break;
    default:
      break;
  }
}


byte readByte(int addr) {
    bus.address=addr;
    bus.data=RAM[addr];
    bitSet(bus.state, MEMR);
    return RAM[addr];
}

void writeByte(int addr, byte value) {
    bus.address=addr;
    if (!protect) RAM[addr]=value;
    bus.data=value;
    bitClear(bus.state,WO); //inverted logic for write LED
}

int readWord(int address) {
  return readByte(address) | (readByte(address + 1) << 8);
}

void writeWord(int address, int value) {
  writeByte(address, value & 0xff);
  writeByte(address + 1, (value >> 8) & 0xff);
}

void writeLEDs() {
  
  led_D(bus.data);
  led_A(bus.address);
  led_S(bus.state);
}

void readSwitches(){
  switches.address=0;
  switches.prev_control = switches.control;
  switches.control=0;
 
 digitalWrite(2,LOW);
 digitalWrite(3,HIGH);
 digitalWrite(4,HIGH);
 digitalWrite(5,HIGH);
 digitalWrite(6,HIGH);
 switches.address+=digitalRead(A0)+digitalRead(A1)*32+digitalRead(A2)*1024+digitalRead(A3)*32768+digitalRead(A4)*8+digitalRead(A5)*16;
 digitalWrite(2,HIGH);
 digitalWrite(3,LOW);
 digitalWrite(4,HIGH);
 digitalWrite(5,HIGH);
 digitalWrite(6,HIGH);
 switches.address+=digitalRead(A0)*2+digitalRead(A1)*64+digitalRead(A2)*2048+digitalRead(A4)*256+digitalRead(A5)*512;
 switches.control+=digitalRead(A3)*256;
 digitalWrite(2,HIGH);
 digitalWrite(3,HIGH); 
 digitalWrite(4,LOW);
 digitalWrite(5,HIGH);
 digitalWrite(6,HIGH);
 switches.address+=digitalRead(A0)*4+digitalRead(A1)*128+digitalRead(A2)*4096+digitalRead(A4)*8192+digitalRead(A5)*16384;
 switches.control+=digitalRead(A3)*512;
 digitalWrite(2,HIGH);
 digitalWrite(3,HIGH);
 digitalWrite(4,HIGH);
 digitalWrite(5,LOW);
 digitalWrite(6,HIGH);
 switches.control+=digitalRead(A0)*16+digitalRead(A1)*32+digitalRead(A2)*64+digitalRead(A3)*128+digitalRead(A4)*4096;
 digitalWrite(2,HIGH);
 digitalWrite(3,HIGH);
 digitalWrite(4,HIGH);
 digitalWrite(5,HIGH);
 digitalWrite(6,LOW);
 switches.control+=digitalRead(A0)+digitalRead(A1)*2+digitalRead(A2)*4+digitalRead(A3)*8+digitalRead(A4)*2048+digitalRead(A5)*1024;
 
}

bool onRelease(int c) {
  return (switches.prev_control & (1<<c)) > 0 && (switches.control & (1<<c)) == 0;
}

bool isDown(int c) {
  return switches.control & (1<<c);
}

byte sense() {
  return switches.address >> 8;
}

extern "C" {
      
  //read/write byte
  int i8080_hal_memory_read_byte(int addr) {
    return readByte(addr);
  }
  
  void i8080_hal_memory_write_byte(int addr, int value) {
    writeByte(addr,value);
  }
  
  //read/write word
  int i8080_hal_memory_read_word(int addr) {
    return readWord(addr);
  }
  
  void i8080_hal_memory_write_word(int addr, int value) {
    writeWord(addr,value);
  }
  
  //input/output
  int i8080_hal_io_input(int port) {
    return input(port);
  }
  
  void i8080_hal_io_output(int port, int value) {
    output(port,value);
  }
  
  //interrupts
  void i8080_hal_iff(int on) {
    //nothing
  }
}


void run() {
  bitClear(bus.state,WAIT);
}

void stop() {
  bitSet(bus.state,WAIT);
}

void examine(int address) {
  i8080_jump(address); //set program counter
  readByte(address);
}

void deposit(int address, byte data) {
  i8080_jump(address); //set program counter
  writeByte(address,data);
  Serial.print(address);
  Serial.print(",");
  Serial.print(data);
  Serial.println();
}

void setup() {
  Serial.begin(115200);
  pinMode(A0,INPUT_PULLUP);
  pinMode(A1,INPUT_PULLUP);
  pinMode(A2,INPUT_PULLUP);
  pinMode(A3,INPUT_PULLUP);
  pinMode(A4,INPUT_PULLUP);
  pinMode(A5,INPUT_PULLUP);

  pinMode(2,OUTPUT);
  pinMode(3,OUTPUT);
  pinMode(4,OUTPUT);
  pinMode(5,OUTPUT);
  pinMode(6,OUTPUT);

  lc.shutdown(0,false);       //The MAX72XX is in power-saving mode on startup
  lc.setIntensity(0,7);      // Set the brightness to maximum value
  lc.clearDisplay(0);  

    
  bus.state=0;
  bitSet(bus.state,WAIT);
  
  
  bus.address = 0;
  bus.data = 0;
  i8080_init();
  examine(0);
}

void loop() {
  bitSet(bus.state,MEMR); //flag set by readByte()
  bitSet(bus.state,M1);  //flag set by step()
  bitClear(bus.state,OUT); //flag set by output()
  bitClear(bus.state,INP); //flag set by input()
  bitSet(bus.state,WO); //flag CLEARED by writeByte() inverted logic
  bitClear(bus.state, STACK); //set by readByte and writeByte if addr==SP

  readSwitches();
    
  if (onRelease(RUN)) run();
  if (onRelease(STOP)) stop();
  if (onRelease(SINGLE_STEP) || onRelease(SINGLE_STEP_)) {i8080_instruction();bus.address=i8080_pc();}
  if (onRelease(EXAMINE)) examine(switches.address);
  if (onRelease(EXAMINE_NEXT)) examine(i8080_pc()+1);
  if (onRelease(DEPOSIT)) deposit(i8080_pc(),switches.address);
  if (onRelease(DEPOSIT_NEXT)) deposit(i8080_pc()+1,switches.address);
  if (onRelease(RESET)) {led_D(255);led_A(65535);led_S(65535);delay(500);i8080_init();examine(0);}
  if (isDown(PROTECT)) {lc.setLed(0,0,7,1);protect=true;} else {lc.setLed(0,0,7,0);protect=false;}
  
  if (!bitRead(bus.state,WAIT)) {
    for (int i=0; i < 50; i++) 
      i8080_instruction();
  }

  writeLEDs();
}

void led_D(uint8_t value){
   lc.setLed(0,1,1,value&1);//d0
   lc.setLed(0,1,2,value&2);//d1
   lc.setLed(0,1,3,value&4);//d2
   lc.setLed(0,1,4,value&8);//d3
   lc.setLed(0,1,5,value&16);//d4
   lc.setLed(0,1,6,value&32);//d5
   lc.setLed(0,1,7,value&64);//d6
   lc.setLed(0,1,0,value&128);//d7
}

void led_A(uint16_t value){
   lc.setLed(0,3,1,value&1);//a0
   lc.setLed(0,3,2,value&2);//a1
   lc.setLed(0,3,3,value&4);//a2
   lc.setLed(0,3,4,value&8);//a3
   lc.setLed(0,3,5,value&16);//a4
   lc.setLed(0,3,6,value&32);//a5
   lc.setLed(0,3,7,value&64);//a6
   lc.setLed(0,3,0,value&128);//a7
   
   lc.setLed(0,2,1,value&256);//a8
   lc.setLed(0,2,2,value&512);//a9
   lc.setLed(0,2,3,value&1024);//a10
   lc.setLed(0,2,4,value&2048);//a11
   lc.setLed(0,2,5,value&4096);//a12
   lc.setLed(0,2,6,value&8192);//a13
   lc.setLed(0,2,7,value&16384);//a14
   lc.setLed(0,2,0,value&32768);//a15   
}

void led_S(uint8_t value){
  lc.setLed(0,4,0,value&2); //WAIT
  lc.setLed(0,4,6,value&4); //WO
  lc.setLed(0,0,1,value&8); //STACK
  lc.setLed(0,0,6,value&16); //MEMR
  lc.setLed(0,0,5,value&32); //INP
  lc.setLed(0,0,4,value&64); //M1
  lc.setLed(0,0,3,value&128); //OUT
  lc.setLed(0,0,3,value&128); //OUT
  
  }
