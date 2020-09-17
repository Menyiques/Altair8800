
extern "C" {
  #include "i8080.h"
}
const  uint8_t mnemonic_type[256]={ 0, 6,12, 0, 0, 0, 2, 0, 0,14,
                                          20, 0, 0, 0, 2, 0, 0, 6,12, 0,
                                           0, 0, 2, 0, 0,14,20, 0, 0, 0,
                                          02, 0, 0, 6,11, 0, 0, 0, 2, 0,
                                           0,14, 9, 0, 0, 0, 2, 0, 0, 6,
                                          10, 0,15,15, 3, 0, 0,14, 8, 0,
                                           0, 0, 2, 0, 0, 0, 0, 0, 0, 0,
                                          16, 0, 0, 0, 0, 0, 0, 0,16, 0,
                                           0, 0, 0, 0, 0, 0,16, 0, 0, 0,
                                           0, 0, 0, 0,16, 0, 0, 0, 0, 0,
                                           0, 0,16, 0, 0, 0, 0, 0, 0, 0,
                                          16, 0,13,13,13,13,13,13, 1,13,
                                           0, 0, 0, 0, 0, 0,16, 0, 0, 0,
                                           0, 0, 0, 0,13, 0, 0, 0, 0, 0,
                                           0, 0,13, 0, 0, 0, 0, 0, 0, 0,
                                          13, 0, 0, 0, 0, 0, 0, 0,16, 0,
                                           0, 0, 0, 0, 0, 0,13, 0, 0, 0,
                                           0, 0, 0, 0,13, 0, 0, 0, 0, 0,
                                           0, 0,13, 0, 0, 0, 0, 0, 0, 0,
                                          13, 0,18,17, 6, 6, 7,17, 2,21,
                                          18,18, 6, 6, 7, 7, 2,21,18,17,
                                           6, 5, 7,17,12,21,18,18, 6, 4,
                                           7, 7, 2,21,18,17, 6,19, 7,17,
                                           2,21,18, 0, 6, 0, 7, 7, 2,21,
                                          18,17, 6, 0, 7,17, 2,21,18, 0,
                                           6, 0, 7, 7, 2,21}; 

const uint8_t cycle_type[22][4]={{0,0,0,0},{10,0,0,0},{2,0,0,0},{2,12,0,0},{2,13,0,0},{2,14,0,0},{2,15,0,0},{2,15,19,18},{2,15,20,0},{2,15,20,23},{2,15,21,0},{2,15,5,5},{3,0,0,0},{4,0,0,0},{4,11,0,0},{4,12,0,0},{5,0,0,0},{6,16,0,0},{7,17,0,0},{7,7,22,9},{8,0,0,0},{9,2,0,0}};

#include <LedControl.h>
int DIN = 11;
int CS =  10;
int CLK = 13;
boolean cycle_mode; 
//byte RAM [512] = {33,0,0,22,128,1,255,10,26,26,26,26,9,210,8,0,219,255,170,15,87,195,8,0};//Kill the bit
byte RAM[512] ={ 0x3e,0xff, 0x21, 0x30, 0x00,0x31, 0x40, 0x00, 0x3d, 0xf5, 0x3c, 0xf1, 0x77, 0xc3, 0x00, 0x00}; //ld, push and pop samples
//uint8_t RAM [512]={0x3A,0x20,0x00,0x32,0x21,0x00,0x31,0x20,0x00,0xF5,0xF1,0xDB,0x10,0xD3,0x10,0xFB,0xF3,0x76}; //Video 4 altair

LedControl lc=LedControl(DIN,CLK,CS,0);

uint8_t current_cycle=1;

enum State1 {INTE, WO,  STACK,   MEMR,  INP, M1,  OUT, HLTA,  HLDA,  INT, WAIT,  PROT};

enum Control { STOP, SINGLE_STEP, EXAMINE, DEPOSIT, RUN, SINGLE_STEP_,EXAMINE_NEXT,DEPOSIT_NEXT,AUX1_UP,AUX2_UP,PROTECT,RESET,CLR};

bool protect=false;

struct {
  uint16_t address;
  uint8_t data;
  uint16_t state;
} bus;

struct {
  uint16_t address;
  uint16_t control,prev_control;
} switches;


byte term_in() {
  return Serial.available() ? Serial.read() : 0; 
}

byte term_out(char c) {
}

int input(int port) {
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
  return RAM[addr];
}

void writeByte(int addr, byte value) {
  bus.address=addr;
  if (!protect) RAM[addr]=value;
  bus.data=value;

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
   if (on==1){bitSet(bus.state,INTE);}else{bitClear(bus.state,INTE);};
  }
}


void run() {
  bitClear(bus.state,WAIT);
}

void stop() {
  bitSet(bus.state,WAIT);
}

void examine(uint16_t address) {
  i8080_jump(address); 
  readByte(address);
}

void deposit(int address, byte data) {
  i8080_jump(address); 
  writeByte(address,data);

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
  reset();
}  


void reset(){
  led_D(255);
  led_A(65535);
  led_S(255);
  delay(300);
  i8080_init();
  current_cycle=1;
  examine(0);
  cycle_step();
  }

void loop() {

  readSwitches();
 if (onRelease(RUN)) run();
 if (onRelease(STOP)) stop();
 if (onRelease(SINGLE_STEP) || onRelease(SINGLE_STEP_)) {if (cycle_mode){cycle_step();} else {i8080_instruction();current_cycle=1;cycle();logging();}}
 if (onRelease(EXAMINE)) {examine(switches.address);logging();}
 if (onRelease(EXAMINE_NEXT)) {examine(bus.address+1);logging();}
 if (onRelease(DEPOSIT)) {deposit(i8080_pc(),switches.address);logging();}
 if (onRelease(DEPOSIT_NEXT)) {deposit(i8080_pc()+1,switches.address);logging();}
 if (onRelease(RESET)) {reset();}
 if (isDown(PROTECT)) { bitSet(bus.state,PROT);} else { bitClear(bus.state,PROT);}
 if (isDown(AUX2_UP)) {cycle_mode=true;} else {cycle_mode=false;} 
 if (!bitRead(bus.state,WAIT)) {
    current_cycle=0;
    for (int i=0; i < 50; i++){ 
      i8080_instruction();  
    }
  }

  writeLEDs();

}
void cycle_step(){
    logging();
    if(cycle()==0){
        current_cycle=1;
        uint16_t ba=bus.address; uint8_t bd=bus.data; uint16_t bs=bus.state;
        i8080_instruction();
        bus.address=ba; bus.data=bd; bus.state=bs;
      }
    else {current_cycle++;}
}

void logging(){

    uint8_t mne=RAM[i8080_pc()];
    uint8_t mne_type=mnemonic_type[mne];
    String ciclos="[M1,"+(String)cycle_type[mne_type][0]+","+(String)cycle_type[mne_type][1]+","+(String)cycle_type[mne_type][2]+","+(String)cycle_type[mne_type][3]+"]";
    Serial.println("------------------------------------------------------------"); 
    Serial.println("PC="+hex4(i8080_pc())+" - SP="+hex4(i8080_regs_sp()));
    Serial.println("AF="+hex4(i8080_regs_af())+" - HL="+hex4(i8080_regs_hl()));
    Serial.println("BC="+hex4(i8080_regs_bc())+" - DE="+hex4(i8080_regs_de()));
    Serial.println("Mnemonic:"+hex2(mne)+" Type:"+mne_type+" Cycle:"+(String)current_cycle+" of "+ciclos);
    Serial.print("Address Bus=");Serial.println(hex4(bus.address));
    Serial.print("data bus=");Serial.println(hex2(bus.data));
    Serial.print("Cycle Mode=");Serial.print(cycle_mode);Serial.print(" Protect="); Serial.println(bitRead(bus.state,PROT));
    Serial.println("    x0 x1 x2 x3 x4 x5 x6 x7 x8 x9 xA xB xC xD xE xF"); 
    Serial.println("   -------------------------------------------------"); 
    for (int y=0;y<16;y++){
      Serial.print(hex2(y).substring(1)+"x| ");
      for (int x=0;x<16;x++){
        Serial.print(hex2(RAM[y*16+x])+" ");
        }Serial.println("|");
      }
    Serial.println("   -------------------------------------------------"); 
  
  }

  
String hex2(uint8_t b){
  char ba[2];
  ba[0]=(b>>4)+48;  
  if (ba[0]>57)ba[0]=ba[0]+7;
  ba[1]=(b&0x0F)+48;
  if (ba[1]>57)ba[1]=ba[1]+7;
  String r = "";
  r.concat(ba[0]);
  r.concat(ba[1]);
  return r;
  }

String hex4(uint16_t w){
  uint8_t b1=w/256;
  uint8_t b2=w%256;
  return hex2(b1)+hex2(b2);
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


void led_S(uint16_t value){
  uint8_t value2=value/256;
  uint8_t value1=value-value2*256;
  lc.setLed(0,0,0,value1&1); //INTE
  lc.setLed(0,4,6,value1&2); //WO  
  lc.setLed(0,0,1,value1&4); //STACK
  lc.setLed(0,0,6,value1&8); //MEMR
  lc.setLed(0,0,5,value1&16); //INP
  lc.setLed(0,0,4,value1&32); //M1
  lc.setLed(0,0,3,value1&64); //OUT
  lc.setLed(0,0,2,value1&128); //HLTA
  
  lc.setLed(0,4,7,value2&1); //HLDA
  lc.setLed(0,4,5,value2&2); //INT
  lc.setLed(0,4,0,value2&4); //WAIT
  lc.setLed(0,0,7,value2&8); //PROT

}
  
uint8_t cycle(){
uint8_t mne=RAM[i8080_pc()];

if (current_cycle==1){ //M1
         bus.address=i8080_pc();
         bus.data=RAM[bus.address];
         bus.state=B00101010;
  }

if (current_cycle>1){
uint8_t c=cycle_type[mnemonic_type[mne]][current_cycle-2];
    switch(c){
      case 0:
        break;
       case 2:
         bus.address=i8080_pc()+1;
         bus.data=RAM[bus.address];
         bus.state=B00001010;
         break;
       case 3:
         bus.address=i8080_pc()+1;
         bus.data=RAM[bus.address];
         bus.state=B00000000;
         break;
       case 4:
         bus.address=i8080_regs_hl();
         bus.data=RAM[i8080_regs_hl()];
         bus.state=B00001010;        
         break;  
       case 5:
         bus.address=i8080_regs_hl();
         bus.data=RAM[i8080_regs_hl()];
         bus.state=B00000000;
         break;  
       case 6:
         bus.address=i8080_regs_sp();
         bus.data=RAM[i8080_regs_sp()];
         bus.state=B00001110;
         break;  
       case 7:
         bus.address=i8080_regs_sp()+1;
         bus.data=RAM[i8080_regs_sp()+1];
         bus.state=B00001110;
         break;  
       case 8://LDAX rr | LD A, (rr) revisar
         bus.address=i8080_regs_sp()+1;
         bus.data=RAM[i8080_regs_sp()+1];
         bus.state=B00001010;
         break;  
       case 9:
         bus.address=i8080_regs_sp()-1;
         bus.data=RAM[i8080_pc()+2];
         bus.state=B10001010;
         break;  
       case 10:
         bus.address=65535;
         bus.data=255;
         bus.state=B10001010;
         break;  
       case 11:
         bus.address=i8080_regs_hl();
         bus.data=RAM[i8080_regs_hl()];
         bus.state=B00001010;
         break;  
       case 12:
         bus.address=i8080_regs_hl();
         bus.data=RAM[i8080_regs_hl()];
         bus.state=B00000000;
         break;          
      case 13://IN
         bus.address=RAM[i8080_pc()+1]*257;
         bus.data=input(RAM[i8080_pc()+1]);
         bus.state=B00010010;
         break;    
       case 14://OUT
         bus.address=RAM[i8080_pc()+1]*257;
         bus.data=i8080_regs_a();
         bus.state=B01000000;
         break; 
       case 15:
         bus.address=i8080_pc()+2;
         bus.data=RAM[i8080_pc()+2];
         bus.state=B00001010;
         break;      
       case 16:
         bus.address=i8080_regs_sp()+1;
         bus.data=RAM[i8080_regs_sp()+1];
         bus.state=B00001110;
         break;      
       case 17:
         bus.address=i8080_regs_sp()+2;
         bus.data=RAM[i8080_regs_sp()+2];
         bus.state=B00001110;
         break;      
       case 18:
         bus.address=i8080_regs_sp()+1;
         bus.data=RAM[i8080_pc()+1];
         bus.state=B00000100;
         break;   
       case 19:
         bus.address=i8080_regs_sp();
         bus.data=RAM[i8080_pc()+2];
         bus.state=B00000100;
         break;                                                                        
       case 20:
         bus.address=RAM[i8080_pc()+1]+RAM[i8080_pc()+2]*256;
         bus.data=RAM[bus.address];
         bus.state=B00001010;
         break;              
       case 21:
         bus.address=RAM[i8080_pc()+1]+RAM[i8080_pc()+2]*256;
         bus.data=RAM[bus.address];
         bus.state=B00000000;
         break;                                                                 
       case 22:
         bus.address=i8080_regs_sp()-1;
         bus.data=RAM[i8080_pc()+2];
         bus.state=B00000100;
         break;       
       case 23:
         bus.address=(RAM[i8080_pc()]+RAM[i8080_pc()+1]*256)+1;
         bus.data=RAM[bus.address];
         bus.state=B00001010;
         break;       
      }
/*      
1  Fetch PC  (PC)  Fetch 
2 PC+1(PC+1)Mem Rd  PC+1  (PC+1)  Mem Rd  B01010000
3 PC+1(PC+1)Mem Wr  PC+1  (PC+1)  Mem Wr  B00000000
4 HL(HL)Mem Rd  HL  (HL)  Mem Rd  B01010000
5 HL(HL)Mem Wr  HL  (HL)  Mem Wr  B00000000
6 SP-1(SP-1)Stack Wr  SP-1  (SP-1)  Stack Wr  B00100000
7 SP+1(SP+1)Stack Rd  SP+1  (SP+1)  Stack Rd  B01110000
8 rr(rr)Mem Rd  rr  (rr)   Mem Rd B01010000
9 SP-1(PC+2)Stack Wr  SP-1  (PC+2)  Stack Wr  B00100000
10  PCFFHaltAck PC  FF  HaltAck B01010001
11  HL(HL)Mem Rd  HL  (HL)  Mem Rd  B01010000
12  HL(HL)Mem Wr  HL  (HL)  Mem Wr  B00000000
13  N,NAInput Rd  N,N A Input Rd  B01001000
14  N,NAOutput Wr N,N A Output Rd B00000010
15  PC+2(PC+2)Mem Rd  PC+2  (PC+2)  Mem Rd  B01010000
16  SP-2(SP-2)Stack Wr  SP-2  (SP-2)  Stack Wr  B00100000
17  SP+2(SP+2)Stack Rd  SP+2  (SP+2)  Stack Rd  B01110000
18  SP+1(PC+1)Stack Wr  SP+1  (PC+1)  Stack Wr  B00100000
19  SP(PC+2)Stack Wr  SP  (PC+2)  Stack Wr  B00100000
20  NN(NN)Mem Rd  NN  (NN)  Mem Rd  B01010000
21  NNAMem Wr NN  A Mem Wr  B00000000
22  SP-1(PC+2)Stack Wr    (PC+2)  Stack Wr  B00100000
23  NN+1(NN+1)Mem Rd    (NN+1)  MemRdç  B01010000
  */  }

  bitSet(bus.state,WAIT);
  uint8_t temp=cycle_type[mnemonic_type[mne]][current_cycle-1];
  return temp;
    }
