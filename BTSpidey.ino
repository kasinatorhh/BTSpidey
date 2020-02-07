/*************************************************** 
  This is the Code for Arduino Spider with PCA9685 PWM controller.
  3D Design Data and Buildup Guide is found here:
  https://www.thingiverse.com/thing:4070234
  The Spider uses pins see chapter pinmapping below
  
  Written by Limor Fried/Ladyada for Adafruit Industries.  
  BSD license, all text above must be included in any redistribution
 ****************************************************/
#define ENABLE_DEBUG_OUTPUT
#include <Wire.h>
#define ENABLE_DEBUG_OUTPUT
#include <Adafruit_PWMServoDriver.h>
#include <EEPROM.h>
#include <FlexiTimer2.h>//to set a timer to manage all servos
#include <Servo.h>
#include <SoftwareSerial.h>
#include "Spider_Hardware.h"
#include "ServoHAL.h"
#include "MoveCore.h"


/**********************************
 * Global Variables / Class inits
 *********************************/
// called this way, it uses the default address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
SoftwareSerial BTSerial(BT_TX, BT_RX); // RX, TX

#define UARTFAST 115200
#define BAUDBTCONF 38400
#define BAUDBTDEFAULT 9600
#define BAUDBTCOM 57600

#define DEBUG_COMMANDS 1
TwoSerialWrap TSerial(&Serial, UARTFAST, &BTSerial, BAUDBTCOM);
ServoHAL Srv;

bool EEPROMUPDATESENABLED = false;

//Structure to store servo related calibration data
typedef struct {
  Servo_Cal_T ServoSpec[13];
  uint8_t RFU[128];
  uint8_t BTName[32];
  uint8_t EEPROMUpdatesDisabled;
} EEPROM_T;
EEPROM_T * NVM=0;
// Depending on your servo make, the pulse width min and max may vary, you 
// want these to be as small/large as possible without hitting the hard stop
// for max range. You'll have to tweak them as necessary to match the servos you
// have!
#define SERVOMIN  100 // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  600 // This is the 'maximum' pulse length count (out of 4096)
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates
#define INPUT_SIZE 30

/**********************************
 * Global Variables / Class inits
 *********************************/

enum POSITIONS{
  POS_INIT=255,
  POS_REF1=254,
  POS_REF2=253
};

void setup() {  
  // Open serial communications and wait for port to open:
  Serial.begin(UARTFAST);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for Native USB only
  }
  SetBluetoothPower(HIGH,BT_COM);
  BTSerial.begin(BAUDBTCOM);
  BTSerial.setTimeout(2);
  TSerial.running=true;
  Serial.setTimeout(2);
  TSerial.println(F("Welcome to Spider Bot!"));
  InitPWM();
//  TSerial.println(F("Servos poweroff"));
//  Srv.Detatch(-1);
  TSerial.print(F("Loading Servo Calibration:"));
  Srv.CalRestore((int)&(((EEPROM_T*)0)->ServoSpec));
  Srv.CalPrint();
  if (UpdatesEnabled()){
    TSerial.print(F("EEPROM Writes Enabled. Send PROTECT to switch off\n"));
  }else{
    PrintUpdateMessage();
  }
  TSerial.println(F("Init Leg positions"));
  Mover.Move(MOV_INIT,0);
  //start servo service
  FlexiTimer2::set(20, MoveCore::servo_service);
  //initialize servos
  if (false){
    Srv.Attach(-1);
    Mover.Move(MOV_STAND,0);
    Mover.Move(MOV_STEPFORWARD,2);
    Mover.Move(MOV_HEADUP,20);
    Mover.Move(MOV_NO,2);
    Mover.Move(MOV_HANDSHAKE,3);
    delay(500);
    Mover.Move(MOV_STEPBACK,2);
    delay(500);
    Mover.Move(MOV_SIT,0);
  }
//  FlexiTimer2::start();
  PrintHelp();
  TSerial.println(F("Ready for Command:"));
}

#define MAGICEEPROMUPDATE_ENABLED 0x1337
bool UpdatesEnabled(){
  uint16_t Magic;
  EEPROM.get((int)&NVM->EEPROMUpdatesDisabled, Magic);
  return Magic==MAGICEEPROMUPDATE_ENABLED;
}
void EnableUpdates(){
    EEPROM.put((int)&NVM->EEPROMUpdatesDisabled,(uint16_t)MAGICEEPROMUPDATE_ENABLED);
}
void DisableUpdates(){
    EEPROM.put((int)&NVM->EEPROMUpdatesDisabled,(uint16_t)!MAGICEEPROMUPDATE_ENABLED);
}
void PrintUpdateMessage(){
    TSerial.println(F("Updates are Disabled, send UNPROTECT to activate"));
}

void InitPWM(){
  TSerial.println(F("Setup Servo Board"));
  pwm.begin();
  // In theory the internal oscillator is 25MHz but it really isn't
  // that precise. You can 'calibrate' by tweaking this number till
  // you get the frequency you're expecting!
  pwm.setOscillatorFrequency(27000000);  // The int.osc. is closer to 27MHz  
  pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates  
  for (int S=0;S<13;S++){
    Srv.SetHandler(S,&pwm,S);//All Servos have PWM control in latest wiring
  }
}

void SetBluetoothPower(uint8_t OnOff, uint8_t EN){
  Serial.print(F("Setting Bluetooth Power:"));
  Serial.print(OnOff);
  Serial.write(',');
  Serial.println(EN==BT_COM?F("CONFIG"):F("COM"));
 //prevent cross currents on shorted portPins
  pinMode(BT_VDD3, INPUT);
  pinMode(BT_VDD2, INPUT);
  pinMode(BT_EN, OUTPUT); //
  digitalWrite(BT_EN, EN); //EN-KEY = Control path
  digitalWrite(BT_VDD3,OnOff);
  digitalWrite(BT_VDD2,OnOff);
  digitalWrite(BT_VDD1,OnOff);
  pinMode(BT_VDD1, OUTPUT);
  pinMode(BT_VDD2, OUTPUT);
  pinMode(BT_VDD3, OUTPUT);
  pinMode(BT_STATE, INPUT);
}

uint8_t BTSetConfig(uint8_t EN){
  uint8_t BAK=digitalRead(BT_EN);
  Serial.print(F("BTConfig "));
  Serial.print(EN);
  Serial.print(F(", Starting "));
  Serial.println(BAK);
  digitalWrite(BT_EN,EN);
  delay(1000);
  return BAK;
}
bool BTCMD(const __FlashStringHelper* CMD){
  uint8_t ENBAK = BTSetConfig(BT_CONFIG);
  BTSerial.print(CMD);
  Serial.print(F("Sending to BT>"));
  Serial.print(CMD);
  uint8_t Timeout = 30;
  while(!BTSerial.available()){
    delay(100);
    if (--Timeout){break;};
  }
  delay(10);
  bool found=false;
  while (BTSerial.available()){
    Serial.write(BTSerial.read());
    found=true;
  }
  BTSetConfig(ENBAK);
  return found;
}

void CheckBlueTooth(){
  BTCMD(F("AT\r\n"));
}

void FormatBlueTooth(){
  BTSerial.end();
  SetBluetoothPower(0,0);
  delay(500);
  SetBluetoothPower(1,BT_CONFIG);
  BTSerial.begin(BAUDBTCONF);
  delay(900);
  bool found=false;
  for (int i=0;i<4;i++){
    found=BTCMD(F("AT\r\n"));
    if (found) break;
    Serial.write('\n');
  }
  BTCMD(F("AT+ORGL\r\n"));
  BTCMD(F("AT+UART=57600,1,0\r\n"));
  BTCMD(F("AT+NAME=SPIDEY\r\n"));
  SetBluetoothPower(0,BT_COM);
  BTSerial.end();
  delay(500);
  SetBluetoothPower(1,BT_COM);
  BTSerial.begin(BAUDBTCOM);
}

void PosServo(uint8_t ServoIdx, float Value){
  if (DEBUG_COMMANDS) TSerial.println(F("PosServo"));
  Srv.write(ServoIdx,Value);
}

void SyncServoMove(uint8_t ServoAxis, uint16_t Delay){
  if (DEBUG_COMMANDS) TSerial.println(F("SyncServoMove"));
  for (float percent=0;percent>-90;percent=percent-3){
    for (int arm = 0;arm<4;arm++){
      Srv.write(ServoAxis+arm*3,percent);
    }
    delay(Delay);
  }
  for (float percent=-90;percent<100;percent=percent+1){
    for (int arm = 0;arm<4;arm++){
      Srv.write(ServoAxis+arm*3,percent);
    }
    delay(Delay);
  }
  for (float percent=100;percent>=0;percent=percent-2){
    for (int arm = 0;arm<4;arm++){
      Srv.write(ServoAxis+arm*3,percent);
    }
    delay(Delay);
  }
}

void SetValue(uint8_t ServoID, SrvCal Entry, const __FlashStringHelper * Name, uint16_t Value){
  if (UpdatesEnabled()){
    TSerial.print(F("Servo "));
    TSerial.print(ServoID);
    TSerial.print(F(", Entered Value for "));
    TSerial.print(Name);
    TSerial.print(F(" was:"));
    TSerial.println(Value);
    Srv.CalSet(ServoID,Entry,Value);
    Srv.CalPrint();
    Srv.CalBackup((int)&NVM->ServoSpec);
  }else{
    PrintUpdateMessage();
  }
}

void printHEX(uint8_t len, char *input){
    Serial.print(F("Input Data:"));
    for (uint8_t i=0;i<len;i++){
      Serial.write(input[i]);
    }
    Serial.print(F("\nin Hex:"));
    for (uint8_t i=0;i<len;i++){
      Serial.write(' ');
      if (input[i]<16) Serial.print('0');
      Serial.print(input[i],HEX);
    }
    Serial.write('\n');
}

void PrintHelp(void){
  TSerial.println(F("Supported Commands, CMD ServoID Value:"));
  TSerial.println(F("d:SetValue S SRV_MODE,N:SetValue S MIN,M:SetValue S MAX,0:SetValue S Ref0,r:SetValue S REF90,R:SetValue S REF180"));
  TSerial.println(F("g:CountServo,G:PosServo,y:MoveIdenticServos,SweepIdenticServos,*:FormatServo"));
  TSerial.println(F("Supported Stand alone commands:"));
  TSerial.println(F("x:DetatchServos, X:AttachServos,*:FormatServo,^:Reset,?:PrintPos,#:PrintCal"));
  TSerial.println(F("q:FormatBluetooth,Q:CheckBlueTooth"));
  TSerial.println(F("Supported Mover commands:"));
  TSerial.print((char)MOV_STAND);TSerial.print(F(":Stand,"));
  TSerial.print((char)MOV_SIT);TSerial.print(F(":Sit,"));
  TSerial.print((char)MOV_STEPFORWARD);TSerial.print(F(":Forward,"));
  TSerial.print((char)MOV_STEPBACK);TSerial.println(F(":Backward"));
  TSerial.print((char)MOV_TURNLEFT);TSerial.print(F(":TurnLeft,"));
  TSerial.print((char)MOV_TURNRIGHT);TSerial.print(F(":TurnRight,"));
  TSerial.print((char)MOV_HANDWAVE);TSerial.print(F(":HandWave,"));
  TSerial.print((char)MOV_HANDSHAKE);TSerial.println(F(":HandShake"));
  TSerial.print((char)MOV_HEADUP);TSerial.print(F(":HeadUp,"));
  TSerial.print((char)MOV_HEADDOWN);TSerial.print(F(":HeadDown,"));
  TSerial.print((char)MOV_NO);TSerial.println(F(":No"));
}

void loop() {
  char input[INPUT_SIZE+1];
  char cmd;
  uint8_t ServoID;
  int8_t RX_X;
  int8_t RX_Y;
  uint16_t Value;
  String inp;
  digitalWrite(LED,digitalRead(BT_STATE));

  if (!TSerial.running)TSerial.go();
  for (int i = 0; i < sizeof(input);i++){
    input[i]=0;
  }
  
  byte rxlen=0;
  while(TSerial.available()){
    delay(2);
    uint8_t rc=TSerial.read();
    if (rc==32||(rc>43)||(rc<58)||(rc>64)||(rc<123)) //valid characters: a-zA-Z,.-0-9
    { 
      input[rxlen++]=rc;
    }else{
      input[rxlen]=0;
      TSerial.end();
      break;
    }
  }
//  if (TSerial.available()){// While serial data are available we store it. Prio on HW
//    rxlen=TSerial.readBytes(input,INPUT_SIZE);
//    input[rxlen]=0;
//  }
  if (rxlen>0){
    printHEX(rxlen,input);
  }
  if (rxlen>0){
    if (input[0]=='%'){ //DIY Bluetooth Sender
      //parsing bytes
      RX_X=0;RX_Y=0;
      for (int i=1;i<rxlen;i++){
        if (input[i]=='X'){
          RX_X=input[++i]-48;
        }else if (input[i]=='Y'){
          RX_Y=input[++i]-48;
        }
      }
      RX_X-=4;RX_Y-=4;
      TSerial.print(F("X="));
      TSerial.print(RX_X);
      TSerial.print(F(" Y="));
      TSerial.println(-RX_Y);
      rxlen=0;//mark handled
    }else if (input[0]=='+'){ //disconnect command
      rxlen=0;//do not handle
    }
    if ((rxlen>0)&&((input[1]=='X')||(input[3]=='Y')||(input[5]==';'))){//assume DIY Bluetooth mode. (there are bit-errors)
      rxlen=0;
      Serial.println(F("Cleaning unhandled BT DIY commands"));
    }
  }
  //others
  if (rxlen>0){
    ServoID=15;
    Value=65535;
    char* command = strtok(input, " ;\n");
    cmd=command[0];
    char* separator= strtok(NULL,' ');
    if (separator != NULL) ServoID=atoi(separator);
    ++separator;
    Value=atoi(++separator);
    TSerial.print(F("Command:"));TSerial.print(cmd);
    TSerial.print(F(",ServoID:"));TSerial.print(ServoID);
    TSerial.print(F(",Value:"));TSerial.println(Value);
    switch (cmd){
      case 'g': FlexiTimer2::stop();pwm.setPWM(ServoID,0,Value);break;
      case 'G': PosServo(ServoID, (float)(int)Value);break;
      case 'x': FlexiTimer2::stop();InitPWM();Srv.Detatch(-1);break;
      case 'X': InitPWM();Srv.Attach(-1);FlexiTimer2::start();break;
      case 'y': Srv.write(Mover.ServoIdx(0,ServoID),(float)(int)Value);
                Srv.write(Mover.ServoIdx(1,ServoID),(float)(int)Value);
                Srv.write(Mover.ServoIdx(2,ServoID),(float)(int)Value);
                Srv.write(Mover.ServoIdx(3,ServoID),(float)(int)Value);
                break;
      case 'Y': SyncServoMove(Value,20);break;
      case 'd': SetValue(ServoID, SRV_MODE,F("Direction? 0=straight 1=invert"),Value);break;
      case 'N': SetValue(ServoID, SRV_MIN, F("Min, 300?"),Value);break;
      case 'M': SetValue(ServoID, SRV_MAX, F("Max, 500?"),Value);break;
      case 'r': SetValue(ServoID, SRV_REF90, F("Center Position"),Value);break;
      case '0': SetValue(ServoID, SRV_REF0, F("-90 Degrees"),Value);break;
      case 'R': SetValue(ServoID, SRV_REF180, F("90 Degrees"),Value);break;
      case '?': Srv.PrintPos();break;
      case '!': Srv.PrintPCA9685();break;
      case '#': Srv.CalPrint();break;
      case '*': Srv.InitServoParameters(ServoID,ServoID,SRV_MODE_INVALID,SERVOMIN,SERVOMAX,SERVOMIN,SERVOMAX-SERVOMIN/2,SERVOMAX);
                if (UpdatesEnabled()){
                  Srv.CalBackup((int)&NVM->ServoSpec);
                }else{ PrintUpdateMessage();};
                Srv.CalPrint();break;
      case 'U': if (input=="UNPROTECT"){EnableUpdates();};break;
      case 'P': if (input=="PROTECT"){DisableUpdates();};break;
//      case 'A': SwipeServo(Value,20);break;
//      case 's': Position(ServoID,300);break;
//      case 'S': Position(ServoID,100);break;
      case MOV_STEPFORWARD: Mover.Move(MOV_STEPFORWARD,1);break;
      case MOV_STEPBACK: Mover.Move(MOV_STEPBACK,1);break;
      case MOV_TURNLEFT: Mover.Move(MOV_TURNLEFT,1);break;
      case MOV_TURNRIGHT: Mover.Move(MOV_TURNRIGHT,5);break;
      case MOV_STAND: Mover.Move(MOV_STAND,0);break;
      case MOV_SIT: Mover.Move(MOV_SIT,0);break;
      case MOV_HANDWAVE: Mover.Move(MOV_HANDWAVE,0);break;
      case MOV_HANDSHAKE: Mover.Move(MOV_HANDSHAKE,0);break;
      case MOV_HEADUP: Mover.Move(MOV_HEADUP,20);break;
      case MOV_HEADDOWN: Mover.Move(MOV_HEADUP,0);break;
      case MOV_NO: Mover.Move(MOV_NO,Value);break;
      case '^': asm volatile ("  jmp 0");break;
      case 'q': FormatBlueTooth();break;
      case 'Q': CheckBlueTooth();break;
      case 27: cmd=command[2];
               switch (cmd)
               {
                case 65:TSerial.println("up");Mover.Move(MOV_STEPFORWARD,1);break;
                case 66:TSerial.println("down");Mover.Move(MOV_STEPBACK,1);break;
                case 67:TSerial.println("right");Mover.Move(MOV_TURNRIGHT,1);break;
                case 68:TSerial.println("left");Mover.Move(MOV_TURNLEFT,1);break;
                case 50:TSerial.println("ins");Mover.Move(MOV_HANDSHAKE,1);break;
                case 51:TSerial.println("del");Mover.Move(MOV_HANDWAVE,1);break;
                case 53:TSerial.println("pgup");Mover.Move(MOV_STAND,0);break;
                case 54:TSerial.println("pgdn");Mover.Move(MOV_SIT,0);break;
                default:TSerial.println((int)cmd);break;                
               }
               break;
      case 'D' : ServoDebug=Value;TSerial.print(F("ServoDebug:"));
                TSerial.println(Value);break;
      default : TSerial.println("Unknown command:");
                TSerial.println((int)command[0]);
                TSerial.println((int)command[1]);
                TSerial.println((int)command[2]);
                PrintHelp();
    }
    TSerial.println("Done");
    command = strtok(0, 10);
  }
}

/*****************************
 * Section deals with Servos
 */
 //void Position(uint8_t Value, uint16_t Speed){
//  if (DEBUG_COMMANDS) TSerial.println(F("Position"));
//  switch (Value){
//    case 0:for (int arm=0;arm<4;arm++){
//               ServoNext[0+arm]=GetServoNext(0+arm,100);
//               ServoNext[4+arm]=GetServoNext(4+arm,250);
//               ServoNext[8+arm]=GetServoNext(8+arm,20);
//           }
//           break;
//    case 1:for (int arm=0;arm<4;arm++){
//           ServoNext[4+arm]=GetServoNext(4+arm,128);
//           ServoNext[8+arm]=GetServoNext(8+arm,128);
//           }
//           break;
//    case POS_INIT:
//           for (int servo=0;servo<8;servo++){
//               setPWM(servo,ServoSpec[servo].Reflen1);//Home Coxa+Thibia
//           }
//           for (int servo=8;servo<12;servo++){
//               setPWM(servo,ServoSpec[servo].Reflen2);//Home Femur
//           }
//           break;
//    case POS_REF1:
//           for (int servo=0;servo<8;servo++){
//               setPWM(servo,ServoSpec[servo].Reflen1);//Home Coxa+Thibia
//           }
//           for (int servo=8;servo<12;servo++){
//               setPWM(servo,ServoSpec[servo].Reflen1);//Home Femur
//           }
//           break;
//    case POS_REF2:
//           for (int servo=0;servo<8;servo++){
//               setPWM(servo,ServoSpec[servo].Reflen2);//Home Coxa+Thibia
//           }
//           for (int servo=8;servo<12;servo++){
//               setPWM(servo,ServoSpec[servo].Reflen2);//Home Femur
//           }
//           break;
//          
//  }
//}
//void SwipeServo(uint8_t ServoIdx, uint16_t Delay){
//  if (DEBUG_COMMANDS) TSerial.println(F("SwipeServo"));
//  for (float percent=0;percent>-90;percent=percent-3){
//    ServoWrite(ServoIdx,percent);
//    delay(Delay);
//  }
//  for (float percent=-90;percent<100;percent=percent+1){
//    ServoWrite(ServoIdx,percent);
//    delay(Delay);
//  }
//  for (float percent=100;percent>=0;percent=percent-2){
//    Srv.ServoWrite(ServoIdx,percent);
//    delay(Delay);
//  }
//}
