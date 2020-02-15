/*************************************************** 
  This is the Code for Arduino Spider with PCA9685 PWM controller.
  3D Design Data and Buildup Guide is found here:
  https://www.thingiverse.com/thing:4070234
  The Spider uses pins see chapter pinmapping below
  
  BSD license, all text above must be included in any redistribution
 ****************************************************/
#include <arduino.h>
#include <EEPROM.h>
#include "ServoHAL.h"
#include "TwoSerialWrap.h"

DebugLevel ServoDebug=dbgOff; //set via command "D 1 <DebugLevelValue>\r\n"

uint8_t BodyAttached = 0; 

const char strBytes[] PROGMEM={"Bytes "};
const char strWritten[] PROGMEM={"written to "};
const char strRead[] PROGMEM={"read from "};
const char strEEPROM[] PROGMEM={"EEPROM"};
typedef struct {
  char name[5];
}CalNames_t;
const CalNames_t CalNames[SRV_MAXCALENUM] PROGMEM={
 {"Chan"},
 {"Mode"},
 {"Mini"},
 {"Maxi"},
 {"Ref-"},
 {"Ref9"},
 {"Ref+"},
};
uint16_t ServoHAL::GetPos(int ServoIdx){
  return _ServoPos[ServoIdx];
}

void ServoHAL::PrintPos(void){
  TSerial.print(F("Reporting ServoPos"));
  for (int i = 0; i<13; i++){
    TSerial.print(':');
    TSerial.print(i);
    TSerial.print('=');
    TSerial.print(Srv.GetPos(i));
  }
  TSerial.println(';');
}

//Load Calibration Data from EEPROM (always) and return 0 if Data was valid
int ServoHAL::CalRestore(int Base){
  uint16_t Magic;
  EEPROM.get(Base+sizeof(_ServoCal),Magic);
  if (Magic==SERVOMAGIC){
    Magic=0;
  };
  EEPROM.get(Base,_ServoCal);
  TSerial.print(sizeof(_ServoCal));
  TSerial.println(F(" read"));
  return Magic;
}
//Write Calibration Data to EEPROM and mark as valid
void ServoHAL::CalBackup(int Base){
  EEPROM.put(Base,_ServoCal);
  EEPROM.put(Base+sizeof(_ServoCal),SERVOMAGIC);
  TSerial.print(sizeof(_ServoCal));
  TSerial.println(F(" written"));
}

void ServoHAL::InitServoParameters(int servoidx,uint16_t Channel, uint16_t Mode, uint16_t Min, uint16_t Max, uint16_t Ref0, uint16_t Ref90, uint16_t Ref180){
  Servo_Cal_T * cal = &_ServoCal[servoidx];
  cal->Channel=Channel;cal->Mode=Mode;cal->Min=Min;cal->Max=Max;cal->Ref0=Ref0;cal->Ref90=Ref90;cal->Ref180=Ref180;
}

void ServoHAL::PrintCalName(SrvCal Item){
  CalNames_t OneName;
  TSerial.write(' ');
  memcpy_P (&OneName,&CalNames[Item],sizeof(OneName));
  TSerial.print(OneName.name);
}
void ServoHAL::writeCount(uint8_t ServoIdx, int count){
  TSerial.print(F("Setting Servo "));
  TSerial.print(ServoIdx);
  TSerial.print(F(" to "));
  TSerial.print(count);
  TSerial.println(F(" Counts"));
  setPWM(ServoIdx,count);
}
void ServoHAL::CalPrint(void){
  for (uint8_t i = 0;i<13;i++){
    TSerial.print(F("Servo "));
    TSerial.print(i);
    for (uint8_t j=0;j<SRV_MAXCALENUM;j++){
      PrintCalName(j);
      TSerial.print('=');
      TSerial.print(CalGet(i,j));
      TSerial.print(',');
    }
    TSerial.print(F(" Path="));
    TSerial.println(GetPath(i));
  }
}
uint16_t ServoHAL::CalGet(int ServoIdx, SrvCal Entity){
  uint16_t * CalVal=(uint16_t*)&_ServoCal[ServoIdx];
  return CalVal[Entity];
}
void ServoHAL::CalSet(int ServoIdx, SrvCal Entity, uint16_t Value){
  uint16_t * CalVal=(uint16_t*)&_ServoCal[ServoIdx];
  if (ServoIdx<NUMSERVOS) CalVal[Entity]=Value;
}
uint16_t ServoHAL::GetPath(int ServoIdx){
  return _ServoCal[ServoIdx].Max-_ServoCal[ServoIdx].Min;
}

//todo: change updates to Burst Register writes, All Servo positions get updated per 20ms.
void ServoHAL::PrintPCA9685(){
  Wire.beginTransmission(0x40);
  Wire.write(0); //start at zero
  Wire.endTransmission();
  Wire.requestFrom(0x40,70);
  TSerial.print(F("Print PCA9685 registers:"));
  while (Wire.available()){
    char c = Wire.read();
    TSerial.print(c,HEX);
  }
  TSerial.print(F(":Done"));
}

void ServoHAL::Detatch(int ServoIdx){
  if (ServoIdx=-1){
    if (BodyAttached!=0){
      for (ServoIdx=0;ServoIdx<NUMSERVOS;ServoIdx++)
      { 
        Detatch(ServoIdx);
      }
    }
    BodyAttached=0;
  }else{
    if (_Handlers[ServoIdx]!=NULL){
      switch (_ServoCal[ServoIdx].Mode){
        case SRV_MODE_PCA9685: ((Adafruit_PWMServoDriver*)_Handlers[ServoIdx])->setPWM(ServoIdx,0,0);break;
        case SRV_MODE_SERVO: ((Servo*)_Handlers[ServoIdx])->detach();break;
      }
    }
  }
}
void ServoHAL::Attach(int i){
  if (i<0){
    if (BodyAttached!=1){
      for (int ServoIdx=0;ServoIdx<NUMSERVOS;ServoIdx++)
      { 
        Attach(ServoIdx);
      }
    }
    BodyAttached=1;
  }else{
    TSerial.println("Servo Attach");
    BodyAttached=1;
  }
}

void ServoHAL::SetHandler(int ServoIdx, Adafruit_PWMServoDriver * pwm, uint16_t Channel){
  _ServoCal[ServoIdx].Mode=SRV_MODE_PCA9685;
  _Handlers[ServoIdx]=(int)(int*) pwm;
  _ServoCal[ServoIdx].Channel=Channel;
}
void ServoHAL::SetHandler(int ServoIdx, Servo * Serv, uint16_t Channel){
  _ServoCal[ServoIdx].Mode=SRV_MODE_SERVO;
  _Handlers[ServoIdx]=(int)(int*) Serv;
  _ServoCal[ServoIdx].Channel=Channel;
}
  
uint16_t ServoHAL::GetServoNext(uint8_t ServoIdx, float percent){
  uint16_t Value;
  Value=map(percent,0,90,_ServoCal[ServoIdx].Ref90,_ServoCal[ServoIdx].Ref180);
//  if (ServoDebug==dbgGetServoNextBody){
//    TSerial.print(F("GetServoNext:"));
//    TSerial.print(percent,2);
//    TSerial.print(F("%="));
//    TSerial.println(Value);
//  }
  if (Value>_ServoCal[ServoIdx].Max)Value=_ServoCal[ServoIdx].Max;
  if (Value<_ServoCal[ServoIdx].Min)Value=_ServoCal[ServoIdx].Min;
  return Value;
}

//void ServoHAL::ShowConversionDone(const __FlashStringHelper* From,const __FlashStringHelper* To){
//  if (ServoDebug==dbgServoConversions){
//    TSerial.print(F("Converting Data from: "));
//    TSerial.print(From);
//    TSerial.print(F(" to "));
//    TSerial.println(To);
//  }
//}

void ServoHAL::write(uint8_t ServoIdx, float percent){
//  ShowConversionDone(F("Float"),F("servoangle"));
  uint16_t Value;
  Value = GetServoNext(ServoIdx, percent);
  _ServoPos[ServoIdx]=Value;
//  if (ServoDebug==dbgServoWrite){
//    TSerial.print(F("Servo "));
//    TSerial.print(ServoIdx);
//    TSerial.print(F(": %="));
//    TSerial.print(percent,3);
//    TSerial.print(F(": #="));
//    TSerial.println((int)Value);
//  }
  setPWM(ServoIdx,Value);
}
void ServoHAL::write(uint8_t ServoIdx, int percent){
//  ShowConversionDone(F("int"),F("servoangle"));
  write(ServoIdx,percent*SERVOANGLEFACTOR);
}
void ServoHAL::write(uint8_t ServoIdx, servoangle_T percent){
  uint16_t Value;
  Value = GetServoNext(ServoIdx, percent);
  _ServoPos[ServoIdx]=Value;
  if (ServoDebug==dbgServoWrite){
    TSerial.print(F("Servo "));
    TSerial.print(ServoIdx);
    TSerial.print(F(": %="));
    TSerial.print(percent,3);
    TSerial.print(F(": #="));
    TSerial.println((int)Value);
  }
  setPWM(ServoIdx,Value);
}

void ServoHAL::setPWM(uint8_t ch, uint16_t Value){
  if (ServoDebug==dbgSetPWM) {
    Serial.write('S');
    TSerial.print(ch);
    Serial.write('=');
    TSerial.println(Value);
  }
  if (_Handlers[ch]!=NULL){
    switch (_ServoCal[ch].Mode){
      case SRV_MODE_PCA9685:((Adafruit_PWMServoDriver *)_Handlers[ch])->setPWM(ch,0,Value); break;
      case SRV_MODE_SERVO:Servo *s=((Servo*)_Handlers[ch]);
                          if (!s->attached()) s->attach(_ServoCal[ch].Channel);
                          s->write(Value);break;
      default: TSerial.println(F("!!wrong Params"));
    }
    _ServoPos[ch]=Value;
  }else{
    TSerial.println(F("Servo Control Data invalid"));
  }
}
