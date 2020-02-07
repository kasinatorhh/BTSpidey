/*************************************************** 
  This is the Code for Arduino Spider with PCA9685 PWM controller.
  3D Design Data and Buildup Guide is found here:
  https://www.thingiverse.com/thing:4070234
  The Spider uses pins see chapter pinmapping below
  
  BSD license, all text above must be included in any redistribution
 ****************************************************/
#ifndef SPIDER_SERVOS_H
#define SPIDER_SERVOS_H
#include <arduino.h>
//template< typename T, typename U > void print( T data, U modifier );
//template< typename T > void print( T data );
//template< typename T, typename U > void println( T data, U modifier );
//template< typename T > void println( T data );

#include <Servo.h>
#include <Adafruit_PWMServoDriver.h>
#include <EEPROM.h>
#include "TwoSerialWrap.h"
typedef long int servoangle_T;
#define SERVOANGLEFACTOR 100
//Structure to store servo related calibration data
typedef struct {
  uint16_t Channel;
  uint16_t Mode;
  uint16_t Min;
  uint16_t Max;
  uint16_t Ref0; //-90 degrees
  uint16_t Ref90; //  0 degrees
  uint16_t Ref180; // 90 degrees
} Servo_Cal_T;
enum SrvCal{
  SRV_CHANNEL,
  SRV_MODE,
  SRV_MIN,
  SRV_MAX,
  SRV_REF0,
  SRV_REF90,
  SRV_REF180,
  SRV_MAXCALENUM
};
enum ServoModes{
  SRV_MODE_INVALID=0xFFFF,
  SRV_MODE_PCA9685=1,
  SRV_MODE_SERVO=2
};
//This enum is used to debug individual functions. 
//to limit communication overhead, debugging only one at a time
enum DebugLevel{
  dbgServoConversions=9,
  dbgGetServoNextBody=8,
  dbgGetServoNextHead=7,
  dbgSetSite=6,
  dbgPolarToServo=5,
  dbgWaitReach=4,
  dbgPrintPos=3,
  dbgSetPWM=2,
  dbgServoWrite=1,
  dbgOff=0
};
extern DebugLevel ServoDebug; //set via command "D 1 <DebugLevelValue>\r\n"
#ifndef NUMSERVOS
#define NUMSERVOS 13
#endif

class ServoHAL{
  public:
//    ServoHAL();    
    int CalRestore(int Base);
    void CalBackup(int Base);
    void CalPrint(void);
    void CalSet(int ServoIdx, SrvCal Entity, uint16_t Value);
    uint16_t CalGet(int ServoIdx, SrvCal Entity);
    
    void SetHandler(int ServoIdx, Adafruit_PWMServoDriver * pwm, uint16_t channel);
    void SetHandler(int ServoIdx, Servo * Serv, uint16_t channel);

    uint16_t GetPath(int ServoIdx);
    void writeCount(uint8_t ServoIdx, int count);
    void write(uint8_t ServoIdx, int percent);
    void write(uint8_t ServoIdx, servoangle_T percent);
    void write(uint8_t ServoIdx, float percent);
    uint16_t GetPos(int ServoIdx);
    void ServoWrite(uint8_t ServoIdx, float percent);
    void PrintPCA9685();
    void PrintPos(void);
    void Detatch(int ServoIdx);
    void Attach(int ServoIdx);
    void InitServoParameters(int servoidx,uint16_t Channel, uint16_t Mode, uint16_t Min, uint16_t Max, uint16_t Ref0, uint16_t Ref90, uint16_t Ref180);
    const int ServoCalSize=sizeof(_ServoCal);
  private:
    void setPWM(uint8_t ch, uint16_t Value);
    uint16_t _Handlers[NUMSERVOS];
    uint16_t GetServoNext(uint8_t ServoIdx, float percent);
    #define SERVOMAGIC 0xE5E7
    uint16_t _ServoPos[NUMSERVOS];
    Servo_Cal_T _ServoCal[NUMSERVOS];
    void PrintCalName(SrvCal Item);
    void ShowConversionDone(const __FlashStringHelper* From,const __FlashStringHelper* To);
};

extern ServoHAL Srv;
#endif
