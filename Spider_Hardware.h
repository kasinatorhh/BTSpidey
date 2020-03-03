/*************************************************** 
  This is the Code for Arduino Spider with PCA9685 PWM controller.
  3D Design Data and Buildup Guide is found here:
  https://www.thingiverse.com/thing:4070234
  The Spider uses pins see chapter pinmapping below
  
  BSD license, all text above must be included in any redistribution
 ****************************************************/
#ifndef SPIDER_HW_H
#define SPIDER_HW_H

/*********************************************************** 
 *  HW-I2C for communication to it's body-Servo driver PCA9685
 *     PCA9685 is assumed default address 0x40
 *  BT_TX, BT_RX, BT_EN, BT_STATE to HC-05 Bluetooth Module  
 *  BT_VDD1, BT_VDD2, BT_VDD3 to supply HC-05 Bluetooth Module    
 *    we use more than one IO pin to supply to prevent disconnect.
 *    when only one VDD is used, the code adapts.  
 *  HEADSERVOPIN to turn the head with Ultrasonic Ranger
 *  VIN_SENSE for a 1/3 divided Battery voltage monitoring
 *  SR_VDD, SR_TRIG and SR_ECHO for Ultrasonic ranging
*/
#define BT_TX 9
#define BT_RX 10
#define BT_EN PIN_A0
#define BT_STATE 12
#define BT_CONFIG HIGH
#define BT_COM LOW
#define BT_VDD1 4 /* Primary supply*/
#define BT_VDD2 2 /* Secondary supply, uncomment line when not used*/
#define BT_VDD3 3 /* Ternary supply, uncomment line when not used*/
#define HEADSERVOPIN 11
#define VIN_SENSE A6
#define SR_TRIG 13
#define SR_ECHO A2
#define SR_VDD A1
#define LED 13

#define NUMSERVOS 13
#define NUMLEGS 4
#define NUMAXIS 3
#define HEADSERVO 12

/* Size of the robot ---------------------------------------------------------*/
const float length_a = 55;//50; //Femur
const float length_b = 80;//77.1;//Tibia
const float length_c = 27.5;//Coxa
const float length_side = 93;//Body Servo Distance Left to Right
const float z_absolute = -32; //turning position of Tibia-servo above ground
/* Constants for movement ----------------------------------------------------*/
const float z_default = -50, z_up = -25, z_boot = z_absolute, z_transport = z_absolute+10;
const float x_default = 62, x_offset = 0;
const float y_start = 0, y_step = 40;
const float y_default = x_default;
#endif
