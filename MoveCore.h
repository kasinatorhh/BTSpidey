/*************************************************** 
  This is the Code for Arduino Spider with PCA9685 PWM controller.
  3D Design Data and Buildup Guide is found here:
  https://www.thingiverse.com/thing:4070234
  The Spider uses pins see chapter pinmapping below
  
  BSD license, all text above must be included in any redistribution
 ****************************************************/
#ifndef SPIDER_MOVES_H
#define SPIDER_MOVES_H
#include "Spider_Hardware.h"
enum Moves{
  MOV_INIT='I',
  MOV_STAND='O', //start !=0 to allow 0 being Abort-Move in String
  MOV_SIT='o',
  MOV_TURNLEFT='l',
  MOV_TURNRIGHT='m',
  MOV_STEPFORWARD='f',
  MOV_STEPBACK='p',
  MOV_BODY_LEFT='1',
  MOV_MODY_RIGHT='2',
  MOV_HANDWAVE='3',
  MOV_HANDSHAKE='4',
  MOV_HEADUP='5',
  MOV_HEADDOWN='6',
  MOV_NO='7',
  MOV_LASTMOV=0
};
/* Constants for turn --------------------------------------------------------*/
//temp length
const float temp_a = sqrt(pow(2 * x_default + length_side, 2) + pow(y_step, 2));
const float temp_b = 2 * (y_start + y_step) + length_side;
const float temp_c = sqrt(pow(2 * x_default + length_side, 2) + pow(2 * y_start + y_step + length_side, 2));
const float temp_alpha = acos((pow(temp_a, 2) + pow(temp_b, 2) - pow(temp_c, 2)) / 2 / temp_a / temp_b);
//site for turn
const float turn_x1 = (temp_a - length_side) / 2;
const float turn_y1 = y_start + y_step / 2;
const float turn_x0 = turn_x1 - temp_b * cos(temp_alpha);
const float turn_y0 = temp_b * sin(temp_alpha) - turn_y1 - length_side;

  //Todo: change all this to Integer-math (not float)
  //we end up with counts between 100 and 600, so we should not waste
  //too much time for unused precision
  static const float spot_turn_speed = 4;
  static const float leg_move_speed = 8;
  static const float body_move_speed = 3;
  static const float stand_seat_speed = 1;
  /* variables for movement ----------------------------------------------------*/
extern volatile float site_now[NUMLEGS][NUMAXIS];    //real-time coordinates of the end of each leg
extern volatile float site_expect[NUMLEGS][NUMAXIS]; //expected coordinates of the end of each leg
extern float temp_speed[NUMLEGS][NUMAXIS];   //each axis' speed, needs to be recalculated before each movement
extern float move_speed;     //movement speed  
extern float speed_multiple; //movement speed multiple
extern float ServoNext[NUMSERVOS];
extern float ServoSpeed[NUMSERVOS];
extern float ServoDelta[NUMSERVOS];




class MoveCore{
  public:
  //functions' parameter
  static const float KEEP = 255;
  static void Move(Moves MOV, int Parameter);
  static void SetSpeedFactor(int value);
  static void set_site(int leg, float x, float y, float z);
#define NUMLEGS 4
#define NUMAXIS 3
  static void servo_service(void);  
  static uint8_t ServoIdx(uint8_t Leg, uint8_t axis){
    return Leg*3+axis;
  }
  private:
  /***********************
   * Section does the Math work
   */

  static void wait_all_reach(void);

//here the Moves declaration, having access to the variables
  static void No(uint8_t count);//added by Kasinator
  static void sit(void);
  static void stand(void);
  static void turn_left(unsigned int step);
  static void turn_right(unsigned int step);
  static void step_forward(unsigned int step);
  static void step_back(unsigned int step);
// add by RegisHsu
  static void body_left(int i);
  static void body_right(int i);
  static void hand_wave(int i);
  static void hand_shake(int i);
  static void head_up(int i);
  static void head_down(int i);
//Math happens here:
  static void wait_reach(int leg);
  static void cartesian_to_polar(volatile float &alpha, volatile float &beta, volatile float &gamma, volatile float x, volatile float y, volatile float z);
  //     Head
  //Leg 1    3 Leg  Coxa(Alpha)   Thibia (Beta)  Femur (Gamma)
  //Leg 2    4 Leg
};
extern MoveCore Mover;
#endif
