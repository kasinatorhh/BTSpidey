/*************************************************** 
  This is the Code for Arduino Spider with PCA9685 PWM controller.
  3D Design Data and Buildup Guide is found here:
  https://www.thingiverse.com/thing:4070234
  The Spider uses pins see chapter pinmapping below
  
  BSD license, all text above must be included in any redistribution
 ****************************************************/
#include "ServoHAL.h"
#include "MoveCore.h"
#include "Spider_Hardware.h"

MoveCore Mover;

//declared locally
void stand(void);
void sit(void);
void turn_left(unsigned int step);
void turn_right(unsigned int step);
void step_forward(unsigned int step);
void step_back(unsigned int step);
void body_left(int i);
void body_right(int i);
void hand_wave(int i);
void hand_shake(int i);
void head_up(int i);
void head_down(int i);

volatile float site_now[NUMLEGS][NUMAXIS];    //real-time coordinates of the end of each leg
volatile float site_expect[NUMLEGS][NUMAXIS]; //expected coordinates of the end of each leg
float temp_speed[NUMLEGS][NUMAXIS];   //each axis' speed, needs to be recalculated before each movement
float move_speed;     //movement speed  
float speed_multiple = 1; //movement speed multiple
float ServoNext[NUMSERVOS];
float ServoSpeed[NUMSERVOS];
float ServoDelta[NUMSERVOS];

void MoveCore::Move(Moves MOV, int Parameter){
  TSerial.print("Mover: ");
  TSerial.print(MOV);
  TSerial.print(' ');
  TSerial.println(Parameter);
  switch(MOV){
    case MOV_INIT:
      set_site(0, x_default - x_offset, y_start + y_step, z_boot);
      set_site(1, x_default - x_offset, y_start + y_step, z_boot);
      set_site(2, x_default + x_offset, y_start, z_boot);
      set_site(3, x_default + x_offset, y_start, z_boot);
      for (int i = 0; i < 4; i++)
      {
        for (int j = 0; j < 3; j++)
        {
          site_now[i][j] = site_expect[i][j];
        }
      };break;

      case MOV_STEPFORWARD: step_forward(1);break;
      case MOV_STEPBACK: step_back(1);break;
      case MOV_TURNLEFT: turn_left(1);break;
      case MOV_TURNRIGHT: turn_right(1);break;
      case MOV_STAND: stand();break;
      case MOV_SIT: sit();break;
      case MOV_HANDWAVE: hand_wave(Parameter);break;
      case MOV_HANDSHAKE: hand_shake(Parameter);break;
      case MOV_HEADUP: head_up(Parameter);break;
      case MOV_HEADDOWN: head_down(Parameter);break;
      case MOV_NO: No(Parameter);break;
  }
}
/********************************+
 * Section deals with Movements
 */

void MoveCore::No(uint8_t count){
  TSerial.print("No(");TSerial.print(count);TSerial.println(F(")"));
  long int p;
  int delta = (Srv.GetPath(HEADSERVO))/12;
  int waits = 20;
  while (count>0){
    for (p=Srv.GetPos(12);p>Srv.CalGet(HEADSERVO,SRV_MIN);p-=delta){
      Srv.write(HEADSERVO,p);
      delay(waits);
    }
    for (p=Srv.GetPos(12);p<Srv.CalGet(HEADSERVO,SRV_MAX);p+=delta){
      Srv.write(HEADSERVO,p);
      delay(waits);
    }
    count--;
  }
  for (p=Srv.GetPos(12);p>Srv.CalGet(HEADSERVO,SRV_REF90);p-=delta){
      Srv.write(HEADSERVO,p);
    delay(waits);
  }
  Srv.write(HEADSERVO,90);
  delay(100);
//  HeadServo.detach();
}

void MoveCore::sit(void)
{
  TSerial.println("sit()");
  move_speed = stand_seat_speed;
  for (int leg = 0; leg < 4; leg++)
  {
    set_site(leg, KEEP, KEEP, z_boot);
  }
  wait_all_reach();
}

/*
  - stand
  - blocking function
   ---------------------------------------------------------------------------*/
void MoveCore::stand(void)
{
  TSerial.println("stand()");
  move_speed = stand_seat_speed;
  for (int leg = 0; leg < 4; leg++)
  {
    set_site(leg, KEEP, KEEP, z_default);
  }
  wait_all_reach();
}


/*
  - spot turn to left
  - blocking function
  - parameter step steps wanted to turn
   ---------------------------------------------------------------------------*/
void MoveCore::turn_left(unsigned int step)
{
  TSerial.print("turn_left(");TSerial.print(step);TSerial.println(F(")"));
  move_speed = spot_turn_speed;
  while (step-- > 0)
  {
    if (site_now[3][1] == y_start)
    {
      //leg 3&1 move
      set_site(3, x_default + x_offset, y_start, z_up);
      wait_all_reach();

      set_site(0, turn_x1 - x_offset, turn_y1, z_default);
      set_site(1, turn_x0 - x_offset, turn_y0, z_default);
      set_site(2, turn_x1 + x_offset, turn_y1, z_default);
      set_site(3, turn_x0 + x_offset, turn_y0, z_up);
      wait_all_reach();

      set_site(3, turn_x0 + x_offset, turn_y0, z_default);
      wait_all_reach();

      set_site(0, turn_x1 + x_offset, turn_y1, z_default);
      set_site(1, turn_x0 + x_offset, turn_y0, z_default);
      set_site(2, turn_x1 - x_offset, turn_y1, z_default);
      set_site(3, turn_x0 - x_offset, turn_y0, z_default);
      wait_all_reach();

      set_site(1, turn_x0 + x_offset, turn_y0, z_up);
      wait_all_reach();

      set_site(0, x_default + x_offset, y_start, z_default);
      set_site(1, x_default + x_offset, y_start, z_up);
      set_site(2, x_default - x_offset, y_start + y_step, z_default);
      set_site(3, x_default - x_offset, y_start + y_step, z_default);
      wait_all_reach();

      set_site(1, x_default + x_offset, y_start, z_default);
      wait_all_reach();
    }
    else
    {
      //leg 0&2 move
      set_site(0, x_default + x_offset, y_start, z_up);
      wait_all_reach();

      set_site(0, turn_x0 + x_offset, turn_y0, z_up);
      set_site(1, turn_x1 + x_offset, turn_y1, z_default);
      set_site(2, turn_x0 - x_offset, turn_y0, z_default);
      set_site(3, turn_x1 - x_offset, turn_y1, z_default);
      wait_all_reach();

      set_site(0, turn_x0 + x_offset, turn_y0, z_default);
      wait_all_reach();

      set_site(0, turn_x0 - x_offset, turn_y0, z_default);
      set_site(1, turn_x1 - x_offset, turn_y1, z_default);
      set_site(2, turn_x0 + x_offset, turn_y0, z_default);
      set_site(3, turn_x1 + x_offset, turn_y1, z_default);
      wait_all_reach();

      set_site(2, turn_x0 + x_offset, turn_y0, z_up);
      wait_all_reach();

      set_site(0, x_default - x_offset, y_start + y_step, z_default);
      set_site(1, x_default - x_offset, y_start + y_step, z_default);
      set_site(2, x_default + x_offset, y_start, z_up);
      set_site(3, x_default + x_offset, y_start, z_default);
      wait_all_reach();

      set_site(2, x_default + x_offset, y_start, z_default);
      wait_all_reach();
    }
  }
}

/*
  - spot turn to right
  - blocking function
  - parameter step steps wanted to turn
   ---------------------------------------------------------------------------*/
void MoveCore::turn_right(unsigned int step)
{
  TSerial.print("turn_right(");TSerial.print(step);TSerial.println(F(")"));
  move_speed = spot_turn_speed;
  while (step-- > 0)
  {
    if (site_now[2][1] == y_start)
    {
      //leg 2&0 move
      set_site(2, x_default + x_offset, y_start, z_up);
      wait_all_reach();

      set_site(0, turn_x0 - x_offset, turn_y0, z_default);
      set_site(1, turn_x1 - x_offset, turn_y1, z_default);
      set_site(2, turn_x0 + x_offset, turn_y0, z_up);
      set_site(3, turn_x1 + x_offset, turn_y1, z_default);
      wait_all_reach();

      set_site(2, turn_x0 + x_offset, turn_y0, z_default);
      wait_all_reach();

      set_site(0, turn_x0 + x_offset, turn_y0, z_default);
      set_site(1, turn_x1 + x_offset, turn_y1, z_default);
      set_site(2, turn_x0 - x_offset, turn_y0, z_default);
      set_site(3, turn_x1 - x_offset, turn_y1, z_default);
      wait_all_reach();

      set_site(0, turn_x0 + x_offset, turn_y0, z_up);
      wait_all_reach();

      set_site(0, x_default + x_offset, y_start, z_up);
      set_site(1, x_default + x_offset, y_start, z_default);
      set_site(2, x_default - x_offset, y_start + y_step, z_default);
      set_site(3, x_default - x_offset, y_start + y_step, z_default);
      wait_all_reach();

      set_site(0, x_default + x_offset, y_start, z_default);
      wait_all_reach();
    }
    else
    {
      //leg 1&3 move
      set_site(1, x_default + x_offset, y_start, z_up);
      wait_all_reach();

      set_site(0, turn_x1 + x_offset, turn_y1, z_default);
      set_site(1, turn_x0 + x_offset, turn_y0, z_up);
      set_site(2, turn_x1 - x_offset, turn_y1, z_default);
      set_site(3, turn_x0 - x_offset, turn_y0, z_default);
      wait_all_reach();

      set_site(1, turn_x0 + x_offset, turn_y0, z_default);
      wait_all_reach();

      set_site(0, turn_x1 - x_offset, turn_y1, z_default);
      set_site(1, turn_x0 - x_offset, turn_y0, z_default);
      set_site(2, turn_x1 + x_offset, turn_y1, z_default);
      set_site(3, turn_x0 + x_offset, turn_y0, z_default);
      wait_all_reach();

      set_site(3, turn_x0 + x_offset, turn_y0, z_up);
      wait_all_reach();

      set_site(0, x_default - x_offset, y_start + y_step, z_default);
      set_site(1, x_default - x_offset, y_start + y_step, z_default);
      set_site(2, x_default + x_offset, y_start, z_default);
      set_site(3, x_default + x_offset, y_start, z_up);
      wait_all_reach();

      set_site(3, x_default + x_offset, y_start, z_default);
      wait_all_reach();
    }
  }
}

/*
  - go forward
  - blocking function
  - parameter step steps wanted to go
   ---------------------------------------------------------------------------*/
void MoveCore::step_forward(unsigned int step)
{
  TSerial.print("step_forward(");TSerial.print(step);TSerial.println(F(")"));
  move_speed = leg_move_speed;
  while (step-- > 0)
  {
    if (site_now[2][1] == y_start)
    {
      //leg 2&1 move
      set_site(2, x_default + x_offset, y_start, z_up);
      wait_all_reach();
      set_site(2, x_default + x_offset, y_start + 2 * y_step, z_up);
      wait_all_reach();
      set_site(2, x_default + x_offset, y_start + 2 * y_step, z_default);
      wait_all_reach();

      move_speed = body_move_speed;

      set_site(0, x_default + x_offset, y_start, z_default);
      set_site(1, x_default + x_offset, y_start + 2 * y_step, z_default);
      set_site(2, x_default - x_offset, y_start + y_step, z_default);
      set_site(3, x_default - x_offset, y_start + y_step, z_default);
      wait_all_reach();

      move_speed = leg_move_speed;

      set_site(1, x_default + x_offset, y_start + 2 * y_step, z_up);
      wait_all_reach();
      set_site(1, x_default + x_offset, y_start, z_up);
      wait_all_reach();
      set_site(1, x_default + x_offset, y_start, z_default);
      wait_all_reach();
    }
    else
    {
      //leg 0&3 move
      set_site(0, x_default + x_offset, y_start, z_up);
      wait_all_reach();
      set_site(0, x_default + x_offset, y_start + 2 * y_step, z_up);
      wait_all_reach();
      set_site(0, x_default + x_offset, y_start + 2 * y_step, z_default);
      wait_all_reach();

      move_speed = body_move_speed;

      set_site(0, x_default - x_offset, y_start + y_step, z_default);
      set_site(1, x_default - x_offset, y_start + y_step, z_default);
      set_site(2, x_default + x_offset, y_start, z_default);
      set_site(3, x_default + x_offset, y_start + 2 * y_step, z_default);
      wait_all_reach();

      move_speed = leg_move_speed;

      set_site(3, x_default + x_offset, y_start + 2 * y_step, z_up);
      wait_all_reach();
      set_site(3, x_default + x_offset, y_start, z_up);
      wait_all_reach();
      set_site(3, x_default + x_offset, y_start, z_default);
      wait_all_reach();
    }
  }
}

/*
  - go back
  - blocking function
  - parameter step steps wanted to go
   ---------------------------------------------------------------------------*/
void MoveCore::step_back(unsigned int step)
{
  TSerial.print("step_back(");TSerial.print(step);TSerial.println(F(")"));
  move_speed = leg_move_speed;
  while (step-- > 0)
  {
    if (site_now[3][1] == y_start)
    {
      //leg 3&0 move
      set_site(3, x_default + x_offset, y_start, z_up);
      wait_all_reach();
      set_site(3, x_default + x_offset, y_start + 2 * y_step, z_up);
      wait_all_reach();
      set_site(3, x_default + x_offset, y_start + 2 * y_step, z_default);
      wait_all_reach();

      move_speed = body_move_speed;

      set_site(0, x_default + x_offset, y_start + 2 * y_step, z_default);
      set_site(1, x_default + x_offset, y_start, z_default);
      set_site(2, x_default - x_offset, y_start + y_step, z_default);
      set_site(3, x_default - x_offset, y_start + y_step, z_default);
      wait_all_reach();

      move_speed = leg_move_speed;

      set_site(0, x_default + x_offset, y_start + 2 * y_step, z_up);
      wait_all_reach();
      set_site(0, x_default + x_offset, y_start, z_up);
      wait_all_reach();
      set_site(0, x_default + x_offset, y_start, z_default);
      wait_all_reach();
    }
    else
    {
      //leg 1&2 move
      set_site(1, x_default + x_offset, y_start, z_up);
      wait_all_reach();
      set_site(1, x_default + x_offset, y_start + 2 * y_step, z_up);
      wait_all_reach();
      set_site(1, x_default + x_offset, y_start + 2 * y_step, z_default);
      wait_all_reach();

      move_speed = body_move_speed;

      set_site(0, x_default - x_offset, y_start + y_step, z_default);
      set_site(1, x_default - x_offset, y_start + y_step, z_default);
      set_site(2, x_default + x_offset, y_start + 2 * y_step, z_default);
      set_site(3, x_default + x_offset, y_start, z_default);
      wait_all_reach();

      move_speed = leg_move_speed;

      set_site(2, x_default + x_offset, y_start + 2 * y_step, z_up);
      wait_all_reach();
      set_site(2, x_default + x_offset, y_start, z_up);
      wait_all_reach();
      set_site(2, x_default + x_offset, y_start, z_default);
      wait_all_reach();
    }
  }
}

// add by RegisHsu

void MoveCore::body_left(int i)
{
  TSerial.print("body_left(");TSerial.print(i);TSerial.println(F(")"));
  set_site(0, site_now[0][0] + i, KEEP, KEEP);
  set_site(1, site_now[1][0] + i, KEEP, KEEP);
  set_site(2, site_now[2][0] - i, KEEP, KEEP);
  set_site(3, site_now[3][0] - i, KEEP, KEEP);
  wait_all_reach();
}

void MoveCore::body_right(int i)
{
  TSerial.print("body_right(");TSerial.print(i);TSerial.println(F(")"));
  set_site(0, site_now[0][0] - i, KEEP, KEEP);
  set_site(1, site_now[1][0] - i, KEEP, KEEP);
  set_site(2, site_now[2][0] + i, KEEP, KEEP);
  set_site(3, site_now[3][0] + i, KEEP, KEEP);
  wait_all_reach();
}

void MoveCore::hand_wave(int i)
{
  TSerial.print("hand_wave(");TSerial.print(i);TSerial.println(F(")"));
  float x_tmp;
  float y_tmp;
  float z_tmp;
  move_speed = 1;
  if (site_now[3][1] == y_start)
  {
    body_right(15);
    x_tmp = site_now[2][0];
    y_tmp = site_now[2][1];
    z_tmp = site_now[2][2];
    move_speed = body_move_speed;
    for (int j = 0; j < i; j++)
    {
      set_site(2, turn_x1, turn_y1, 50);
      wait_all_reach();
      set_site(2, turn_x0, turn_y0, 50);
      wait_all_reach();
    }
    set_site(2, x_tmp, y_tmp, z_tmp);
    wait_all_reach();
    move_speed = 1;
    body_left(15);
  }
  else
  {
    body_left(15);
    x_tmp = site_now[0][0];
    y_tmp = site_now[0][1];
    z_tmp = site_now[0][2];
    move_speed = body_move_speed;
    for (int j = 0; j < i; j++)
    {
      set_site(0, turn_x1, turn_y1, 50);
      wait_all_reach();
      set_site(0, turn_x0, turn_y0, 50);
      wait_all_reach();
    }
    set_site(0, x_tmp, y_tmp, z_tmp);
    wait_all_reach();
    move_speed = 1;
    body_right(15);
  }
}

void MoveCore::hand_shake(int i)
{
  TSerial.print("hand_shake(");TSerial.print(i);TSerial.println(F(")"));
  float x_tmp;
  float y_tmp;
  float z_tmp;
  move_speed = 1;
  if (site_now[3][1] == y_start)
  {
    body_right(15);
    x_tmp = site_now[2][0];
    y_tmp = site_now[2][1];
    z_tmp = site_now[2][2];
    move_speed = body_move_speed;
    for (int j = 0; j < i; j++)
    {
      set_site(2, x_default - 30, y_start + 2 * y_step, 55);
      wait_all_reach();
      set_site(2, x_default - 30, y_start + 2 * y_step, 10);
      wait_all_reach();
    }
    set_site(2, x_tmp, y_tmp, z_tmp);
    wait_all_reach();
    move_speed = 1;
    body_left(15);
  }
  else
  {
    body_left(15);
    x_tmp = site_now[0][0];
    y_tmp = site_now[0][1];
    z_tmp = site_now[0][2];
    move_speed = body_move_speed;
    for (int j = 0; j < i; j++)
    {
      set_site(0, x_default - 30, y_start + 2 * y_step, 55);
      wait_all_reach();
      set_site(0, x_default - 30, y_start + 2 * y_step, 10);
      wait_all_reach();
    }
    set_site(0, x_tmp, y_tmp, z_tmp);
    wait_all_reach();
    move_speed = 1;
    body_right(15);
  }
}

void MoveCore::head_up(int i)
{ float speedBackup=move_speed;
  TSerial.print("head_up(");TSerial.print(i);TSerial.println(F(")"));
  move_speed = 0.5;
  set_site(0, KEEP, KEEP, site_now[0][2] - i);
  set_site(1, KEEP, KEEP, site_now[1][2] + i);
  set_site(2, KEEP, KEEP, site_now[2][2] - i);
  set_site(3, KEEP, KEEP, site_now[3][2] + i);
  wait_all_reach();
  move_speed = speedBackup;
}

void MoveCore::head_down(int i)
{
  TSerial.print("head_down(");TSerial.print(i);TSerial.println(F(")"));
  set_site(0, KEEP, KEEP, site_now[0][2] + i);
  set_site(1, KEEP, KEEP, site_now[1][2] - i);
  set_site(2, KEEP, KEEP, site_now[2][2] + i);
  set_site(3, KEEP, KEEP, site_now[3][2] - i);
  wait_all_reach();
}

void MoveCore::SetSpeedFactor(int Value){
  TSerial.print(F("Setting Speed to:"));
  speed_multiple = Value/(float)100;  
  TSerial.println(speed_multiple);
}

//void body_dance(int i)
//{
//  float x_tmp;
//  float y_tmp;
//  float z_tmp;
//  float body_dance_speed = 2;
//  sit();
//  move_speed = 1;
//  set_site(0, x_default, y_default, KEEP);
//  set_site(1, x_default, y_default, KEEP);
//  set_site(2, x_default, y_default, KEEP);
//  set_site(3, x_default, y_default, KEEP);
//  wait_all_reach();
//  //stand();
//  set_site(0, x_default, y_default, z_default - 20);
//  set_site(1, x_default, y_default, z_default - 20);
//  set_site(2, x_default, y_default, z_default - 20);
//  set_site(3, x_default, y_default, z_default - 20);
//  wait_all_reach();
//  move_speed = body_dance_speed;
//  head_up(30);
//  for (int j = 0; j < i; j++)
//  {
//    if (j > i / 4)
//      move_speed = body_dance_speed * 2;
//    if (j > i / 2)
//      move_speed = body_dance_speed * 3;
//    set_site(0, KEEP, y_default - 20, KEEP);
//    set_site(1, KEEP, y_default + 20, KEEP);
//    set_site(2, KEEP, y_default - 20, KEEP);
//    set_site(3, KEEP, y_default + 20, KEEP);
//    wait_all_reach();
//    set_site(0, KEEP, y_default + 20, KEEP);
//    set_site(1, KEEP, y_default - 20, KEEP);
//    set_site(2, KEEP, y_default + 20, KEEP);
//    set_site(3, KEEP, y_default - 20, KEEP);
//    wait_all_reach();
//  }
//  move_speed = body_dance_speed;
//  head_down(30);
//}
/*
  - microservos service /timer interrupt function/50Hz
  - when set site expected,this function move the end point to it in a straight line
  - temp_speed[4][3] should be set before set expect site,it make sure the end point
   move in a straight line,and decide move speed.
   ---------------------------------------------------------------------------*/
void MoveCore::servo_service(void)
{
  sei(); //don't waste time for other Interrupts
  static float alpha, beta, gamma;

  for (int i = 0; i < 4; i++)
  {
    for (int j = 0; j < 3; j++)
    {
      if (abs(site_now[i][j] - site_expect[i][j]) >= abs(temp_speed[i][j]))
        site_now[i][j] += temp_speed[i][j];
      else
        site_now[i][j] = site_expect[i][j];
    }

    cartesian_to_polar(alpha, beta, gamma, site_now[i][0], site_now[i][1], site_now[i][2]);
    if (ServoDebug==dbgPolarToServo){
      TSerial.print("ISRServos:");TSerial.print(i);TSerial.print(":");TSerial.print(alpha); TSerial.print(":"); TSerial.print(beta);TSerial.print(":"); TSerial.println(gamma);
    }

    //Direction Conversion was done already through calibration.
    beta = beta-90;
    Srv.write(ServoIdx(i,0), gamma);//coxa (Body-Side)
    Srv.write(ServoIdx(i,1), alpha);//Tibia (Middle)
    Srv.write(ServoIdx(i,2), beta);//Femur (Tip)
  }
}

/*
  - wait one of end points move to expect site
  - blocking function
   ---------------------------------------------------------------------------*/
void MoveCore::wait_reach(int leg)
{
  while (1)
    if (site_now[leg][0] == site_expect[leg][0])
      if (site_now[leg][1] == site_expect[leg][1])
        if (site_now[leg][2] == site_expect[leg][2])
          break;
}

/*
  - wait all of end points move to expect site
  - blocking function
   ---------------------------------------------------------------------------*/
void MoveCore::wait_all_reach(void)
{
  for (int i = 0; i < 4; i++)
    wait_reach(i);
}



/*
  - trans site from cartesian to polar
  - mathematical model 2/2
  - 1 ------ 3
  -   |Body|
  - 2 ------ 4
  -Coxa: Alpha
  -Femur: Beta, Femur-Coxy=0°-->90° Beta
  -Tibia: Gamma, Femur-Tibia=90°-->0° Gamma
   ---------------------------------------------------------------------------*/

void MoveCore::cartesian_to_polar(volatile float &alpha, volatile float &beta, volatile float &gamma, volatile float x, volatile float y, volatile float z)
{
  //Black art for now, Change to Integer arithmetics.s
  //define PI for calculation
  const float pi = 3.1415926;
  //calculate w-z degree
  float v, w;
  w = (x >= 0 ? 1 : -1) * (sqrt(pow(x, 2) + pow(y, 2)));
  v = w - length_c;    //          const              const             dyn       dyn            const               dyn         dyn
  alpha = atan2(z, v) + acos((pow(length_a, 2) - pow(length_b, 2) + pow(v, 2) + pow(z, 2)) / 2 / length_a / sqrt(pow(v, 2) + pow(z, 2)));
  beta = acos((pow(length_a, 2) + pow(length_b, 2) - pow(v, 2) - pow(z, 2)) / 2 / length_a / length_b);
  //calculate x-y-z degree
  gamma = (w >= 0) ? atan2(y, x) : atan2(-y, -x);
  //trans degree pi->180
  alpha = alpha / pi * 180;
  beta = beta / pi * 180;
  gamma = gamma / pi * 180;
}

/*
  - set one of end points' expect site
  - this founction will set temp_speed[4][3] at same time
  - non - blocking function
   ---------------------------------------------------------------------------*/
void MoveCore::set_site(int leg, float x, float y, float z)
{
//  if (ServoDebug==dbgSetSite){
//    TSerial.print("SetSite: Leg=");
//    TSerial.print(leg);
//    TSerial.print(" x=");
//    TSerial.print(x);
//    TSerial.print(" y=");
//    TSerial.print(y);
//    TSerial.print(" z=");
//    TSerial.print(z);
//    TSerial.println(';');
//  }
  float length_x = 0, length_y = 0, length_z = 0;

  if (x != KEEP)
    length_x = x - site_now[leg][0];
  if (y != KEEP)
    length_y = y - site_now[leg][1];
  if (z != KEEP)
    length_z = z - site_now[leg][2];

  float length = sqrt(pow(length_x, 2) + pow(length_y, 2) + pow(length_z, 2));

  //make sure we end up in all directions at final position
  temp_speed[leg][0] = length_x / length * move_speed * speed_multiple;
  temp_speed[leg][1] = length_y / length * move_speed * speed_multiple;
  temp_speed[leg][2] = length_z / length * move_speed * speed_multiple;

  if (x != KEEP)
    site_expect[leg][0] = x;
  if (y != KEEP)
    site_expect[leg][1] = y;
  if (z != KEEP)
    site_expect[leg][2] = z;
}
