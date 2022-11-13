#if SERV0_SPT_5435
const float jumpLenth = 30;
const float const_max_jump_high = 180;
#else if SERV0_SPT_5835
const float jumpLenth = 60;
const float const_max_jump_high = 170;
#endif

//跨越过程依然使用摆线方程
//跨越距离 = jumpLenth*2
//跨越高度 = 2*r
//摆线方程 x = r(theta-sin(theta))
//        y = r(1-cos(theta))
//        theta = 2*pi 时跨越结束，这时x = jumpLenth*2
//得出            jumpLenth*2 = r*2*pi
//摆线方程半径      r = jumpLenth/pi
        

//运行时变量
int jump_last_time = 0;
int jump_cur_time = 0;//当前所在周期的时间
int jump_cycle_time = 200;//跳跃周期
float jump_start_time = jump_cycle_time*1/5;//起跳时间 为 2/5
float jump_crossing_time = jump_cycle_time*3/5; // 1/5 -> 3/5 为跨越时间，剩下2/5为手收腿缓冲时间
float jump_cycloid_r = jumpLenth/M_PI;
int jump_direction = 0;
float startJumpHigh = 0;
bool bJumping = false;

int backflip_cycle_time = 200;//后空翻周期
bool isJumping()
{
  return bJumping;
}
void start_jump(int d)
{
  bJumping = true;
  jump_last_time = millis();
  jump_direction =1;
  servoThreadMode = 2;
  jump_direction = d;
  if(isSit())
    startJumpHigh = const_sit_high;
  else //if(isStand())
    startJumpHigh = getStepStandHigh();
  Serial.println("start_jump");
}
void check_cycle_jump()
{//0:原地跳，1：前跳， 2：后跳，3：左跳，4：右跳
  jump_cur_time = millis();
  int period = jump_cur_time - jump_last_time;
  float rate,x,y,z;

  if(period > jump_cycle_time)
  {
     actionDirectTo(0,0,getStepStandHigh());
     bJumping = false;
//     Serial.println("check_cycle_jump end");
     return;
  }
//  Serial.print("check_cycle_jump ");
//  Serial.println(period);
  if(period <= jump_start_time)//起跳过程
  {
    //x 0->jumpLenth
    //z 70->const_max_jump_high
    rate = period/jump_start_time;
    x = jumpLenth*rate;
    z = (const_max_jump_high-startJumpHigh)*rate+startJumpHigh;
    
  }
  else if(period>jump_start_time && period <= jump_crossing_time)//迈腿过程
  {
    rate = (period-jump_start_time)/(jump_crossing_time-jump_start_time);
    rate *= 2*M_PI;//转成弧度
    x = jumpLenth - jump_cycloid_r*(rate-sin(rate));
//    if(x<0) x = 0;
    z = const_max_jump_high - jump_cycloid_r*(1-cos(rate));

  }
  else//period <= jump_cycle_time 收腿，缓冲，回到站立的过程
  {
    rate = (period - jump_crossing_time)/(jump_cycle_time-jump_crossing_time);
    x = -jumpLenth + jumpLenth*rate;
//    if(x<0) x = 0;
    z = const_max_jump_high - (const_max_jump_high-getStepStandHigh())*rate;

  }
  if(jump_direction == 0){
    x = 0;
    y = 0;
    direct_all_postion(x,y,z);
  }
  else if(jump_direction == 1){
    y = 0;
    direct_all_postion(x,y,z);
  }
  else if(jump_direction == 2){
    x = -x;
    y = 0;
    direct_all_postion(x,y,z);
  }
  else if(jump_direction == 3){
    y = x;
    direct_postion(0,0,y,z);
    direct_postion(2,0,y,z);
    y = -x;
    direct_postion(1,0,y,z);
    direct_postion(3,0,y,z);    
  }
  else if(jump_direction == 4){
    y = -x;
    direct_postion(0,0,y,z);
    direct_postion(2,0,y,z);
    y = x;
    direct_postion(1,0,y,z);
    direct_postion(3,0,y,z);      
  }  
}

void start_backflip()
{
  servoThreadMode = 0;
  //step 1
  setActionSpeed(1);
  set_expect_postion_all(50,0,100);
  wait_all_reach_postion(); 
  
  servoThreadMode = 4;
  jump_last_time = millis();
  bJumping = true;
}
void check_cycle_backflip()
{
  jump_cur_time = millis();
  int period = jump_cur_time - jump_last_time;
  float rate,x,y,z;  
  rate = 3;
  if(period <= 20*rate)
  {//step 2
    direct_postion_front(-50,0,70);
    direct_postion_rear(-50,0,100);
  }
  else if(period <= 40*rate)
  {//step 3
    direct_postion_front(-100,0,100);
    direct_postion_rear(-100,0,120);
  }
  else if(period <= 60*rate)
  {//step 4
    direct_postion_front(-20,0,160);
    direct_postion_rear(-50,0,150);
  }  
  else if(period <= 80*rate)
  {//step 5
    direct_postion_front(0,0,160);
    direct_postion_rear(100,0,90);
  } 
  else if(period <= 100*rate)
  {//step 6
    direct_postion_front(20,0,160);
    direct_postion_rear(100,0,90);
  }   
  else if(period <= 120*rate)
  {//step 7
    direct_postion_front(50,0,160);
    direct_postion_rear(70,0,150);
  }    
  else if(period <= 140*rate)
  {//step 8
    direct_all_postion(0,0,190);
  }
  else
  {
     servoThreadMode = 0;
     cmdStr = "stand";
     bJumping = false;
     return;
  }  
}
