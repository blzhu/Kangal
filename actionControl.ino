//const float jumpLenth = 50;
//const float const_max_jump_high = 170;
float body_theta = atan(const_body_width/const_body_length);//身体的对角线夹角,也是yaw角的最大值
float body_theta_len = sqrt(pow(const_body_width,2)+pow(const_body_length,2));//对角线长度
volatile float tempPostion[4][3];
void standTest()
{
  servoThreadMode = 10;
  for(int i = 0 ;i<4;i++)
  {
    for(int j=0;j<3;j++)
    {
      if(j == 2)
        setAngle(Legs_pin[i][j],90+Legs_offset[i][j]);//所有腿统一初始值为90度
      else{
        if(i==0||i==1)
        {
           setAngle(Legs_pin[i][j],90+45+Legs_offset[i][j]);
        }
        else
        {
          setAngle(Legs_pin[i][j],90-45+Legs_offset[i][j]);
        }
      }
      delay(100);
    }
  }
}

//void initAdjust()
//{
//  for(int i = 0 ;i<4;i++)
//  {
//    for(int j=0;j<3;j++)
//    {
//      setAngle(Legs_pin[i][j],90);//所有腿统一初始值为90度
//    }
//  }
//}

void initAllLegs()
{

  for(int i=0;i<4;i++)
  {
    initLeg(i);
  }
}
void initLeg(int nLeg)
{
//  servoThreadMode = 10;
    for(int j=0;j<3;j++)
    {
#if SERVO_180_HAM
      setAngle(Legs_pin[nLeg][j],90+Legs_offset[nLeg][j]);//所有腿统一初始值为90度
#else//SERVO_300_HAM
      if(j==0)
        setAngle300(Legs_pin[nLeg][j],150+Legs_offset[nLeg][j]);//大腿使用300度舵机，设中位为150度
      else
        setAngle(Legs_pin[nLeg][j],90+Legs_offset[nLeg][j]);//其他腿统一初始值为90度
#endif   
      delay(100);
    } 
}
void initAllLegsZero()
{
  for(int i=0;i<4;i++)
  {
    initLegZero(i);
  }
}
void initLegZero(int nLeg)
{
//  servoThreadMode = 10;
    for(int j=0;j<3;j++)
    {
#if SERVO_180_HAM
      setAngle(Legs_pin[nLeg][j],90);//所有腿统一初始值为90度
#else//SERVO_300_HAM
      if(j==0)
        setAngle300(Legs_pin[nLeg][j],150);//大腿使用300度舵机，设中位为150度
      else
        setAngle(Legs_pin[nLeg][j],90);//其他腿统一初始值为90度
#endif   
      delay(100);
    } 
}
void adjustLeg(int nLeg,int servoNo,bool bAddDegree)
{
  if(bAddDegree)
    Legs_offset[nLeg][servoNo]++;
  else
    Legs_offset[nLeg][servoNo]--;
//  String str = "Legs_offset ["+String(nLeg)+"]["+String(servoNo)+"]="+String(Legs_offset[nLeg][servoNo]);
//  Serial.println(str);
  setAngle(Legs_pin[nLeg][servoNo],90+Legs_offset[nLeg][servoNo]); 
//  actionPlay();
}

void adjustLegOffset(int nLeg,int servoNo,int offset,int bInit)
{
  if(nLeg<0 || nLeg>3)return;
  if(servoNo<0 || servoNo>2)return;
  Legs_offset[nLeg][servoNo] = offset;
  if(bInit)
  {
#if SERVO_180_HAM
    setAngle(Legs_pin[nLeg][servoNo],90+Legs_offset[nLeg][servoNo]); 
#else
    if(servoNo==0)
      setAngle300(Legs_pin[nLeg][servoNo],150+Legs_offset[nLeg][servoNo]); 
    else
      setAngle(Legs_pin[nLeg][servoNo],90+Legs_offset[nLeg][servoNo]); 
#endif
  }
}

void adjustMove(bool bAddMove)
{
  if(bAddMove)
    moveOffsetX++;
  else
    moveOffsetX--;
  Serial.print("moveOffsetX = ");
  Serial.println(moveOffsetX);

}

//void testAllServo()
//{
//  int angle = 45;
//  int direction = 0;//0表示++ ,1表示--
//  int count = 180;
//  while(count-->0){
//    for(int i = 0 ;i<16;i++)
//    {
////      for(int j=0;j<3;j++)
//      {
//        setAngle(i,angle);//所有腿统一初始值为90度
//      }
//    }
//    if(angle>=135)
//      direction = 1;
//    else if(angle<=45)
//      direction = 0;
//
//    if(direction == 0)
//      angle++;
//    else
//      angle--;
//    vTaskDelay(20);    
//  }
//
//}

void actionPlay()
{
  servoThreadMode = 4;
  direct_all_postion(0,0,getStepStandHigh());
}

void actionSit()
{
  Serial.println("actionSit");
  servoThreadMode = 0;
  set_expect_postion_all(0,0,const_sit_high);
}

void actionStand()
{
  Serial.println("actionStand");
  attachServo();
  servoThreadMode = 0;
  set_expect_postion_all(0,0,getStepStandHigh());
}

void actionLie()
{
  Serial.println("actionLie");
  attachServo();
  servoThreadMode = 0;
  set_expect_postion_all(50,0,30);
}

void actionWait()
{
  Serial.println("actionWait");
  attachServo();
  servoThreadMode = 0;
  set_expect_postion(0,100,0,180);
  set_expect_postion(2,100,0,180);
  set_expect_postion(1,20,0,60);
  set_expect_postion(3,20,0,60);  
}
void actionBainian()
{
  set_expect_postion(0,30,0,130);
  set_expect_postion(2,30,0,130);
  set_expect_postion(1,-80,0,110);
  set_expect_postion(3,-80,0,110);  
  wait_all_reach_postion();  
  setActionSpeed(4);
  set_expect_postion(0,-100,0,180);
  set_expect_postion(2,-100,0,180);
  set_expect_postion(1,0,0,160);
  set_expect_postion(3,0,0,160);
  wait_all_reach_postion(); 
  setActionSpeed(1);
  set_expect_postion(0,-100,80,180);
  set_expect_postion(2,-100,-80,180);
  set_expect_postion(1,20,0,160);
  set_expect_postion(3,20,0,160);  
  wait_all_reach_postion(); 
  setActionSpeed(3);
  for(int i = 0;i<3;i++)
  {
    set_expect_postion(0,-100,80,180);
    set_expect_postion(2,-100,-80,180);  
    wait_all_reach_postion();   
    set_expect_postion(0,0,80,180);
    set_expect_postion(2,0,-80,180);  
    wait_all_reach_postion();    
  }
  set_expect_postion(0,-100,80,180);
  set_expect_postion(2,-100,-80,180);  
  wait_all_reach_postion(); 
  setActionSpeed(1);
}
void actionDirectTo(float x,float y,float z)
{
  servoThreadMode = 0;
  direct_all_postion(x,y,z);
}

void actionLeft()
{
  servoThreadMode = 0;
    set_expect_postion_all(0,20,getStepStandHigh());
}
void actionRight()
{
  servoThreadMode = 0;
    set_expect_postion_all(0,-20,getStepStandHigh());
}

void actionBackflip()
{//动作分解参考 https://zhuanlan.zhihu.com/p/87052862
  servoThreadMode = 0;
  //step 1
  setActionSpeed(1);
  set_expect_postion_all(50,0,100);
  wait_all_reach_postion(); 
  
  //step 2  
  setActionSpeed(3);
  set_expect_postion_front(-50,0,70);
  set_expect_postion_rear(-50,0,100);
  wait_all_reach_postion(); 
  
  //step 3
  setActionSpeed(60);
  set_expect_postion_front(-100,0,100);
  set_expect_postion_rear(-100,0,120);
  wait_all_reach_postion(); 

  //step 4
  set_expect_postion_front(-20,0,160);
  set_expect_postion_rear(-50,0,150);  
  wait_all_reach_postion(); 

  //step 5
  set_expect_postion_front(0,0,160);
  set_expect_postion_rear(100,0,90);
  wait_all_reach_postion(); 

  //step 6
  set_expect_postion_front(20,0,160);
  set_expect_postion_rear(100,0,90);
  wait_all_reach_postion();   

  //step 7
  set_expect_postion_front(50,0,160);
  set_expect_postion_rear(70,0,150);
  wait_all_reach_postion();    

  //step 8
  set_expect_postion_all(0,0,190);
  wait_all_reach_postion();  

  //step 8
  setActionSpeed(1);
  set_expect_postion_all(0,0,150);
  wait_all_reach_postion();     
}
//
//void actionJump(int direction,bool bSit)
//{//direction  0:原地跳，1：前跳， 2：后跳，3：左跳，4：右跳 , bSit  0:蹲下起跳，1 站立态起跳
//
//  float x, y, z;
//  x = y = 0;
//  z = const_max_jump_high;
//  switch(direction)
//  {
//    case 0:
//    {
//      break;
//    }
//    case 1:
//    {
//      x = jumpLenth;
//      break;
//    }
//    case 2:
//    {
//      x=-jumpLenth;
//      break;
//    }
//    case 3:
//    {
//      y = jumpLenth;
//      break;
//    }
//    case 4:
//    {
//      y = -jumpLenth;
//      break;
//    }
//    default:
//      break;
//  }
//
//  actionDirectTo(x,y,z);
//  vTaskDelay(50);//起跳过程负载大，延时长一点
//  actionDirectTo(0,0,const_sit_high);
//  vTaskDelay(50);//抬腿时间
//  actionDirectTo(0,0,getStepStandHigh());
//}

void actionRotate(float rAngleYaw,float rAngleRoll,float rAnglePitch,float moveX,float moveY,float moveZ)
{//三维方向转动，解算
 //yaw<0 左偏转， roll<0左滚动，pitch<0 前倾，前低后高
// const float const_body_width = 160;
// const float const_body_length = 190;
//  Serial.printf("actionRotate yaw = %f,roll = %f,pitch = %f,movex = %f\n",rAngleYaw,rAngleRoll,rAnglePitch,moveX);
  servoThreadMode = 0;
  if(rAngleYaw>max_yaw)rAngleYaw=max_yaw;
  if(rAngleYaw<-max_yaw)rAngleYaw=-max_yaw;
  if(rAngleRoll>max_roll)rAngleRoll=max_roll;
  if(rAngleRoll<-max_roll)rAngleRoll=-max_roll;
  if(rAnglePitch>max_pitch)rAnglePitch=max_pitch;
  if(rAnglePitch<-max_pitch)rAnglePitch=-max_pitch;      
  body_attitude[0] = rAngleYaw;
  body_attitude[1] = rAngleRoll;
  body_attitude[2] = rAnglePitch;
  body_attitude[3] = moveX;
  body_attitude[4] = moveZ;

  float yaw = abs( angle2pi(rAngleYaw));
  float roll = abs(angle2pi(rAngleRoll));
  float pitch = abs(angle2pi(rAnglePitch));

//  float dxYaw,dyYaw,dyRoll,dzRoll,dxPitch,dzPitch;
//  dxYaw = dyYaw = dyRoll = dzRoll = dxPitch = dzPitch = 0;
  float a,x,y,z;
  float dx1_yaw,dx2_yaw,dy1_yaw,dy2_yaw;
  float dy1_roll,dy2_roll,dz1_roll,dz2_roll;
  float dx1_pitch,dx2_pitch,dz1_pitch,dz2_pitch;
  dx1_yaw = dx2_yaw = dy1_yaw = dy2_yaw = 0;
  dy1_roll = dy2_roll = dz1_roll = dz2_roll = 0;
  dx1_pitch = dx2_pitch = dz1_pitch = dz2_pitch=0;
  x = y = 0;
  z = getStepStandHigh();
  resetTempPostion();
  //先计算Yaw偏转，默认计算 rAngleYaw<0 的情况 left
  if(rAngleYaw != 0){
    a = const_body_width/2*tan(yaw/2);
    dx1_yaw = abs((const_body_length/2+a) - (const_body_length/2-a)*cos(yaw));
    dy1_yaw = (const_body_length/2-a)*sin(yaw);
    dx2_yaw = abs((const_body_length/2+a)*cos(yaw)-(const_body_length/2-a));
    dy2_yaw = (const_body_length/2+a)*sin(yaw);

  }

  //再计算Roll偏转，默认计算 rAngleRoll<0 的情况 
  if(rAngleRoll !=0 ){
    float z1_roll,z2_roll;
    a = const_body_width/2*tan(roll/2);//用半角去求解
    dy1_roll = (z-a)*sin(roll);
    dy2_roll = (z+a)*sin(roll);
    
    z1_roll = (z-a)*cos(roll)-a;
    z2_roll = (z+a)*cos(roll)+a;
    dz1_roll = z1_roll-z;
    dz2_roll = z2_roll-z;
  }

  //再计算Pitch偏转，默认计算 rAnglePich<0 的情况
  if(rAnglePitch != 0){
    float z1_pitch,z2_pitch;
    a = const_body_length/2*tan(pitch/2);//用半角去求解
    dx1_pitch = (z-a)*sin(pitch);
    dx2_pitch = (z+a)*sin(pitch);   
    
    z1_pitch = (z-a)*cos(pitch)-a;
    z2_pitch = (z+a)*cos(pitch)+a;
    dz1_pitch = z1_pitch -z;
    dz2_pitch = z2_pitch -z;
  }
   
  for(int i = 0;i<4;i++)
  {
    x = 0;//getCurPostionX(i);
    y = 0;//getCurPostionY(i);
    z = getStepStandHigh();//getCurPostionZ(i);

    //yaw
    if(rAngleYaw<0)
    {//left
      if(i==0)
      {
          x = x-dx2_yaw;
          y = y+dy2_yaw;
      }
      else if(i == 1)
      {
          x = x-dx1_yaw;
          y = y-dy1_yaw;
      }
      else if(i == 2)
      {    
          x = x+dx1_yaw;
          y = y+dy1_yaw;  
      }
      else
      {
          x = x+dx2_yaw;
          y = y-dy2_yaw;
      }      
    }
    else//right
    {
      if(i==0)
      {
          x = x+dx1_yaw;
          y = y-dy1_yaw;
      }
      else if(i == 1)
      {
          x = x+dx2_yaw;
          y = y+dy2_yaw;
      }
      else if(i == 2)
      {    
          x = x-dx2_yaw;
          y = y-dy2_yaw;  
      }
      else
      {
          x = x-dx1_yaw;
          y = y+dy1_yaw;
      }       
    }

    //roll
    if(rAngleRoll<0)//左
    {
      if(i == 0 || i == 1)
      {
        y = y-dy1_roll;
        z = z+dz1_roll;
      }
      else{
        y = y-dy2_roll;
        z = z+dz2_roll; 
      }     
    }
    else
    {
      if(i == 0 || i == 1)
      {
        y = y+dy2_roll;
        z = z+dz2_roll;
      }
      else{
        y = y+dy1_roll;
        z = z+dz1_roll; 
      }       
    }

    //pitch    
    if(rAnglePitch<0){//低头
      if(i == 0 || i == 2){
        x = x-dx1_pitch;
        z = z+dz1_pitch;
      }
      else{
        x = x-dx2_pitch;
        z = z+dz2_pitch;
      }
    }
    else{//抬头
      if(i == 0 || i == 2){
        x = x+dx2_pitch;
        z = z+dz2_pitch;
      }
      else{
        x = x+dx1_pitch;
        z = z+dz1_pitch;
      }     
    }

    //伸头,侧移，高低
    x = x + moveX;
    y = y + moveY;
    z = z + moveZ; 
//    direct_postion(i,x,y,z);   
    set_temp_postion(i,x,y,z);
//    Serial.printf("actionRotate x = %f,y = %f,z = %f\n",x,y,z);
  }
  adjust_temp_postion_to_adapt_max_min();
  direct_temp_postion();
}

void resetTempPostion()
{
  for(int i=0;i<4;i++)
    for(int j = 0;j<3;j++)
      tempPostion[i][j] = 0; 
}
void set_temp_postion(int nLeg,float x,float y,float z)
{
  tempPostion[nLeg][0]=x;
  tempPostion[nLeg][1]=y;
  tempPostion[nLeg][2]=z;
}

void direct_temp_postion()
{
  float x,y,z;
  for(int i=0;i<4;i++)
  {
    x = tempPostion[i][0];
    y = tempPostion[i][1];
    z = tempPostion[i][2];
    direct_postion(i,x,y,z); 
//    Serial.printf("direct_temp_postion x = %f,y = %f,z = %f\n",x,y,z);
  }
}

int verify_leg_postion(float x,float y,float z)
{
  float legLen = 0;
  int offset = 0;
  legLen = sqrt(pow(x,2)+pow(y,2)+pow(z,2));
  if(legLen > const_max_stand_high)
  {
    offset = legLen-const_max_stand_high;
    Serial.printf("verify failed > max, offset = %d\n",offset);
  }
  else if(legLen<const_sit_high){
    offset = legLen-const_sit_high;
    Serial.printf("verify failed < sit, offset = %d\n",offset);
  }

  return offset;
}

void adjust_temp_postion_to_adapt_max_min()
{
  float x,y,z,dz;
  int ret = 0;
  dz = 0;
 
  for(int i = 0;i<4;i++)
  {
    x = tempPostion[i][0];
    y = tempPostion[i][1];
    z = tempPostion[i][2];
    ret = verify_leg_postion(x,y,z);
    if(ret == 0)
      continue;
    if(ret > 0)
      dz-=ret;
    else
      dz+=ret;
    break;      
  }
  if(dz != 0)
  {
    for(int i = 0;i<4;i++)
      tempPostion[i][2]+=dz;
//    Serial.printf("adjust z = %f\n",tempPostion[0][2]);
  }

  
}
