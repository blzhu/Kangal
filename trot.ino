
//摆线方程
/*
 *      x = r(t-sin*t)
 *      z = r(1-cos*t)
 * 
 *      r为半径，为抬腿高度的一半，t为滚动角， 0 < t < 2*pi , 
 *      从原点启动的话t是0-2pi，我们是从站立方式启动，启动是在t=pi位置，就是摆线的中心点开始
 * 
 * 
 * 
 *      对于0，3脚     半个摆动相 ----- 支撑相 ---- 半个摆动相
 *      对于1，2脚     半个支持相 ----- 摆动相 ---- 半个支撑相
 * 
 * 
 * */
//hardcode 值
//#define STAND_HIGH 150
//#define SIT_HIGH 70
//#define LIFT_HIGH 40
//可以保存flash
int step_lift_high = LIFT_HIGH;//trot 步态抬腿高度
int step_cycle_time = 350;//300毫秒为一个周期，对应 2*PI
float step_rate = 0.5;// 摆动相 / 支撑相， 必须小于1 ， 可以取值(0.5 - 1)
float step_stand_high = STAND_HIGH;//站立高度
//float step_sit_high = 70;//蹲下高度
//运行时变量
int step_last_time = 0;
int step_cur_time = 0;//当前所在周期的时间
float step_cur_theta = 0;//当前时间所处的滚动角度
float step_cycloid_phase_time = (step_rate/(step_rate+1))*step_cycle_time;//摆动相占用的周期时间,对应2*PI
float step_support_phase_time = step_cycle_time - step_cycloid_phase_time;
float step_cycloid_phase_r = step_lift_high/2;//摆线半径
float step_length = step_cycloid_phase_r*2*M_PI;//x = r*(t-sin(t)),t=2*PI,x=r*(2*PI-0) = r*2*PI
float trot_step_length = 160;//这个值是限定真实trot的步长
float trot_step_rate = trot_step_length/step_length;//调整比例
//float step_deflection_len = getDeflectionLen(deflection_angle);//n度偏转角的偏转距离，调头时使用n的偏转
//float step_deflection_rate = step_deflection_len/step_length;//为了适应偏转距离，步长调整的比例
//float step_deflection_theta_l = getDeflectionThetaL(deflection_angle);//伸腿的角度,左偏转
//float step_deflection_theta_r = getDeflectionThetaR(deflection_angle);//伸腿的角度,右偏转
float deflection_angle = 0;//身体的偏转角度,一个调头周期的偏转角度，最大30度
float deflection_angle_new = 0;
float step_deflection_len_1 = 0;//一个调头周期内，n度偏转，左顶点移动距离
float step_deflection_len_2 = 0;//一个调头周期内，n度偏转，右顶点移动距离
float step_deflection_rate_1 = 0;//一个调头周期内，左顶点移动的距离 和步长的比值
float step_deflection_rate_2 = 0;//一个调头周期内，右顶点移动的距离 和步长的比值
float step_deflection_theta_1 = 0;//一个调头周期内，左前腿的偏转方向，同右后
float step_deflection_theta_2 = 0;//一个调头周期内，右前腿的偏转方向，同左后
float dx1_yaw,dy1_yaw,dx2_yaw,dy2_yaw;//左右顶点的偏转坐标
int zoomRate = 1000;
float step_direction = 0;//0 原地踏步，1 前进，-1后退
float step_direction_new = 0;//用于循环中
float step_direction_theta = 0;//方向偏转角，90 到 -90度，向左偏转为正,弧度制
float step_direction_theta_new = 0;//用于命令设定
float step_direction_left = 0;
float step_direction_right = 0;
float step_direction_left_new = 0;//左右要保持同符号，否则就会旋转
float step_direction_right_new = 0;
float step_direction_front = 0;
float step_direction_rear = 0;
float step_direction_front_new = 0;//前后设置不同幅度，在横行时可以实现绕圈横行
float step_direction_rear_new = 0;
float step_body_pitch = 0;//踏步和前后行走时身体的前后倾斜角，pitch<0 前倾，pitch>0 后倾
float step_body_roll = 0;//踏步时的左右倾斜角，roll<0 左倾斜，roll>0 右倾斜
float step_body_pitch_new = 0;
float dx_pitch,dz_pitch;
int start_trot = 0;//trot启动标志
int start_deflection = 0;//调头标志 -1左转，1右转
int start_deflection_new = 0;//调头标志 -1左转，1右转
float step_stand_dz = 0;//踏步的高度调节值，差值，可以正负
//===

float getStepStandHigh()
{
  return step_stand_high;
}

void setStepStandDelta(float h)
{
  step_stand_dz = h;
}
void setStepStandHigh(float h)
{
  step_stand_high = h;
}

void setStartDeflection(int start)//-1向左，1向右
{
  #if WITHOUT_NEW
  start_deflection = start;
  #else
  start_deflection_new = start;
  #endif
}
void startTrot()
{
  //先站立。再走动
  if(!isStand())
    return;
  step_last_time = millis();
  servoThreadMode = 1;
  start_trot = 1000;
  start_deflection = 0;
  if(new_cycle_time == 0)
    new_cycle_time = step_cycle_time;
  if(new_lift_high == 0)
    new_lift_high = step_lift_high;
  if(new_stand_high == 0)
    new_stand_high = step_stand_high;

  step_lift_high = LIFT_HIGH;
  step_direction_left_new = 0;
  step_direction_right_new = 0;
  step_direction_left = 0;
  step_direction_right = 0;  
  
  step_direction_front = 0;
  step_direction_rear = 0;  
  step_direction_front_new = 0;
  step_direction_rear_new = 0;  
   
  deflection_angle_new = 0;

  Serial.println("startTrot OK");
//  Serial.print("step_cycloid_phase_time = ");
//  Serial.println(step_cycloid_phase_time);
//  Serial.print("step_support_phase_time = ");
//  Serial.println(step_support_phase_time);
//  Serial.print("step_cycloid_phase_r = ");
//  Serial.println(step_cycloid_phase_r);
//  Serial.print("step_length = ");
//  Serial.println(step_length);
}

bool isTrotRunning()
{
  if(servoThreadMode == 1 &&start_trot)
    return true;
  return false; 
}

void setDeflectionAngle(float a)
{
  #if WITHOUT_NEW
  deflection_angle = a;
  #else
  deflection_angle_new = a;
  #endif
}
void setTrotDirection(float d)
{
  #if WITHOUT_NEW
  step_direction = d;
  step_direction_left = d;
  step_direction_right = d;
  step_direction_front = d;
  step_direction_rear = d;    
  #else
  step_direction_new = d;
  step_direction_left_new = d;
  step_direction_right_new = d;
  step_direction_front_new = d;
  step_direction_rear_new = d;  
  #endif
}

void setTrotDirectionFR(float fd,float rd)//前腿和后腿方向幅度设置
{
  #if WITHOUT_NEW
  step_direction_front = fd;
  step_direction_rear = rd;  
  #else
  step_direction_front_new = fd;
  step_direction_rear_new = rd;  
  #endif
}
void setTrotDirectionLR(float ld,float rd)//左腿和右右方向幅度设置
{
  #if WITHOUT_NEW
  step_direction_left = ld;
  step_direction_right = rd;  
  #else  
  step_direction_left_new = ld;
  step_direction_right_new = rd;
  #endif
}
void addTrotDirection(bool bAdd)
{//方向是左为正，上为正

  #if WITHOUT_NEW
  if(bAdd)
    step_direction+=0.1;
  else
    step_direction-=0.1;

  if(step_direction>1){
    step_direction = 1;
    BeepWarning = 5;
  }
  if(step_direction<-1){
    step_direction = -1;
    BeepWarning = 5;
  }
  if(step_direction == 0)
    BeepWarning = 2;
  step_direction_right = step_direction;
  step_direction_left = step_direction;
  #else
  if(bAdd)
    step_direction_new+=0.1;
  else
    step_direction_new-=0.1;
//  BeepWarning = 0;
  if(step_direction_new>1){
    step_direction_new = 1;
    BeepWarning = 5;
  }
  if(step_direction_new<-1){
    step_direction_new = -1;
    BeepWarning = 5;
  }
  if(step_direction_new == 0)
    BeepWarning = 2;
  step_direction_right_new = step_direction_new;
  step_direction_left_new = step_direction_new;
  #endif
}

float getDirection()
{
  float d = (step_direction_left+step_direction_right)/2;
  d = ((int)d*10)/10;//取1位小数
  return d;
}
void turnDirection(bool bLeft)//是行进中的偏转，不是调头
{
  #if WITHOUT_NEW
  if(bLeft){//左转 ，leftdirection 趋近 0
    if(step_direction>0)
    {
      step_direction_left -= 0.1;
      if(step_direction_left == 0)//要保持移动
        step_direction_left = 0.1;
    }
    else
    {
      step_direction_left += 0.1;
      if(step_direction_left == 0)
        step_direction_left = -0.1;
    }
  }
  else
  {
    if(step_direction>0)
    {
      step_direction_right -= 0.1;
      if(step_direction_right == 0)//要保持移动
        step_direction_right = 0.1;
    }
    else
    {
      step_direction_right += 0.1;
      if(step_direction_right == 0)
        step_direction_right = -0.1;
    }    
  }

  if(step_direction>0){//取绝对值大的值
    step_direction = step_direction_left>step_direction_right?step_direction_left:step_direction_right;
  }
  else
  {
    step_direction = step_direction_left<step_direction_right?step_direction_left:step_direction_right;    
  }
  #else
  if(bLeft){//左转 ，leftdirection 趋近 0
    if(step_direction>0)
    {
      step_direction_left_new -= 0.1;
      if(step_direction_left_new == 0)//要保持移动
        step_direction_left_new = 0.1;
    }
    else
    {
      step_direction_left_new += 0.1;
      if(step_direction_left_new == 0)
        step_direction_left_new = -0.1;
    }
  }
  else
  {
    if(step_direction>0)
    {
      step_direction_right_new -= 0.1;
      if(step_direction_right_new == 0)//要保持移动
        step_direction_right_new = 0.1;
    }
    else
    {
      step_direction_right_new += 0.1;
      if(step_direction_right_new == 0)
        step_direction_right_new = -0.1;
    }    
  }

  if(step_direction>0){//取绝对值大的值
    step_direction_new = step_direction_left_new>step_direction_right_new?step_direction_left_new:step_direction_right_new;
  }
  else
  {
    step_direction_new = step_direction_left_new<step_direction_right_new?step_direction_left_new:step_direction_right_new;    
  }
  #endif
}
void setTrotDirectionRate(int d)
{
  #if WITHOUT_NEW
  step_direction = d*step_direction;
  #else
  step_direction_new = d*step_direction;
  #endif
}

void setTrotTheta(float theta)
{//theta 方向与x轴夹角
  #if WITHOUT_NEW
  if(step_direction_theta!=theta)
    step_direction = 0;
  step_direction_theta = theta;  
  #else
  if(step_direction_theta_new!=theta)
    step_direction_new = 0;
  step_direction_theta_new = theta;  
  #endif
}

void setBodyPitch(float p)
{
  #if USE_COORDINATES_CONVERSION
  step_body_pitch = p;
  #else
  step_body_pitch_new = p;
  #endif
}

void setBodyRoll(float r)
{
  step_body_roll = r;
}
void stopTrot()
{
  start_trot = 0; 
  start_deflection = 0;
  step_direction = 0;
  step_direction_new = 0;
  step_direction_theta = 0;
  step_direction_theta_new = 0;
  step_direction_left = 0;
  step_direction_right = 0;  
  step_direction_left_new = 0;
  step_direction_right_new = 0;
  step_direction_front = 0;
  step_direction_rear = 0;  
  step_direction_front_new = 0;
  step_direction_rear_new = 0;  
  deflection_angle = 0;

  Serial.println("stopTrot OK"); 
}
void adjust_stand_high()
{
  if(step_stand_dz>0)//降低高度
  {
    if((STAND_HIGH - step_stand_dz - LIFT_HIGH)>=SIT_HIGH){//小幅调节，lift high 不变
      step_stand_high = STAND_HIGH - step_stand_dz;
      step_lift_high = LIFT_HIGH;
    }
    else if((STAND_HIGH - step_stand_dz)<= SIT_HIGH){//调节幅度太大，不让其调节，保持最后的值
      BeepWarning = 1;//警告
    }
    else{//需要调节 lift high，适应高度的变小
      step_stand_high = STAND_HIGH - step_stand_dz;
      step_lift_high = STAND_HIGH-step_stand_dz-SIT_HIGH;
    }
  }
  else{//step_stand_dz <= 0 抬升高度
    step_stand_high = STAND_HIGH - step_stand_dz;
  }  
  #if !USE_COORDINATES_CONVERSION
  step_body_pitch = step_body_pitch_new;
  if(step_body_pitch == 0)
  {
    dx_pitch = dz_pitch = 0;
    step_lift_high = LIFT_HIGH;
  }
  else 
  {
    float pitch = angle2pi(abs(step_body_pitch));
    dx_pitch = const_body_length/2*(1-cos(pitch));
    dz_pitch = const_body_length/2*sin(pitch);
    step_lift_high = LIFT_HIGH * (STAND_HIGH-dz_pitch)/STAND_HIGH;
  }  
  #endif
}

void reCalcParam()
{
   step_cycloid_phase_time = (step_rate/(step_rate+1))*step_cycle_time;//摆动相占用的周期时间,对应2*PI
   step_support_phase_time = step_cycle_time - step_cycloid_phase_time;
   step_cycloid_phase_r = step_lift_high/2;//摆线半径
   step_length = step_cycloid_phase_r*2*M_PI;//x = r*(t-sin(t)),t=2*PI,x=r*(2*PI-0) = r*2*PI
   trot_step_rate = trot_step_length/step_length;
     
  if(start_deflection != 0 && deflection_angle>0)
  {
    float yaw,a;
    yaw = angle2pi(deflection_angle);
    a = const_body_width/2*tan(yaw/2);
    dx1_yaw = abs((const_body_length/2+a) - (const_body_length/2-a)*cos(yaw));
    dy1_yaw = (const_body_length/2-a)*sin(yaw);
    dx2_yaw = abs((const_body_length/2+a)*cos(yaw)-(const_body_length/2-a));
    dy2_yaw = (const_body_length/2+a)*sin(yaw); 
  }
 
//  if(step_body_pitch == 0)
//  {
//    dx_pitch = dz_pitch = 0;
//  }
//  else 
//  {
//    float pitch = angle2pi(abs(step_body_pitch));
//    dx_pitch = const_body_length/2*(1-cos(pitch));
//    dz_pitch = const_body_length/2*sin(pitch);
//    step_lift_high = LIFT_HIGH * (STAND_HIGH-dz_pitch)/STAND_HIGH;
//  }
  
}

void setCycleTime(int t)
{
  step_cycle_time = t;
}
void setLiftHigh(float h)
{
  if(step_stand_high-h>=70 && h>0){//站立高度相对抬腿高度要大于 70,站立高度保持不变
    step_lift_high = h;
  }  
}
void check_cycle_trot()
{
  String str;
  step_cur_time = millis();
  int period = step_cur_time - step_last_time;
  if(period > step_cycle_time)
  {
//    Serial.println("period >= step_cycle_time"); 
    if(start_trot <= 3)//一个周期结束可以停止trot步态
    {
      #if WITHOUT_NEW
      step_direction = step_direction*0.7;//最后几个周期direction递减
      step_direction_left = step_direction_right = step_direction;
      #else
      step_direction_new = step_direction_new*0.7;//最后几个周期direction递减
      step_direction_left_new = step_direction_right_new = step_direction_new;
      #endif
      if(start_trot == 0)
      {
        actionDirectTo(0,0,getStepStandHigh());
        Serial.println("start_trot == 0"); 
        return;
      }
    }
    start_trot--;//
    step_last_time = step_cur_time;
    period = 0;     
    
    #if !WITHOUT_NEW
    step_direction = step_direction_new;
    step_direction_theta = step_direction_theta_new;
    step_cycle_time = new_cycle_time;
    if(step_stand_high-new_lift_high>=70 && new_lift_high>0){//站立高度相对抬腿高度要大于 70,站立高度保持不变
      step_lift_high = new_lift_high;
    }
    step_direction_left = step_direction_left_new;
    step_direction_right = step_direction_right_new;
    step_direction_front = step_direction_front_new;
    step_direction_rear = step_direction_rear_new;    
    start_deflection = start_deflection_new;
    deflection_angle =deflection_angle_new;
    #endif
//    step_body_pitch = step_body_pitch_new;
    reCalcParam();
  }
//  Serial.print("period = ");
//  Serial.println(period);
  float x,y,z;
  float xp,yp,zp;
  float theta;
  float lineDirection;
//  float yaw,a, dx1_yaw,dy1_yaw,dx2_yaw,dy2_yaw;//左右顶点的偏转坐标
  adjust_stand_high();
//  Serial.printf("trot1 start_deflection = %d,deflection_angle = %f\n",start_deflection,deflection_angle); 
  for(int i = 0;i<4;i++)
  {
    if(i==0||i==3)//对角线的腿保持一致相位
    { 
      if(period <= step_cycloid_phase_time/2)
      {//摆动相 前半
//        Serial.println("摆动相 前半");
        theta = (float)map(period, 0, step_cycloid_phase_time/2, M_PI*zoomRate, 2*M_PI*zoomRate);
        theta = theta/zoomRate;
        x = -(step_cycloid_phase_r*(theta-sin(theta)) - step_length/2);//半个摆动相，平移半个步长，取负值适应狗的x轴方向
        z = step_cycloid_phase_r*(1-cos(theta));
        #if USE_COORDINATES_CONVERSION
        z = -z;
        #else
        z = step_stand_high - z;
        #endif
        y = 0;
      }
      else if(period >= (step_cycle_time-step_cycloid_phase_time/2 ))
      {//摆动相 后半
//        Serial.println("摆动相 后半");
        theta = (float)map(period, (step_cycle_time-step_cycloid_phase_time/2 ), step_cycle_time, 0, M_PI*zoomRate);
        theta = theta/zoomRate;       
        x = -(step_cycloid_phase_r*(theta-sin(theta)) - step_length/2);//半个摆动相，平移半个步长，取负值适应狗的x轴方向
#if 1//正常的摆线        
        z = step_cycloid_phase_r*(1-cos(theta));//后半依然是摆动相
        #if USE_COORDINATES_CONVERSION
        z = -z;
        #else
        z = step_stand_high - z;
        #endif
#else//变成直线，跨越能力提升，振动增加
        z = step_stand_high-step_lift_high;//后半摆动相变成直线，可以提高跨越能力
#endif
        y = 0;
      }
      else
      {//支撑相
//        Serial.println("支撑相");
        x = (float)map(period, step_cycloid_phase_time/2, step_cycle_time-step_cycloid_phase_time/2 , -step_length/2*zoomRate, step_length/2*zoomRate);
        x = x/zoomRate;
        #if USE_COORDINATES_CONVERSION
        z = 0;
        #else
        z = step_stand_high;
        #endif
        y = 0;
      }
    }
    else
    {
      if(period <= step_support_phase_time/2)
      {//支撑相 前半
//        Serial.println("支撑相 前半");
        x = (float)map(period, 0 ,step_support_phase_time/2 , 0, step_length/2*zoomRate)/zoomRate;
        #if USE_COORDINATES_CONVERSION
        z = 0;
        #else
        z = step_stand_high;
        #endif
        y = 0;
      }
      else if(period >= (step_cycle_time - step_support_phase_time/2))
      {//支撑相 后半
//        Serial.println("支撑相 后半");
        x = (float)map(period, (step_cycle_time - step_support_phase_time/2) ,step_cycle_time , -step_length/2*zoomRate,0)/zoomRate;
        #if USE_COORDINATES_CONVERSION
        z = 0;
        #else
        z = step_stand_high;
        #endif
        y = 0;
      }
#if 0//变成直线，跨越能力提升，振动增加      
//  1，2腿也分成摆动相的前半和后半，前半为直线，后半为摆线  ， 1，2腿和0，3腿的前半后半是相反的
      else if(period > step_support_phase_time/2 && period <=  step_cycle_time/2)
      {//前半摆动相变成直线
        theta = (float)map(period,step_support_phase_time/2, step_cycle_time-step_support_phase_time/2, 0, 2*M_PI*zoomRate);
        theta = theta/zoomRate;         
        x = -(step_cycloid_phase_r*(theta-sin(theta)) - step_length/2);
        z = step_stand_high - step_lift_high;
        y = 0;        
      }
#endif
      else
      {//摆动相,变成后半摆动相，继续摆线
//        Serial.println("摆动相");
        theta = (float)map(period,step_support_phase_time/2, step_cycle_time-step_support_phase_time/2, 0, 2*M_PI*zoomRate);
        theta = theta/zoomRate;         
        x = -(step_cycloid_phase_r*(theta-sin(theta)) - step_length/2);
        z = step_cycloid_phase_r*(1-cos(theta));
        #if USE_COORDINATES_CONVERSION
        z = -z;
        #else
        z = step_stand_high - z;
        #endif
        y = 0;
      }
      
    }
    x = x*trot_step_rate;//限定trot步长最大为trot_step_length 
//    Serial.printf("trot2 start_deflection = %d,deflection_angle = %f\n",start_deflection,deflection_angle); 
    if(start_deflection != 0 && deflection_angle>0)//左右调头，原地转 -1左，1右
    {
      if(start_deflection < 0)//left
      {
        step_deflection_len_1 = sqrt(pow(dx1_yaw,2)+pow(dy1_yaw,2));
        step_deflection_len_2 = sqrt(pow(dx2_yaw,2)+pow(dy2_yaw,2));
        step_deflection_theta_1 = atan(abs(dy1_yaw/dx1_yaw));
        step_deflection_theta_2 = atan(abs(dx2_yaw/dy2_yaw));
        step_deflection_rate_1 = step_deflection_len_1/trot_step_length;//step_length;//其实rate1 == rate2
        step_deflection_rate_2 = step_deflection_len_2/trot_step_length;//step_length;    
        if(i == 0 || i == 3)
        {
          lineDirection = x*step_deflection_rate_1;
          if(i == 0)
            step_deflection_theta_1 = M_PI - step_deflection_theta_1;
          else
            step_deflection_theta_1 = -step_deflection_theta_1;
          step_direction_theta = step_deflection_theta_1;
        }
        else//1,2
        {
          lineDirection = x*step_deflection_rate_2;
          if(i == 2)
            step_deflection_theta_2 = M_PI/2 - step_deflection_theta_2;
          else
            step_deflection_theta_2 = -(M_PI/2 + step_deflection_theta_2);
          step_direction_theta = step_deflection_theta_2;
        }            
      }
      else//1,right
      {
        step_deflection_len_1 = sqrt(pow(dx2_yaw,2)+pow(dy2_yaw,2));
        step_deflection_len_2 = sqrt(pow(dx1_yaw,2)+pow(dy1_yaw,2));  
        step_deflection_theta_1 = atan(abs(dx2_yaw/dy2_yaw));
        step_deflection_theta_2 = atan(abs(dy1_yaw/dx1_yaw));   
        step_deflection_rate_1 = step_deflection_len_1/trot_step_length;//step_length;//其实rate1 == rate2
        step_deflection_rate_2 = step_deflection_len_2/trot_step_length;//step_length;  
        if(i == 0 || i == 3)
        {
          lineDirection = x*step_deflection_rate_1;    
          if(i == 0)
            step_deflection_theta_1 = -(M_PI/2 - step_deflection_theta_1);
          else
            step_deflection_theta_1 = (M_PI/2 + step_deflection_theta_1);
          step_direction_theta = step_deflection_theta_1;
        }
        else//1,2
        {
          lineDirection = x*step_deflection_rate_2;   
          if(i == 2)
             step_deflection_theta_2 = -(M_PI-step_deflection_theta_2);
          else
             step_deflection_theta_2 = step_deflection_theta_2;
          step_direction_theta = step_deflection_theta_2;
        }
      }
 
      x = lineDirection*cos(step_direction_theta);
      y = lineDirection*sin(step_direction_theta);      
    }
    else{
//    lineDirection = x*step_direction;

      if(step_direction_front != step_direction_rear)//是绕圈模式
      {
        if(i == 0 || i == 2)
          lineDirection = x*step_direction_front;
        else
          lineDirection = x*step_direction_rear;        
      }
      else//是左右偏转模式,或前进后退
      {
        if(i == 0 || i == 1)
          lineDirection = x*step_direction_left;
        else
          lineDirection = x*step_direction_right;
      }      
      x = lineDirection*cos(step_direction_theta);
      y = lineDirection*sin(step_direction_theta);
#if !USE_COORDINATES_CONVERSION
      if(step_body_pitch != 0)//俯仰行走对x，z做变换
      {
        float pitch = angle2pi(abs(step_body_pitch));
        float theta,len;
        if(step_body_pitch<0)//前趴
        { 
          if(i == 0 || i == 2){
            z = z - dz_pitch;
            x = x+dx_pitch;//让两个坐标系重合
          }
          else{
            z = z + dz_pitch;
            x = x-dx_pitch;//让两个坐标系重合
          }    
        }
        else//抬头
        {
          if(i == 0 || i == 2){
            z = z + dz_pitch;
            x = x+dx_pitch;//让两个坐标系重合
          }
          else{
            z = z - dz_pitch;
            x = x-dx_pitch;//让两个坐标系重合
          }          
        }
        len = sqrt(pow(x,2)+pow(z,2));
        theta = atan(x/z);
        x = len*sin(theta - pitch);    
        z = len*cos(theta - pitch);   
        lineDirection = x*step_direction;     
      }
#endif
    }

#if USE_COORDINATES_CONVERSION
      attitudeAdjustStep(i,x,y,z,step_body_pitch,step_body_roll);
#endif    
    direct_postion(i,x,y,z);

//  if(i == 0)
//  {
////    Serial.printf("trot3 start_deflection = %d,deflection_angle = %f\n",start_deflection,deflection_angle); 
//    if(start_deflection != 0 && deflection_angle>0)
//      Serial.printf("trot4 x = %f,y = %f,z = %f\n",x,y,z);  
//  }
////    str = "leg = "+String(i)+",x = "+String(x)+",y = "+String(y)+",z = "+String(z);
////    Serial.println(str);
  }

}

//矩形宽a ，长b ，偏转夹角为theta，求顶点偏转距离l，
//先得出矩形对角线长度的一半为c
//  c = sqrt(pow(a,2)+pow(b,2))/2
//  由余弦定理,偏转矩形和原矩形的对角线和偏转距离构成一个等腰三角形，腰长c，底边长l,夹角theta
//  pow(l,2) = pow(c,2)+pow(c,2)-2cc*cos(theta)
//  pow(l,2) = 2*pow(c,2)-2*pow(c,2)*cos(theta) = 2*pow(c,2)(1-cos(theta))
//
//float getDeflectionLen(float angle)//根据偏转角求偏转距离
//{//const_body_width ,const_body_length
//  float len,theta,c2;
//  theta = angle*M_PI/180;
//  c2 = (pow(const_body_width,2)+pow(const_body_length,2))/4;//c2代表c的平方，这里只要c的平方值
//  len = sqrt(2*c2*(1-cos(theta)));
//  Serial.printf("getDeflectionLen = %f\n",len);   
//  return len;
//}

//float getDeflectionThetaL(float angle)
//{
//  float theta,t;
//  t = angle*M_PI/180;
//  theta = (M_PI-t)/2-atan(const_body_width/const_body_length);
//  Serial.printf("getDeflectionThetaR = %f\n",theta/M_PI*180); 
//  return theta;
//}
// float getDeflectionThetaR(float angle)
//{
//  float theta,t;
//  t = angle*M_PI/180;
//  theta = (M_PI-t)/2-atan(const_body_length/const_body_width);
//  Serial.printf("getDeflectionThetaR = %f\n",theta/M_PI*180); 
//  return theta;
//}
  
void attitudeAdjustStep(int i,float& x,float& y,float& z,float pitch,float roll)
{
  //pitch角调整
  float theta=0;
  float deltaHigh,h,z_raw;
  h=step_stand_high;
  z_raw = z;
  theta = pitch/180*M_PI;  
  deltaHigh=const_body_length/2*sin(theta);
  if(i==0 || i==2)
    h += deltaHigh;
  else
    h -= deltaHigh;
  x = x*cos(theta)+h*sin(theta);
  z = z*cos(theta)+h*cos(theta);  

  //roll角调整
  h = z;
  theta = roll/180*M_PI;  
  deltaHigh=const_body_width/2*sin(theta);
  if(i==0 || i==1)
    h += deltaHigh;
  else
    h -= deltaHigh;
  y = y*cos(theta)+h*sin(theta);
  z = z_raw*cos(theta)+h*cos(theta);  
//  if(i == 1)
//  Serial.printf("attitude x = %f,y = %f,z = %f,pitch = %f,roll = %f\n",x,y,z,pitch,roll);  
}

  
  
