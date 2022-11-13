
//摆线方程
/*
 *      x = r(t-sin(t))
 *      z = r(1-cos(t))
 * 
 *      r为半径，为抬腿高度的一半，t为滚动角， 0 < t < 2*pi , 
 *      从原点启动的话t是0-2pi，我们是从站立方式启动，启动是在t=pi位置，就是摆线的中心点开始
 * 
 * 
 * 
 *      前进 0,3,2,1 按这个顺序依次做摆线
 *      后退 3,0,1,2 按这个顺序依次做摆线
 *      一个大循环分成4段，每一段对应的腿抬起做摆线移动一个步长，其余腿做地面的直线移动一个步长，产生前进或后退
 *      从站立位置开始做循环，第一个循环稍有不同，之后的循环保持不变
 *      第一个循环由于起始位是在零点位置，最后移动的是1脚，会移动到3个步长才开始抬腿做摆线，
 *      之后的循环都是到2个步长就开始做摆线
 *      大循环是4个脚全部走一遍，小周期是每个脚做一次摆线
 *      
 *      这里要加入坐标系变换，A坐标是原始坐标系，B坐标系是俯仰后的足端相对A的坐标系
 * */

//hardcode 值
const float walk_step_length = 70;//步长，跨出离中心点的距离 
const float walk_step_high = 60;//抬腿高度
const int walk_step_cycle_time = 4000;//大循环周期 
const int forwardOrder[4] = {0,3,2,1}; 
const int backOrder[4] = {3,0,1,2}; 
const float walk_gravity_offset = 15;        //为了重心规划，需要在抬腿的那一前后排身体抬起一个高度，避免失去重心
const float walk_gravity_offset_y = 30; 
const float walk_step_gravity_rate = 0.4;//每个小周期开始先做重心移动，再做摆动，在平移
const float walk_step_cycloid_rate = 0.6;//0.5-1 ， 1为小周期内都在做摆动相，任何时间有一条腿在做摆动相，其余3条腿在做支撑相，
                                          //小于1 表示一个小周期内摆动相的占比，这样在任意小周期内就有4腿同时支撑的可能，有助于系统保持稳定
                                          
                                          
//运行时变量
int walk_step_last_time = 0;
int walk_step_cur_time = 0;//当前所在周期的时间
float walk_step_length_left_rate = 1;//左右步长比率，用于转弯和调整方向
float walk_step_length_right_rate = 1;
float walk_step_direction = 1;//1是标准步长前进，-1是标准步长后退,0是原地， 小数可以表示移动的幅度，小于一个步长
float walk_cycle_init_postion[4][3] = {0};//四条腿在每个周期开始的时候 的位置
float walk_raw_postion[4][3] = {0};//没有变化前的坐标，相对的curPostion[][]是可能已经变化roll和pitch之后的坐标，做roll和pitch变换需要在raw的基础上做
float walk_step_rate[4]={0};//每条腿在每个小周期开始做摆线的时候需要知道自己摆线的长度，不做摆线的是设为0
int walk_cycle_cur_step=0;//当前的小周期步骤
float walk_step_support_rate = 1-walk_step_cycloid_rate-walk_step_gravity_rate;//0-0.5,小周期内的支撑相比率
float walk_gravity_pitch = 0;//行进中为了重心规划，调整身体的倾斜度，正为抬头，负为低头，先进要抬头，后退要低头
float walk_gravity_roll = 0;//左翻滚为正，右翻滚为负
int walk_cycle_cur_step_status = 0;//这个是为了连续走动模式 0为重力调节相位，1为支撑相位（在摆动循环中的）
void setWalkStepLengthRate(float left,float right)
{
    if(left>1 || left<0)
        return;
    if(right>1 || right<0)
        return;
    walk_step_length_left_rate = left;
    walk_step_length_right_rate = right;
}

void setWalkDirection(float d)
{
    if(d>1 || d<-1)
        return;
    walk_step_direction = d;
}
void startWalk()
{
    Serial.println("startWalk.....");
    walk_step_last_time = millis();
//    getCycleInit();
    setRawInitPostionAll();
    walk_cycle_cur_step = 0;
    servoThreadMode = 3;
}

void stopWalk()
{
    Serial.println("stopWalk.....");
    servoThreadMode = 0;
}

bool isWalkRunning()
{
//    Serial.println("isWalkRunning.....");
    if(servoThreadMode == 3)
        return true;
    return false;
}
void getCycleInitAll()
{
    for(int i=0;i<4;i++)
    {
      getCycleInit(i);
    }
}

void getCycleInit(int i)
{
    for(int j=0;j<3;j++)
    {
        walk_cycle_init_postion[i][j] = curPostion[i][j];
    }  
}

void setRawInitPostionAll()
{
  for(int i=0;i<4;i++)
  {
    for(int j=0;j<3;j++)
    {
        walk_raw_postion[i][j] = 0;
    }  
  }
}
void setRawPostion(int i,float x,float y,float z)
{
  if(i<0 ||i>4)
    return;
  walk_raw_postion[i][0] = x;
  walk_raw_postion[i][1] = y;
  walk_raw_postion[i][2] = z;       
}
void getRawPostionAll()
{
  for(int i=0;i<4;i++)
    getRawPostion(i);
}
void getRawPostion(int i)
{
  if(i<0 ||i>4)
    return;
  for(int j=0;j<3;j++)
    walk_cycle_init_postion[i][j] = walk_raw_postion[i][j];
}
void check_cycle_walk()
{
    walk_step_cur_time = millis();
    int period = walk_step_cur_time - walk_step_last_time;
    if(period>walk_step_cycle_time)//一个周期结束，可以重新调整一些参数
    {
        walk_step_last_time = walk_step_cur_time;
        period = 0;//period-walk_step_cycle_time;
    }

    if(period<=walk_step_cycle_time/4)
    {
        do_cycle_step(1,period);
    }
    else if(period<=walk_step_cycle_time/2)
    {
        do_cycle_step(2,period);
    }
    else if(period<=walk_step_cycle_time*3/4)
    {
        do_cycle_step(3,period);
    }
    else if(period<=walk_step_cycle_time)
    {
        do_cycle_step(4,period);
    }

}

void do_cycle_step(int nStep,int period)
{
    if(nStep>4 ||nStep<=0)
        return;
    float x,y,z,t;
    int nLegCycloid = 0;//要抬腿做摆动相的腿
    if(walk_step_direction>=0)
        nLegCycloid=forwardOrder[nStep-1]; 
    else
        nLegCycloid=backOrder[nStep-1];    
    float processingValue = (float)(period-walk_step_cycle_time*(nStep-1)/4)/(walk_step_cycle_time/4);//在单个四分之一周期中的进度百分比
    float gravityValue = processingValue;
    float gravityPeakRate = 0.5;//峰值比例，就是gravityValue何时达到最高值1，小于0.5是快速达到，大于0.5是慢速
    //gravityValue*(0-1-0)
    if(processingValue<=gravityPeakRate)
      gravityValue = processingValue/gravityPeakRate;
    else  
      gravityValue = (1-processingValue)/gravityPeakRate;
    
    //theta 为摆线的相位角，0-2pi  , 把时间转换成对应阶段的摆线相位角
    float theta = processingValue*2*M_PI/walk_step_cycloid_rate;
//    Serial.printf("do_cycle_step.....nstep=%d,period=%d,value=%f\n",nStep,period,processingValue);
    if(walk_cycle_cur_step != nStep)
    {//每个小周期都要记录一次位置
      walk_cycle_cur_step = nStep;
      getCycleInitAll();
    }
    for(int i=0;i<4;i++)
    {
        if(i == nLegCycloid)//做摆线
        {
            if(processingValue<=walk_step_cycloid_rate)
            {//摆动相
              //第一个周期每次摆线的长度是不相同的，所以walk_step_rate每次要计算，其实从第二个周期开始是一样的，简化计算就每次都算
              if(walk_step_rate[i]==0)
              {
                walk_step_rate[i] = (float)(walk_step_length+abs(walk_cycle_init_postion[i][0]))/(walk_step_high*M_PI);
              }
              t = (float)walk_step_high/2*(theta - sin(theta))*walk_step_rate[i]*walk_step_direction;//按步长对摆线在宽度方向通过walk_step_rate进行压缩，使其移动宽度为一个步长 walk_step_length，再通过walk_step_direction进行幅度调整
              if(abs(abs(t)-walk_step_length)<1)//消除x方向的误差
                t = walk_step_length;
              x = (float)walk_cycle_init_postion[i][0]-t;
              z = (float)walk_step_high/2*(1-cos(theta));
              z = (float)walk_cycle_init_postion[i][2]-z;
//            Serial.printf("nStep=%d,nLeg=%d,rate=%f,pos=%f,t=%f x=%f\n",nStep,i,walk_step_rate[i],walk_cycle_init_postion[i][0],t,x);
            }
            else
            {//提前完成摆动相后，剩下的时间做支撑相
              if(walk_step_rate[i]>0)
              {
                walk_step_rate[i]=0;
                getCycleInit(i);
              }
              //把进度转换到 支撑相的进度
              t = ((processingValue-walk_step_cycloid_rate)/walk_step_support_rate)*(walk_step_length*2/(3+walk_step_support_rate))*walk_step_support_rate*walk_step_direction;
              x = walk_cycle_init_postion[i][0]+t;
              z = STAND_HIGH;
            }
        }
        else//做平移,每个小周期的平移都是2/3个步长
        {
            //这里(3+walk_step_support_rate)表示做支撑相的周期数，目前为3.25，为每条腿在做摆动相后要做3.25个支撑相
            t = (float)processingValue*(walk_step_length*2/(3+walk_step_support_rate))*walk_step_direction;
            x = walk_cycle_init_postion[i][0]+t;
//            if(i == 0)
//              Serial.printf("nStep = %d ,processing= %f x = %f\n",nStep,processingValue,x);
            z = STAND_HIGH;//walk_cycle_init_postion[i][2];z方向用数组会累积误差,用固定值消除误差
            walk_step_rate[i]=0;
        }
        
#if 1
        //重心规划，方案1 ，通过调节前后腿的高度，使重心保持相对稳定，缺点：前后俯仰幅度偏大
        if(nLegCycloid == 0||nLegCycloid==2)
        {//前排做摆线
          if(i == 1 || i==3)
          {//后排降低高度
            if((nLegCycloid == 0 && i == 3)||(nLegCycloid == 2 && i == 1))//对角线的腿高度多降低一倍
              z = z-walk_gravity_offset*gravityValue*2;//gravityValue*(0-2-0)
            else
              z = z-walk_gravity_offset*gravityValue;//gravityValue*(0-1-0)
          }
          else if(nLegCycloid!=i)
          {//前排不做摆线的腿抬高角度
            z = z+walk_gravity_offset*gravityValue;
          }
        }
        else
        {//后排做摆线
          if(i == 0 || i == 2)
          {//前排降低高度
            if((nLegCycloid == 1 && i == 2)||(nLegCycloid == 3 && i == 0))//对角线的腿高度多降低一倍
              z = z-walk_gravity_offset*gravityValue*2;//gravityValue*(0-2-0)
            else
              z = z-walk_gravity_offset*gravityValue;//gravityValue*(0-1-0)
          }
          else if(nLegCycloid!=i)
          {//后排不做摆线的腿抬高角度
            z = z+walk_gravity_offset*gravityValue;
          }         
        }     
        y = 0;   
#endif

#if 1
        //重心规划，方案2，通过调整y来保持重心在3条支撑腿形成的三角形内
        //目前先用简单的hardcode值walk_gravity_offset_y来简化,效果不明显
        if(nLegCycloid == 0||nLegCycloid==1)
        {//左侧抬腿，重心右移 ，减小y
          if(y > -walk_gravity_offset_y)
          {
            t = abs(-walk_gravity_offset_y-walk_cycle_init_postion[i][1])*processingValue;
            y = walk_cycle_init_postion[i][1]-t;
            if(y<-walk_gravity_offset_y)
              y=-walk_gravity_offset_y;
          }
        }
        else
        {//右侧抬腿，重心左移，加y
          if(y < walk_gravity_offset_y)
          {
            t = abs(walk_gravity_offset_y-walk_cycle_init_postion[i][1])*processingValue;
            y = walk_cycle_init_postion[i][1]+t;
            if(y>walk_gravity_offset_y)
              y=walk_gravity_offset_y;
          }
        }  
        
#endif
        //通过左右rate 进行转向
        if(i==0 || i==1)
            x = (float)x*walk_step_length_left_rate;
        else
            x = (float)x*walk_step_length_right_rate;
        
       
        direct_postion(i,x,y,z);
//        if(i == 0)
//          Serial.printf("nStep = %d ,t= %f x = %f,z = %f\n",nStep,t,x,z); 
    }    
}


void check_cycle_walk_ex()
{
    walk_step_cur_time = millis();
    int period = walk_step_cur_time - walk_step_last_time;
    if(period>walk_step_cycle_time)//一个周期结束，可以重新调整一些参数
    {
        walk_step_last_time = walk_step_cur_time;
        period = 0;//period-walk_step_cycle_time;
    }

    if(period<=walk_step_cycle_time/4)
    {
        do_cycle_step_ex(1,period);
    }
    else if(period<=walk_step_cycle_time/2)
    {
        do_cycle_step_ex(2,period);
    }
    else if(period<=walk_step_cycle_time*3/4)
    {
        do_cycle_step_ex(3,period);
    }
    else if(period<=walk_step_cycle_time)
    {
        do_cycle_step_ex(4,period);
    }

}

void do_cycle_step_ex(int nStep,int period)//这个方式是摆动相的时候，其余腿不动，再一起做支撑相
{
  if(nStep>4 ||nStep<=0)
      return;
  if(walk_cycle_cur_step != nStep)
  {//每个小周期都要记录一次位置
    walk_cycle_cur_step = nStep;
    getRawPostionAll();
  }
  float processingValue = (float)(period-walk_step_cycle_time*(nStep-1)/4)/(walk_step_cycle_time/4);//在单个四分之一周期中的进度百分比
#if 0 //这个是只有支撑相的时候做平移
  if(processingValue <= walk_step_gravity_rate)
  {
    float gravity_processing = processingValue/walk_step_gravity_rate;
    do_cycle_step_gravity(nStep,gravity_processing);
  }
  else if(processingValue <= walk_step_cycloid_rate+walk_step_gravity_rate)
  {
    float cycloid_processing = (processingValue-walk_step_gravity_rate)/walk_step_cycloid_rate;
    do_cycle_step_cycloid(nStep,cycloid_processing);
  }
  else
  {
    float support_processing = (processingValue-walk_step_gravity_rate-walk_step_cycloid_rate)/walk_step_support_rate;
    do_cycle_step_support(nStep,support_processing);
  }
#else//这个是每个每个时刻都在做平移的
  if(processingValue <= walk_step_gravity_rate)
  {
    float gravity_processing = processingValue/walk_step_gravity_rate;
    do_cycle_step_gravity_ex(nStep,gravity_processing);
  }
  else
  {
    float cycloid_processing = (processingValue-walk_step_gravity_rate)/(1-walk_step_gravity_rate);
    do_cycle_step_cycloid_ex(nStep,cycloid_processing);
  }
#endif
}

void do_cycle_step_gravity(int nStep,float processing)
{
  float x,y,z,t;
  int nLegCycloid = 0;//将要抬腿做摆动相的腿
  if(walk_step_direction>=0)
      nLegCycloid=forwardOrder[nStep-1]; 
  else
      nLegCycloid=backOrder[nStep-1];    
  for(int i=0;i<4;i++) 
  {
    x = walk_cycle_init_postion[i][0];
    y = walk_cycle_init_postion[i][1];

    if(nStep==2 || nStep==4)
    {//跳转到下一个步骤 step_cycloid
      walk_step_last_time -= walk_step_cycle_time/4*walk_step_gravity_rate;
      return;        
    }
    
    if(abs(y)<walk_gravity_offset_y/2)
    {
      if(nLegCycloid == 0 ||nLegCycloid == 1)
        y = -walk_gravity_offset_y*processing;
      else
        y = walk_gravity_offset_y*processing;
    }
    else
    {//跳转到下一个步骤 step_cycloid
      walk_step_last_time -= walk_step_cycle_time/4*walk_step_gravity_rate;
      return;
    }
    z = 0;
    setRawPostion(i,x,y,z);
    attitudeAdjust(i,x,y,z);
    direct_postion(i,x,y,z);   
//    getRawPostion(i);  
    if(i == 0)
      Serial.printf("Gravity nStep = %d,p = %f,x = %f,y = %f,z = %f\n",nStep,processing,x,y,z);  
  }
}
void do_cycle_step_cycloid(int nStep,float processing)
{
  float x,y,z,t;
  int nLegCycloid = 0;//要抬腿做摆动相的腿
  if(walk_step_direction>=0)
      nLegCycloid=forwardOrder[nStep-1]; 
  else
      nLegCycloid=backOrder[nStep-1];    
  float theta = processing*2*M_PI;    

  for(int i=0;i<4;i++)
  {
    if(i == nLegCycloid)//做摆线
    {
      //第一个周期每次摆线的长度是不相同的，所以walk_step_rate每次要计算，其实从第二个周期开始是一样的，简化计算就每次都算
      if(walk_step_rate[i]==0)
      {
        getRawPostion(i);
        walk_step_rate[i] = (float)(walk_step_length+abs(walk_cycle_init_postion[i][0]))/(walk_step_high*M_PI);
//        if(i == 0)
//          Serial.printf("Cycloid nStep = %d,init x = %f.........\n",nStep,walk_cycle_init_postion[i][0]); 
      }
      t = (float)walk_step_high/2*(theta - sin(theta))*walk_step_rate[i]*walk_step_direction;//按步长对摆线在宽度方向通过walk_step_rate进行压缩，使其移动宽度为一个步长 walk_step_length，再通过walk_step_direction进行幅度调整
//      if(abs(abs(t)-walk_step_length)<1)//消除x方向的误差
//        t = walk_step_length;
      x = walk_cycle_init_postion[i][0]-t;
      z = -walk_step_high/2*(1-cos(theta));//在B坐标下，z是在上方进行摆线运动，是在负方向进行的
//      z = (float)walk_cycle_init_postion[i][2]-z;
      y = walk_cycle_init_postion[i][1];//0;
    }  
    else
    {
       walk_step_rate[i] = 0;
       getRawPostion(i); 
       x = walk_cycle_init_postion[i][0];
       y = walk_cycle_init_postion[i][1];//0;
       z = 0;   
    }
     
    setRawPostion(i,x,y,z);
    attitudeAdjust(i,x,y,z);
    direct_postion(i,x,y,z); 
    if(i == 0)
      Serial.printf("Cycloid nStep = %d,p = %f,x = %f,y = %f,z = %f\n",nStep,processing,x,y,z);         
  }
}

void do_cycle_step_support(int nStep,float processing)
{
  float x,y,z,t;

  for(int i=0;i<4;i++)
  {
    if(walk_step_rate[i]>0)
    {
      walk_step_rate[i]=0;
      getRawPostion(i);
    }
    
    t = (float)processing*walk_step_length*2/3*walk_step_direction;
    x = walk_cycle_init_postion[i][0]+t;
    z = 0;//walk_cycle_init_postion[i][2];z方向用数组会累积误差,用固定值消除误差
    if(nStep==2||nStep==4)
    {
      if(nStep==2)
        y = walk_cycle_init_postion[i][1]+processing*walk_gravity_offset_y;
      else
        y = walk_cycle_init_postion[i][1]-processing*walk_gravity_offset_y;  
    }
    else    
      y = walk_cycle_init_postion[i][1]*(1-processing);//0;

    
    setRawPostion(i,x,y,z);
    attitudeAdjust(i,x,y,z);
    direct_postion(i,x,y,z); 
    if(i == 0)
      Serial.printf("Support nStep = %d,p = %f,x = %f,y = %f,z = %f\n",nStep,processing,x,y,z);                 
  }
}

void attitudeAdjust(int i,float& x,float& y,float& z)
{
  //pitch角调整
  float theta=0;
  float deltaHigh,h;
  h=STAND_HIGH;
  
  theta = walk_gravity_pitch/180*M_PI;  
  deltaHigh=const_body_length/2*sin(theta);
  if(i==0 || i==2)
    h += deltaHigh;
  else
    h -= deltaHigh;
  x = x*cos(theta)+h*sin(theta);
  z = z*cos(theta)+h*cos(theta);  

  //roll角调整
  if(walk_gravity_roll!=0)
  {
  
  }
}

//void gravityAdjust(int nStep,float processing,float& x,float& y,float& z)
//{
//  float gravityValue = processing;
////  float gravityPeakRate = 0.5;//峰值比例，就是gravityValue何时达到最高值1，小于0.5是快速达到，大于0.5是慢速
////  //gravityValue*(0-1-0)
////  if(processing<=gravityPeakRate)
////    gravityValue = processing/gravityPeakRate;
////  else  
////    gravityValue = (1-processing)/(1-gravityPeakRate);  
//
//  float adjustY = 0;
//  float yValue = 40;
//  int nLegCycloid;
//  if(nStep>=4)
//    nStep = 0;
//  if(walk_step_direction>=0)
//      nLegCycloid=forwardOrder[nStep]; 
//  else
//      nLegCycloid=backOrder[nStep];  
//
//      
//  if(walk_step_direction>0)//前进
//  {
//    if(nLegCycloid == 0)
//       adjustY = -yValue;
//    else if(nLegCycloid == 2)
//      adjustY = yValue;
//  }
//  else if(walk_step_direction<0)
//  {
//    if(nLegCycloid == 1)
//       adjustY = -yValue;
//    else if(nLegCycloid == 3)
//      adjustY = yValue;    
//  }
//  y+=adjustY*gravityValue;
//}

void do_cycle_step_gravity_ex(int nStep,float processing)
{
  float x,y,z,t;
  int nLegCycloid = 0;//将要抬腿做摆动相的腿
  if(walk_step_direction>=0)
      nLegCycloid=forwardOrder[nStep-1]; 
  else
      nLegCycloid=backOrder[nStep-1];    
  walk_cycle_cur_step_status = 0;    
  for(int i=0;i<4;i++) 
  {
    t = walk_step_length*2/3*walk_step_gravity_rate*walk_step_direction*processing;
    x = walk_cycle_init_postion[i][0]+t;
    y = walk_cycle_init_postion[i][1];

    if(nStep==1 || nStep==3)
    {    
      if(abs(y)<walk_gravity_offset_y/2)
      {
        if(nLegCycloid == 0 ||nLegCycloid == 1)
          y = -walk_gravity_offset_y*processing;
        else
          y = walk_gravity_offset_y*processing;
      }
    }
    else
    {
      if(abs(y)>walk_gravity_offset_y/2)
      {
        if(nLegCycloid == 0 ||nLegCycloid == 1)
          y = walk_gravity_offset_y*(1-processing);
        else
          y = -walk_gravity_offset_y*(1-processing);   
      }   
    }
    z = 0;
    
    setRawPostion(i,x,y,z);
    attitudeAdjust(i,x,y,z);
    direct_postion(i,x,y,z);   
//    getRawPostion(i);  
    if(i == 0)
      Serial.printf("Gravity nStep = %d,p = %f,x = %f,y = %f,z = %f\n",nStep,processing,x,y,z);  
  }
}
void do_cycle_step_cycloid_ex(int nStep,float processing)
{
  float x,y,z,t,r;
  int nLegCycloid = 0;//要抬腿做摆动相的腿
  if(walk_step_direction>=0)
      nLegCycloid=forwardOrder[nStep-1]; 
  else
      nLegCycloid=backOrder[nStep-1];    
  float theta = processing*2*M_PI;    
  if(walk_cycle_cur_step_status == 0)
  {
     walk_cycle_cur_step_status = 1;
     getRawPostionAll(); 
  }
  for(int i=0;i<4;i++)
  {
    if(i == nLegCycloid)//做摆线
    {
      //第一个周期每次摆线的长度是不相同的，所以walk_step_rate每次要计算，其实从第二个周期开始是一样的，简化计算就每次都算
      if(walk_step_rate[i]==0)
      {
        getRawPostion(i);
        walk_step_rate[i] = (float)(walk_step_length+abs(walk_cycle_init_postion[i][0]))/(walk_step_high*M_PI);
      }
      t = (float)walk_step_high/2*(theta - sin(theta))*walk_step_rate[i]*walk_step_direction;//按步长对摆线在宽度方向通过walk_step_rate进行压缩，使其移动宽度为一个步长 walk_step_length，再通过walk_step_direction进行幅度调整

      x = walk_cycle_init_postion[i][0]-t;
      z = -walk_step_high/2*(1-cos(theta));//在B坐标下，z是在上方进行摆线运动，是在负方向进行的
//      y = walk_cycle_init_postion[i][1];//0;
    }  
    else
    {
      if(walk_step_rate[i]>0)
      {
         getRawPostion(i); 
         walk_step_rate[i] = 0;
      }

      t = walk_step_length*2*(1-walk_step_gravity_rate)/3*walk_step_direction*processing;
      x = walk_cycle_init_postion[i][0]+t;
      z = 0;   
    }
    y = walk_cycle_init_postion[i][1];//0;
    if(nStep == 1 || nStep ==3)
    {
      if(nLegCycloid == 0 ||nLegCycloid == 1)
        y = -walk_gravity_offset_y;
      else
        y = walk_gravity_offset_y;
   
    }

    setRawPostion(i,x,y,z);
    attitudeAdjust(i,x,y,z);
    direct_postion(i,x,y,z); 
    if(i == 0)
      Serial.printf("Cycloid nStep = %d,p = %f,x = %f,y = %f,z = %f\n",nStep,processing,x,y,z);         
  }
}
