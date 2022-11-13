//接线方式 
//左前脚0 大腿，小腿，肩关节       
//左后腿1 大腿，小腿，肩关节   
//右前退2 大腿，小腿，肩关节    
//右后退3 大腿，小腿，肩关节   

//beta 是大小腿的夹角范围，由模型决定了其范围
#define BETA_MIN 30
#define BETA_MAX 135


#if SERV0_SPT_5835
//腿的角度范围都是根据 0腿来设定的，其他三条腿要做相应调整，参考：verifyAnagleMaxAndMin
#define min_servo_ham_degree 45    //大腿的最小角度
#define max_servo_ham_degree 175  //大腿的最大角度 alpha

#define min_servo_leg_degree 70 //小腿的最小角度,这角度是小腿摇臂和大臂上的肩关节容易发生碰撞的地方
#define max_servo_leg_degree 175 //小腿的最大角度 beta

#define min_servo_shoulder_degree 45    //肩关节的最小direct_postion角度
#define max_servo_shoulder_degree 135      //肩关节的最大角度 gama

#define min_diff_degree 40 //servo1-servo2, 
#define max_diff_degree 150

#else//SERV0_SPT_5435
#define min_servo_ham_degree 95   //大腿的最小角度
#define max_servo_ham_degree 285  //大腿的最大角度 alpha

#define min_servo_leg_degree 90 //小腿的最小角度,这角度是小腿摇臂和大臂上的肩关节容易发生碰撞的地方
#define max_servo_leg_degree 180 //小腿的最大角度 beta

#define min_servo_shoulder_degree 45    //肩关节的最小direct_postion角度
#define max_servo_shoulder_degree 135      //肩关节的最大角度 gama
#endif

#define stand_Z 160
#define lift_Z 100
#define step_X 20

//#define pi 3.14 use M_PI
#define KEEP 255
volatile float curPostion[4][3];
volatile float expPostion[4][3];
volatile float postionReady[4][3];
float stepSpeed = 1;//步速,最大为6

//坐标系标定 x 为前后方向，向后为正， y为左右方向，向右为正， z为上下方向，向下为正

const float La=100; // 大腿长度
const float Lb=100;   // 小腿长度
const float Lc=40;  // 肩膀长度

void setActionSpeed(float s)
{
  stepSpeed = s;
}
void postion_to_angle(int leg, float x, float y, float z)
{
#if ONLY_TEST_FIRST_LEG
  if(leg != 0)
    return;
#endif
  //alpha为大腿和上垂线的夹角，大腿在水平线下方为正， beta为大小腿的夹角
  float alpha,beta,gama;
  if(y != 0)//对于0脚
  {//需要变成三维坐标，z需要重新计算
    float L4 = 0;
    if(leg == 0 || leg == 1)
      L4 = abs(y-Lc);
    else//if(leg == 2 || leg == 3)
      L4 = abs(y+Lc);
    
    float L5 = sqrt(pow(z,2)+pow(L4,2));
//    if(L4 == 0){
//      Serial.println("avoid crash L4 = 0 !!!!!!!!");
//      return;
//    }
    float A = 0;// atan(z/L4);
    if(L4 == 0)
      A = M_PI/2;
    else
      A =  atan(z/L4);
    float B = acos(Lc/L5);
    float za = sqrt(pow(z,2)+pow(L4,2)-pow(Lc,2));
    if(leg == 0 || leg == 1){
      if(y<0)
        gama = (B-A)/M_PI*180;
      else if(y<=Lc)// 0 < y <= Lc
        gama = (A-B)/M_PI*180;
      else//y>Lc
        gama = 180-(A+B)/M_PI*180;
    }
    else//if(leg == 2 || leg == 3)
    {
      if(y>0)
        gama = (B-A)/M_PI*180;
      else if(y>= -Lc) //0 > y >= -Lc
        gama = (A-B)/M_PI*180;
      else//y < -Lc
        gama = 180-(A+B)/M_PI*180;
    }

    if(y<0)
      gama = 90+gama;
    else if(y>0)
      gama = 90-gama;  
  
    //后方的肩关节舵机是反向安装，也需要取反
    if(leg == 1 || leg == 3)
      gama = 180 - gama;
    z = za;
  }
  else
      gama = 90;

//  if(z == 0){
//    Serial.println("avoid crash z = 0 !!!!!!!!");
//    return;
//  }
  float a1 = (pow(La,2)-pow(Lb,2)+pow(x,2)+pow(z,2))/(2*La*(sqrt(pow(x,2)+pow(z,2))));//需要在考虑 x==0 z==0的情况
  float a2 = (pow(La,2)+pow(Lb,2)-pow(x,2)-pow(z,2))/(2*La*Lb);

  if(z == 0){
    if(x>0)
      alpha = acos(a1)+M_PI/2;
    else if(x<0)
      alpha = acos(a1)-M_PI/2;
    else //x == 0
      alpha = acos(a1);
  }
  else
    alpha = acos(a1)+atan(x/z);
  beta = acos(a2);
  //注意，这里的alpha是d+c角度，beta是夹角

  //弧度转为角度
  alpha = alpha/M_PI*180;//这时的alpha是d+c
  alpha = 180-alpha; //这是的alpha是上垂线和大腿的夹角
  beta = beta/M_PI*180;
//  if(beta>BETA_MAX)beta = BETA_MAX;
//  if(beta<BETA_MIN)beta = BETA_MIN;
//  Serial.printf("postion angle alpha=%f,beta=%f\n",alpha,beta);
 
#if SERV0_SPT_5835  
  //目前的安装是所有舵机90度安装，大腿垂直身体向下，大腿和小腿垂直
  //最新的安装方式改为大腿平行身体安装
  //beta =  小腿 - 大腿 + 90 
  //大腿为angleHam， 小腿为angleLeg
  float angleHam = 180-alpha；//+90;大腿改为平行身体安装
  float angleLeg = 180-alpha+beta;//angleHam + beta-90;
  if(leg == 2 || leg == 3)//右侧需要取反
  {  
    angleHam = 180 - angleHam;
    angleLeg = 180 - angleLeg;  
  }
#else//SPT_5435  SERVO_300_HAM
  
  #if SERVO_180_HAM //180度大腿的安装是所有舵机90度安装，大腿平行身体安装，大腿和小腿垂直，只是5435换向就行 180-5835的度数
  float angleHam = alpha;
  float angleLeg = alpha-beta;
  #else//SERVO_300_HAM  300度为大腿时安装是除了大腿所有舵机90度安装，大腿为150度安装，大腿和身体平行，大腿和小腿垂直
  float angleHam = 60+alpha;
  float angleLeg = alpha-beta+90;
//  Serial.printf("postion_to_angle alpha = %f,beta = %f,angleLeg = %f\n",alpha,beta,angleLeg);
  #endif
  if(leg == 2 || leg == 3)//右侧需要取反
  {
  #if SERVO_180_HAM    
    angleHam = 180 - angleHam;
  #else
    angleHam = 300 - angleHam;
  #endif
    angleLeg = 180 - angleLeg;      
  }
  gama = 180 - gama;//gama因为之前计算就按脚取反过了，这里只是5435的反向旋转对所有脚再取反一次
#endif
//  Serial.printf("postion angle angleHam=%f,angleLeg=%f\n",angleHam,angleLeg);
  angle_to_servo(leg,angleHam,angleLeg,gama);

//
//  if(gama != 90)
//  {
//    if(leg == 0 )
//      Serial.print("postion angle ");
//    String str = String(gama)+" ";
//    Serial.print(str);
//    if(leg == 3 )
//      Serial.println();
//  }

}

void angle_to_servo(int leg ,float alpha,float beta,float gama)
{
//  String s = "leg = "+ String(leg);
//  s += " alpha = "+String(alpha);
//  s+=" beta = "+String(beta);
//  s+=" gama = "+String(gama);
//  Serial.println(s);

  verifyAnagleMaxAndMin(leg,alpha,beta,gama);
  alpha = alpha+Legs_offset[leg][0];
  beta = beta+Legs_offset[leg][1];
  gama = gama+Legs_offset[leg][2]; 
#if SERVO_180_HAM
  setAngle(Legs_pin[leg][0],alpha);
#else //SERVO_300_HAM
  setAngle300(Legs_pin[leg][0],alpha);
#endif
  setAngle(Legs_pin[leg][1],beta); 
  setAngle(Legs_pin[leg][2],gama);
}

void verifyAnagleMaxAndMin(int leg,float& alpha,float& beta,float& gama)
{
  static float ham_max,ham_min,leg_max,leg_min,shoulder_max,shoulder_min;
  if(leg == 0 || leg == 1)//大腿和小腿 是0和1脚 一样的范围
  {
      ham_max = max_servo_ham_degree;
      ham_min = min_servo_ham_degree;
      leg_max = max_servo_leg_degree;
      leg_min = min_servo_leg_degree;
  }
  else
  {
#if SERVO_180_HAM    
      ham_max = 180 - min_servo_ham_degree;
      ham_min = 180 - max_servo_ham_degree;
#else//SERVO_300_HAM
      ham_max = 300 - min_servo_ham_degree;
      ham_min = 300 - max_servo_ham_degree;
#endif
      leg_max = 180 - min_servo_leg_degree;
      leg_min = 180 - max_servo_leg_degree;
  }

  //shoulder以90度为中线，各调整45度，所以每条腿都一样 45-135
  shoulder_max = max_servo_shoulder_degree;
  shoulder_min = min_servo_shoulder_degree;

  if(alpha>ham_max)alpha = ham_max;
  if(alpha<ham_min)alpha = ham_min;
  if(beta>leg_max)beta = leg_max;
  if(beta<leg_min)beta = leg_min;
  if(gama>shoulder_max)gama = shoulder_max;
  if(gama<shoulder_min)gama = shoulder_min;  
}

void set_current_postion(int leg, float x, float y, float z)
{
  curPostion[leg][0] = x;
  curPostion[leg][1] = y;
  curPostion[leg][2] = z;    
}

void set_expect_postion(int leg, float x, float y, float z)
{
  if(x != KEEP)
    expPostion[leg][0] = x;
  if(y != KEEP)  
    expPostion[leg][1] = y;
  if(z != KEEP)   
    expPostion[leg][2] = z;     
}

void set_expect_postion_all(float x, float y, float z)
{
    for(int i = 0;i<4;i++)
      set_expect_postion(i,x,y,z);
}

void set_expect_postion_front(float x, float y, float z)
{
  set_expect_postion(0,x,y,z);
  set_expect_postion(2,x,y,z);
}

void set_expect_postion_rear(float x, float y, float z)
{
  set_expect_postion(1,x,y,z);
  set_expect_postion(3,x,y,z);
}

void  check_expect_postion()
{
  static float alpha, beta, gama;
  int nReady=0;
  for(int i=0;i<4;i++)
  {
    nReady = 0;
    for(int j=0;j<3;j++)
    {
      if(curPostion[i][j]!= expPostion[i][j])
      {
        if(abs(curPostion[i][j] - expPostion[i][j])<stepSpeed)
        {
            curPostion[i][j] = expPostion[i][j];
        }
        else{
         if(curPostion[i][j] < expPostion[i][j])
           curPostion[i][j] += stepSpeed;
         else
           curPostion[i][j] -= stepSpeed;
        }
        postionReady[i][j] = 0; 
      }
      else{
        postionReady[i][j] = KEEP;
        continue;
      }
    }
    for(int j=0;j<3;j++)
    {
      nReady += postionReady[i][j];
    }
    if(nReady == 3*KEEP)//如果1条腿的3个节点已经驱动到位，就不要再计算舵机的角度了
      continue;
//    Serial.println("check_expect_postion action");  
    postion_to_angle(i,curPostion[i][0],curPostion[i][1],curPostion[i][2]);
 
  }
}

void wait_reach_postion(int leg)
{
  while (1){
    if (curPostion[leg][0] == expPostion[leg][0]){
      if (curPostion[leg][1] == expPostion[leg][1]){
        if (curPostion[leg][2] == expPostion[leg][2]){
          break;
        }
      }
    }
    vTaskDelay(10);
  }
}

void wait_all_reach_postion()
{
//  checkPostionOrAngle = 1;
  for (int i = 0; i < 4; i++)
    wait_reach_postion(i);
}

bool is_all_reach_postion()
{
  for (int i = 0; i < 4; i++)
  {
    if (curPostion[i][0] != expPostion[i][0])
       return false;
    if (curPostion[i][1] != expPostion[i][1])
       return false;
    if (curPostion[i][2] != expPostion[i][2])
       return false;   
  }
  return true;
}
void direct_postion(int nLeg,float x,float y,float z)
{
    set_current_postion(nLeg,x,y,z);
    set_expect_postion(nLeg,x,y,z);
    postion_to_angle(nLeg,x,y,z);  
}

void direct_all_postion(float x,float y,float z)
{
  for (int i = 0; i < 4; i++)
  {
    direct_postion(i,x,y,z);
  }
}

void direct_postion_front(float x,float y,float z)
{
  direct_postion(0,x,y,z);
  direct_postion(2,x,y,z);
}

void direct_postion_rear(float x,float y,float z)
{
  direct_postion(1,x,y,z);
  direct_postion(3,x,y,z);
}

void initMovePosition()
{
  Serial.println("initMove");
  
  int initX = 0;
  int initY = 0;
  int initZ = const_sit_high;
//  direct_all_postion(initX,initY,initZ);
  
  for(int i = 0;i<4;i++)
  {
    set_current_postion(i,initX,initY,initZ);
    set_expect_postion(i,initX,initY,initZ);
  }  
}
bool isStand()
{
  for(int i = 0 ;i<4;i++)
  {
    for(int j=0;j<3;j++)
    {
      if(curPostion[i][0] != 0)
        return false;
      if(curPostion[i][1] != 0)
        return false;       
      if(curPostion[i][2] != getStepStandHigh())
        return false;
    }
  }  
  return true;
}

bool isSit()
{
  for(int i = 0 ;i<4;i++)
  {
    for(int j=0;j<3;j++)
    {
      if(curPostion[i][2] != const_sit_high)
        return false;
    }
  }  
  return true;
}

float getCurPostionX(int leg)
{
  return curPostion[leg][0];
}

float getCurPostionY(int leg)
{
  return curPostion[leg][1];  
}
float getCurPostionZ(int leg)
{
  return curPostion[leg][2];  
}
float angle2pi(float a)
{
  float f;
  f = a/180*M_PI;
  return f;
}
float pi2angle(float p)
{
  float a;
  a = p/M_PI*180;
  return a;
}
