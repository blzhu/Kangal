#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps_V6_12.h"
// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
//#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
//#include "Wire.h"
//#endif
#define mpu_avg 10  
#define DIVISION 9//4 //1/4 = 1/( (10/2)-1), 和 MPUThread里的时间设定有关
MPU6050 mpu;

float ypr_array[mpu_avg][3]={{0},{0},{0}}; 

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 gy;         // [x, y, z]            gyro sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}

void stopMPU()
{
  Serial.println("stop mpu");
  dmpReady = false;
  mpuStatusReported = 0;  
  mpu.resetFIFO();
  Wire.flush();
}
void initMPU()
{
  // join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif
  dmpReady = false;
  mpuStatusReported = 0;
  // initialize device
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();
  mpu.resetFIFO();
  
  // verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  // load and configure the DMP
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(0);
  mpu.setYGyroOffset(0);
  mpu.setZGyroOffset(0);
  mpu.setXAccelOffset(0);
  mpu.setYAccelOffset(0);
  mpu.setZAccelOffset(0);
  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // Calibration Time: generate offsets and calibrate our MPU6050
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    Serial.println();
    mpu.PrintActiveOffsets();
    // turn on the DMP, now that it's ready
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }  
}

void checkMpuData()
{
  // if programming failed, don't try to do anything
  if (!dmpReady) return;
  
//  wdt_reset();
  mpuIntStatus = mpu.getIntStatus();
  fifoCount = mpu.getFIFOCount();
//  Serial.printf("fifoCount = %d\n",fifoCount);
  if ((mpuIntStatus & _BV(MPU6050_INTERRUPT_FIFO_OFLOW_BIT)) || fifoCount >= 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        fifoCount = mpu.getFIFOCount();
        Serial.printf("FIFO overflow! %d\n",fifoCount);
    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    return;
  }
  // read a packet from FIFO
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet 
        // display Euler angles in degrees
    fifoCount -= mpu.dmpGetFIFOPacketSize();
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    static int indexOfypr=0;
    ypr_array[indexOfypr][0] = ypr[0];
    ypr_array[indexOfypr][1] = ypr[1];
    ypr_array[indexOfypr][2] = ypr[2];  
    indexOfypr++;
    if(indexOfypr>=mpu_avg)
      indexOfypr = 0;
    if(mpuStatusReported == 0)
    {
      mpuStatusReported = 1;
      BeepWarning = 2;
    
      Serial.print("ypr\t");
      Serial.print(ypr[0] * 180 / M_PI);
      Serial.print("\t");
      Serial.print(ypr[1] * 180 / M_PI);
      Serial.print("\t");
      Serial.print(ypr[2] * 180 / M_PI);
    
      mpu.dmpGetAccel(&aa, fifoBuffer);
      Serial.print("\tRaw Accl XYZ\t");
      Serial.print(aa.x);
      Serial.print("\t");
      Serial.print(aa.y);
      Serial.print("\t");
      Serial.print(aa.z);
      mpu.dmpGetGyro(&gy, fifoBuffer);
      Serial.print("\tRaw Gyro XYZ\t");
      Serial.print(gy.x);
      Serial.print("\t");
      Serial.print(gy.y);
      Serial.print("\t");
      Serial.print(gy.z);
      Serial.printf("temperature = %d",mpu.getTemperature()); 
      Serial.println();
    }
  }
  else
  {
//    mpuStatusReported = 0;
//    Serial.println("no MPU data");
  }
  
}

bool isMpuWorking()
{
  if(mpuStatusReported && dmpReady)
    return true;
  
  return false;
    
}
void checkBalance()//调整身体姿态，保持相对静止
{
  if(!isMpuWorking())
    return;
  if(!bNeedBalance)
    return;

  float yaw,roll,pitch;
  yaw = pi2angle(ypr[0]);//(getAvgYaw());
  roll = pi2angle(-ypr[1]);//(getAvgRoll());//本机的  和 mpu的 设定相反，所以取负值
  pitch = pi2angle(ypr[2]);//(getAvgPitch());

//  yaw = pi2angle(getAvgYaw());
//  roll = pi2angle(-getAvgRoll());//本机的  和 mpu的 设定相反，所以取负值
//  pitch = pi2angle(getAvgPitch());

  if(abs(yaw) < 2)yaw = 0;
  if(abs(roll) < 1)roll = 0;
  if(abs(pitch) < 1)pitch = 0;  
  if(yaw == 0 && roll == 0 && pitch == 0)
    return;
  if(yaw>max_yaw)yaw=max_yaw;
  if(yaw<-max_yaw)yaw=-max_yaw;
  if(roll>max_roll)roll=max_roll;
  if(roll<-max_roll)roll=-max_roll;
  if(pitch>max_pitch)pitch=max_pitch;
  if(pitch<-max_pitch)pitch=-max_pitch; 

  yaw =  body_attitude[0]- yaw/DIVISION;// 
  roll =  body_attitude[1]- roll/DIVISION;
  pitch =  body_attitude[2]- pitch/DIVISION;    


//  String s = "rotate?y="+String(yaw)+"&r="+String(roll)+"&p="+String(pitch)+"&x=0&z=0";
//  Serial.println(s);
  actionRotate(yaw,roll,pitch,0,0,0);
  Serial.printf("checkBalance y = %f,r = %f,p = %f\n",yaw,roll,pitch);
}

float getAvgYaw()
{
  float sum = 0;
  for(int i=0;i<mpu_avg;i++)
  {
    sum += ypr_array[i][0];
  }
  return sum/mpu_avg;  
}

float getAvgRoll()
{
  float sum = 0;
  for(int i=0;i<mpu_avg;i++)
  {
    sum += ypr_array[i][1];
  }
  return sum/mpu_avg;  
}

float getAvgPitch()
{
  float sum = 0;
  for(int i=0;i<mpu_avg;i++)
  {
    sum += ypr_array[i][2];
  }
  return sum/mpu_avg;  
}

//从后方看向左侧翻滚 roll>0 , 从左侧看 抬头是pitch>0
void keepStepBalance()//双脚抬起，保持平衡
{
  if(!isMpuWorking())
    return; 
  direct_postion(0,0,0,100);
  direct_postion(3,0,0,100);

  static float y = 0;
  static float lastP=0;
  static float lastR=0;
  float pitch,roll;
  pitch = pi2angle(ypr[2]);
  roll = pi2angle(ypr[1]);
  if(abs(pitch)>0.5 || abs(roll)>0.5 )
  {
    if(roll>0 && pitch<0){
      y--;
    }
    else if(roll<0 && pitch>0)
    {
      y++;
    }
//    if(lastP>pitch)
//    {
//      y--;
//      if(y<-50)y=-50;
//    }
//    else
//    {
//      y++;
//      if(y>50)y=50;
//    }
  }
  else
  {

  }
  if(y<-80)y=-80;
  else if(y>80)y = 80;
  
  lastP = pitch;
  lastR = roll;
  direct_postion(1,0,y,150);
  direct_postion(2,0,y,150);  
  Serial.printf("keepStepBalance y = %f,r = %f,p = %f\n",y,roll,pitch);
}
