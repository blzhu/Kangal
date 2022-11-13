#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <WiFi.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <PS4Controller.h>
#include <arduino.h>
#include "esp32-hal-dac.h"
//#include <AsyncTCP.h>
//#include <ESPAsyncWebServer.h>
#include "FS.h"
#include "SPIFFS.h"
#include "EEPROM.h"
////测试使用定时器比较freeRTOS
#define USE_FREERTOS 1
#define SERVO_RUN 1//舵机上电，真实测试，0 就跑逻辑，舵机不动
#define SERVO_TEST 0//1 是舵机测试，上电就跑舵机测试程序
#define SERVO_AUTO_TEST 0//不用遥控器来测试简单的动作
#define SERV0_SPT_5835 0 //5835为正向旋转,是大多数舵机的旋转方向
#define SERVO_180_HAM 0 //180度舵机 作为大腿，多少用180
#define SERV0_SPT_5435 1 //5435为反向旋转，比较特殊，基本就只有这种舵机是这个旋转方向
#define SERVO_300_HAM 1 //5435-320是300度舵机，作为大腿，可以提高旋转角度，有助于活动范围扩大
#define ONLY_TEST_FIRST_LEG 0        
#define USE_COORDINATES_CONVERSION 1   //使用坐标变换，身体坐标为world坐标系，足端为B坐标系      
#define WITHOUT_NEW 0                                                                    
#if USE_FREERTOS
//hw_timer_t * threadTimerHandle = NULL;
//volatile SemaphoreHandle_t timerSemaphore;
//portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;
#else
#include <Ticker.h>
Ticker timerServo;
Ticker timerCmd;
Ticker timerUtility;
Ticker timerAction;

#endif
//hardcode value
#define STAND_HIGH 150
#define SIT_HIGH 70
#define LIFT_HIGH 20
const float const_sit_high = 70;
const float const_max_stand_high = 180;
const float const_body_width = 160;
const float const_body_length = 200;
///config value in flash
int wifiConfigMode = 0;//0 表示 用PS4来遥控，1表示用wifi来配置
int moveOffsetX = 0;//前后平移，用来平衡前后重心,这个不鞥这样设，要重新设计

//===========
int curWifiMode = 0;
int new_cycle_time = 0;
int new_lift_high = 0;
int new_stand_high = 0;
int intvalServo = 10;
int intvalCmd = 20;
int intvalMpu = 20;
int intvalUtility = 10;
int intvalUDP = 10;
int intvalAction = 20;
int intvalBeepIdle = 20;
int intvalBeepWarning = 50;

int muteBeep = 0;
WiFiUDP Udp;
unsigned int localUdpPort = 80;  // local port to listen on
//AsyncWebServer server(80);
int hadHandleRoot = 0;
char incomingPacket[255];  // buffer for incoming packets
char sendingPacket[255];
String cmdStr;
String textHTML;
float directX = 0;
float directY = 0;
float directZ = 0;
/////
#define speakerPin 26 //PCB_v9
#define oeSwitchPin 25
#define ledPin1 5
#define ledPin2 18
#define ledPin3 19
#define ledPin4 23
const int currentPin[]={33,35,32,34};//电流检测pin 

int servoThreadMode = 0;//0 表示postion 模式，1 表示周期模式之 trot步态 ,2 为跳跃，3为walk
int BeepWarning = 0;//0 默认，1 就是一直响
int BeepVolume = 150;
int isServoAttached = false;
//SemaphoreHandle_t xSemaphore = NULL;

//接线方式 ，PWM是有一个 用0x40 默认值,接线方式经常变化，PWM端口也经常变化
//左前脚0 大腿，小腿，肩关节 分别对应 15，14，11  三个 PWM 口       
//左后腿1 大腿，小腿，肩关节 分别对应 13，12，10  三个 PWM 口  
//右前退2 大腿，小腿，肩关节 分别对应 0，1，4 三个 PWM 口   
//右后退3 大腿，小腿，肩关节 分别对应 2，3，5 三个 PWM 口    

//          腿2 -大腿 - 小腿- 关节     关节-小腿- 大腿 -腿1
//          7 -  6  -  5 -  4        0 -  1 -  2 - 3
//
//          8  -  9 -   10 - 11       15 -  14  - 13 - 12
//          腿4 - 大腿 -小腿 -关节      关节3- 小腿 -大腿 - 腿3
//

//腿的引脚定义 0，1，2，3 分别为 左前，左后， 右前，右后 ，每条腿按 大腿，小腿，肩关节的顺序定义
//PCB_v9
const int Legs_pin[4][3] = { {2,1,0}, {6,5,4}, {13,14,15}, {9,10,11} };

//PCB_v10 腿的引脚定义 内侧为大腿，中间为小腿，外侧为关节，数组顺序为 大、小、关
//const int Legs_pin[4][3] = { {2,1,0}, {5,4,3}, {13,14,15}, {10,11,12} };

int Legs_offset[4][3] = { {0, 0, 0},    //0 
                                {0, 0, 0},    //1 
                                {0, 0, 0},   //2 
                                {0, 0, 0} };    //3 
#define num_avg 20                               
float Legs_current[4][num_avg] = {{0},{0},{0},{0}};
#define max_yaw 30
#define max_roll 25
#define max_pitch 20
float body_attitude[5]={0};//yaw , roll ,pitch, dx,dz
bool bNeedBalance = false;
#define SERVO_FREQ 330 // Analog servos run at ~50 Hz updates
#define SERVO_INTERVAL 5
#define MAX_ANGLE 180

int btStatusReported  = 0;
int mpuStatusReported = 0;
int OTAStarted = 0;
int OTAInited = 0;
int UDPStarted = 0;
const char* ssid = "ssid";
const char* password = "12345678";
const char* ssid_config = "kangle";
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

/////

void setup() {
  // put your setup code here, to run once:
//  pinMode(speakerPin,OUTPUT);
//  pinMode(oeSwitchPin,OUTPUT);
//  pinMode(ledPin1,OUTPUT);
//  pinMode(ledPin2,OUTPUT);
//  pinMode(ledPin3,OUTPUT);
//  pinMode(ledPin4,OUTPUT);
  
  Serial.begin(115200);
  Serial.println("Start robot !!!!!");
  Serial2.begin(115200);
  showBluetoothAddr();
//  clearBlueToothIfNeed();
//  return;
//  detachServo();
  initCurrentPin();//PCB_v9
  initFileSys();  
  readConfig();
//  saveConfig(0);
  initPWM();
  curWifiMode = wifiConfigMode;
//  wifiConfigMode =1;
  if(wifiConfigMode == 0)
  {
    MakeBeep(1);
    initPS4();
  }
  else if(wifiConfigMode == 1)
  {
    initUDP();
//    initWifi();webserver 导致一些异常问题，弃用
    MakeBeep(2);
    wifiConfigMode = 0;//wifi mode 开一次之后下次重启就不开了
    saveConfig(1);
  }
  else if(wifiConfigMode == 2)
  {
    initOTA();
    MakeBeep(3);
    wifiConfigMode = 0;//OTA mode 开一次之后下次重启就不开了
    saveConfig(1);   
  }
    
  
  initMovePosition();
//  initMPU();
  startThreads();
//  MakeBeep(2);

  
}
void checkOTA()
{
  if(OTAInited)
    ArduinoOTA.handle();
}

void loop() {
//  checkMpuData();

#if SERVO_TEST
  servoTest();
#endif

#if SERVO_AUTO_TEST
  servoAutoTest();
#endif

}


void  setServoPulse(uint8_t n, double pulse) {
  if(n<0 ||n >=16)
    return;
  double pulselength;
  pulselength = 1000000;   // 1,000,000 us per second
  pulselength /= SERVO_FREQ;   // Analog servos run at ~60 Hz updates
  pulselength /= 4096;  // 12 bits of resolution

  pulse /= pulselength;
#if SERVO_RUN  //0 测试，不会驱动舵机
  pwm.setPWM(n,0,pulse);
#endif

}

void  setAngle(int n,double angle)
{
   if(n<0 || n>=16)
    return;
   double pulse = angle;
   pulse = pulse/90 + 0.5;
   pulse *= 1000;
   setServoPulse(n,pulse);//0到180度映射为0.5到2.5ms
//   if(n == 1||n==2)
//   Serial.printf("servo num=%d, angle = %f, pulse = %f\n",n,angle,pulse);
}

void setAngle300(int n,double angle)
{
   double pulse = angle;
   pulse = pulse/150 + 0.5;
   pulse *= 1000;
   setServoPulse(n,pulse);//0到300度映射为0.5到2.5ms
}


void initOTA()
{
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.println("Connection Failed! Rebooting...");
    delay(5000);
    ESP.restart();
  }
  ArduinoOTA
    .onStart([]() {
      String type;
      if (ArduinoOTA.getCommand() == U_FLASH)
        type = "sketch";
      else // U_SPIFFS
        type = "filesystem";

      // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
      Serial.println("Start updating " + type);
    })
    .onEnd([]() {
      Serial.println("\nEnd");
    })
    .onProgress([](unsigned int progress, unsigned int total) {
      Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    })
    .onError([](ota_error_t error) {
      Serial.printf("Error[%u]: ", error);
      if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
      else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
      else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
      else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
      else if (error == OTA_END_ERROR) Serial.println("End Failed");
    });

  ArduinoOTA.begin();
  OTAInited = 1;
  Serial.println("Ready");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());  
}

void initPS4()
{
  Serial.println("initPS4");  
//  clearBlueToothIfNeed(); 
//  PS4.begin("04:04:04:04:04:04");
  PS4.begin();
}
void initPWM()
{
  pwm.begin();
  //pca9685 use 25MHz
  pwm.setOscillatorFrequency(25000000);

  pwm.setPWMFreq(SERVO_FREQ);  

  Serial.println("start PWM");   
}

void initUDP()
{
  WiFi.softAP(ssid_config);
  Udp.begin(localUdpPort);
  IPAddress myIP = WiFi.softAPIP();
  Serial.print("AP IP address: ");  
  Serial.println(myIP);
  Serial.println("start UDP"); 
  UDPStarted = 1;
}

void startThreads()
{
    Serial.println("start Threads"); 

#if USE_FREERTOS    
    CreateFreeRTOS();
#else
    timerServo.attach_ms(intvalServo,timer_process_servo);
    timerCmd.attach_ms(intvalCmd,timer_process_cmd);//upd,ps4
    timerUtility.attach_ms(intvalAction,timer_process_utility);//beep,mpu
    timerAction.attach_ms(intvalAction,timer_process_action);
#endif
}

void CreateFreeRTOS()
{
    xTaskCreatePinnedToCore(
    ServoThread
    ,  "ServoThread"
    ,  10240  // Stack size
    ,  NULL
    ,  2  // Priority
    ,  NULL ,1);
   
    xTaskCreatePinnedToCore(
    CMDThread
    ,  "CMDThread"
    ,  10240  // Stack size
    ,  NULL
    ,  2  // Priority
    ,  NULL ,0);  
    
    xTaskCreatePinnedToCore(
    ActionThread
    ,  "ActionThread"
    ,  4096  // Stack size
    ,  NULL
    ,  2  // Priority
    ,  NULL ,0);
    
    
    xTaskCreatePinnedToCore(
    UtilityThread
    ,  "UtilityThread"
    ,  4096  // Stack size
    ,  NULL
    ,  2  // Priority
    ,  NULL ,0);   

    xTaskCreatePinnedToCore(
    MPUThread
    ,  "MPUThread"
    ,  1024  // Stack size
    ,  NULL
    ,  2  // Priority
    ,  NULL ,0);   
}
void ServoThread(void *pvParameters) 
{
  (void) pvParameters;
  for (;;)
  {
//    Serial.println("ServoThread"); 
    timer_process_servo();
    vTaskDelay(intvalServo);
  }
  vTaskDelete(NULL);
}

void CMDThread(void *pvParameters) 
{
  (void) pvParameters;
  for (;;)
  {
//    Serial.println("CMDThread"); 
    timer_process_cmd();
    vTaskDelay(intvalCmd);
  }
  vTaskDelete(NULL);
}

//void UDPThread(void *pvParameters) 
//{
//  (void) pvParameters;
//  for (;;)
//  {
//    check_UdpCmd();
//    vTaskDelay(intvalUDP);
//  }
//  vTaskDelete(NULL);
//}

void ActionThread(void *pvParameters) 
{
  (void) pvParameters;
  for (;;)
  {
//    Serial.println("ActionThread"); 
    timer_process_action();
    vTaskDelay(intvalAction);
  }
  vTaskDelete(NULL);
}
void UtilityThread(void *pvParameters) 
{
  (void) pvParameters;
  for (;;)
  {
//    Serial.println("UtilityThread"); 
    timer_process_utility();
    vTaskDelay(intvalUtility);
  }
  vTaskDelete(NULL);
}

void MPUThread(void *pvParameters) 
{
  (void) pvParameters;
  for (;;)
  {
    checkMpuData();
    checkBalance();
//    keepStepBalance();
    vTaskDelay(intvalMpu);
  }
  vTaskDelete(NULL);  
}

void initMpuThread(void *pvParameters) 
{
  (void) pvParameters;
  initMPU();
  vTaskDelete(NULL);  
}
//
//void BeepThread(void *pvParameters) 
//{
//  (void) pvParameters;
//  for (;;)
//  {
//    if(BeepWarning)
//    {
//      dacWrite(speakerPin,BeepVolume); 
//      vTaskDelay(intvalBeepWarning);
//      dacWrite(speakerPin,0); 
//      vTaskDelay(intvalBeepWarning);
////      Serial.println("BeepThread!");
//    }
//    else
//    {
//      vTaskDelay(intvalBeepIdle);
//    }
//  }
//  vTaskDelete(NULL);  
//}

void MakeBeep(int nCount)
{
  if(muteBeep)return; 
  if(nCount<=0)
    return;
  for(int i = 0;i<nCount;i++){
    dacWrite(speakerPin,BeepVolume); 
    delay(100);
    dacWrite(speakerPin,0); 
    delay(100);
  }
}

void MuteBeep()
{
  if(muteBeep)return;
  dacWrite(speakerPin,0); 
  BeepWarning = 0;
}
void MakeWarning()
{
  if(muteBeep)return;
  for(int i = 0;i<10;i++){
    dacWrite(speakerPin,BeepVolume); 
    delay(50);
    dacWrite(speakerPin,0); 
    delay(50);
  }
}
void send_packet(const char* data)
{
    Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
    Udp.print(data);
    Udp.endPacket();
}
void check_SerialCmd()
{
//  if (!Serial2.available())
//    return;
//  if (!Serial.available())
//    return;
    
  String cmd;
  while (Serial.available() > 0){
     cmd = Serial.readString();
     cmd.trim();
     Serial.println(cmd);
     cmdStr = cmd;
  }  
//  while (Serial2.available() > 0){
//     cmd = Serial2.readString();
//     Serial.print("Serial2 cmd ");
//     Serial.println(cmd);
//     cmdStr = cmd;     
//  }

}

void check_UdpCmd()
{
  if(!UDPStarted)
    return;
  int packetSize = Udp.parsePacket();  
  if (packetSize)
  {
    int len = Udp.read(incomingPacket, 255);
    if (len > 0)
    {  
      incomingPacket[len] = 0;
      String cmd = String(incomingPacket);
      if(cmd.equals("heartbeat"))
      {
        send_packet(incomingPacket);
      }             
      else
      {
        Serial.print("check_cmd ");
        Serial.println(cmd);
        cmdStr = cmd;
      }
    }
  }
}


void timer_process_servo()
{
   if(servoThreadMode == 0)//位置模式
     check_expect_postion();
   else if(servoThreadMode == 1)//周期模式trot
      check_cycle_trot();  
   else if(servoThreadMode == 2)//跳跃模式
      check_cycle_jump();
//   else if(servoThreadMode == 3)//walk模式
//      check_cycle_walk();
   else if(servoThreadMode == 3)//walk模式
      check_cycle_walk_ex(); 
   else if(servoThreadMode == 4)//后空翻模式
      check_cycle_backflip();
}

void timer_process_cmd()
{
  check_UdpCmd();
  checkPS4();
  check_SerialCmd();
}


void timer_process_utility()
{
  static int utilCount = 0;
  if(utilCount%5 == 0){
  checkCurrent();//PCB_v9
  checkBeep();
  checkServoAttach();
  checkPS4Battery();
  checkWifiCount();
  checkLED();
  checkOTA();
  }  
  
  utilCount++;
}

void timer_process_action()
{
    if(cmdStr.length()==0)
      return;
//    Serial.println(cmdStr.c_str());
    String cmd = cmdStr;
    cmdStr = "";
    int count = countSplitCharacters(cmd,'?');//兼容web命令 cmd?key1=value1&key2=value2
    if(count>0)//只有 1 个问号
    {
      String value,param;
      value = getSplitValue(cmd,'?',0);
      param = getSplitValue(cmd,'?',1);
      if(value.equals("s"))//setting
      {
        doSetting(param);
      }
      else if(value.equals("offset"))
      {
        doOffset(param);
      }
      else if(value.equals("ro"))
      {
        requestOffset(param);
      }
      else if(value.equals("jump"))
      {
        doJump(param);
      }
      else if(value.equals("rotate"))
      {
        doRotate(param);
      }
      
      return;
    }
    if(cmd.equals("sit"))
    {
      actionSit();
    }   
    else if(cmd.equals("stand"))
    {
      actionStand();
//      wait_all_reach_postion();
//      actionSit();
//      wait_all_reach_postion();
//      actionStand();
//      wait_all_reach_postion();
//      actionSit();
//      wait_all_reach_postion();
    }
    else if(cmd.equals("init"))
    {
      initAllLegs();
    }
    else if(cmd.equals("initzero"))
    {
      initAllLegsZero();
    }
//    else if(cmd.equals("testall"))
//    {
//      testAllServo();
//    }       
//    else if(cmd.equals("teststand"))
//    {
//      standTest();
//    }
    else if(cmd.equals("directTo"))
    {
      actionDirectTo(directX,directY,directZ);
    }  
    else if(cmd.equals("left"))
    {
      actionLeft();
    }  
    else if(cmd.equals("right"))
    {
      actionRight();
    }   
    else if(cmd.equals("ps4connect"))
    {
//      MakeBeep(3);
      BeepWarning = 3;
    }         
    else if(cmd.equals("startTrot"))
    {
      if(isTrotRunning())
      {
        stopTrot();
      }
      else if(isStand())
        startTrot();
      else
        actionStand();
    }
    else if(cmd.equals("stopTrot"))
    {
       stopTrot();
    }    
    else if(cmd.equals("startWalk"))
    {
      Serial.println("cmd startWalk");
      if(isWalkRunning()){
        stopWalk();
        actionStand();
      }
      else if(isStand())
        startWalk();
      else
        actionStand();
    }
    else if(cmd.equals("sc"))
    {
      saveAllconfig();
    }
    else if(cmd.equals("initMpu"))
    {
      xTaskCreatePinnedToCore(
      initMpuThread
      ,  "initMpuThread"
      ,  4096  // Stack size
      ,  NULL
      ,  2  // Priority
      ,  NULL ,1);       
    }
    else if(cmd.equals("bainian"))
    {
      actionBainian();
//      actionLie();
    }
    else if(cmd.equals("wait"))
    {
      actionWait();
    }
    else if(cmd.equals("backflip"))
    {
//      actionBackflip();
      start_backflip();
    }
}

void checkBeep()
{
  if(muteBeep)
    return;  
  static int i = 0;
    if(BeepWarning)
    {
      if(i%2==0)
      {
        dacWrite(speakerPin,BeepVolume); 
        BeepWarning--;
      }
      else
        dacWrite(speakerPin,0); 
    }
    else
      dacWrite(speakerPin,0); 
    i++;
}

 void checkPS4Battery()
 {
  if(curWifiMode>0)
    return;
  static int checkCount=0;
  checkCount++; 
  if(checkCount%1000 == 0)
  {
    if (PS4.isConnected() && PS4.Battery()<1) 
    {
      Serial.println("PS4 Low Battery");
      BeepWarning = 10;
//      MakeBeep(4);
    }
  }
 }

 void checkWifiCount()
 {
    
  if(curWifiMode==0)
    return;
  static int wifiConnectCount = 0;
  if(wifiConnectCount != WiFi.softAPgetStationNum())
  {
    if(wifiConnectCount<WiFi.softAPgetStationNum())//连接增加
      BeepWarning = 1;
    else//连接减少
      BeepWarning = 2;
    wifiConnectCount = WiFi.softAPgetStationNum();
    Serial.print("wifiConnectCount change current=");
    Serial.println(wifiConnectCount);
  }
 }



 void doSetting(String str)//s?lh=140
 {
   String key,value;
   key = getSplitValue(str,'=',0);
   value = getSplitValue(str,'=',1);
   if(key.equals("lh"))
   {
      new_lift_high = value.toInt();
   }
   else if(key.equals("sh"))
   {
      new_stand_high = value.toInt();
   }
   else if(key.equals("ct"))
   {
      new_cycle_time = value.toInt();
   }
 }

void requestOffset(String str)//ro?l=0&init=0
{
  String leg,bInit;
  Serial.println(str);
//  int count = countSplitCharacters(str,'&');
//  Serial.println(count);
  leg = getSplitValue(str,'&',0);
  leg = getSplitValue(leg,'=',1);
  bInit = getSplitValue(str,'&',1);
  bInit = getSplitValue(bInit,'=',1);
  int nLeg = leg.toInt();
//  Serial.println(leg);
  str = "offset?o="+String(Legs_offset[nLeg][0])+";";
  str += String(Legs_offset[nLeg][1])+";";
  str += String(Legs_offset[nLeg][2])+";";
  Serial.println(str);
  send_packet(str.c_str());
  int nInit = bInit.toInt();
  if(nInit)
  {
    initLeg(nLeg);
  }
  
}
void doOffset(String str)//"offset?l=0&s=0&o=0"
{
  int count = countSplitCharacters(str,'&');
  if(count != 3)
  {
    Serial.println("doOffset count error");
    return;
  }
  Serial.println(str);
  String leg,servo,offset,bInit;
  
  leg = getSplitValue(str,'&',0);
  leg = getSplitValue(leg,'=',1);

  servo = getSplitValue(str,'&',1);
  servo = getSplitValue(servo,'=',1);     

  offset = getSplitValue(str,'&',2);
  offset = getSplitValue(offset,'=',1);     

  bInit = getSplitValue(str,'&',3);
  bInit = getSplitValue(bInit,'=',1); 

  Serial.println(leg);
  Serial.println(servo);
  Serial.println(offset);
  adjustLegOffset(leg.toInt(),servo.toInt(),offset.toInt(),bInit.toInt()); 
}

void doJump(String str)//jump?d=1
{
   String key,value;
   key = getSplitValue(str,'=',0);
   value = getSplitValue(str,'=',1);   
   int d = value.toInt();

//   if(isStand())
//    actionJump(d,false);
//   else if(isSit())
//    actionJump(d,true);

  start_jump(d);

}

void doRotate(String str)//rotate?y=10&r=14&p=17&x=20&y=15&z=10
{
  String yaw,roll,pitch,movex,movey,movez;
  yaw = getSplitValue(str,'&',0);
  yaw = getSplitValue(yaw,'=',1);
  
  roll = getSplitValue(str,'&',1);
  roll = getSplitValue(roll,'=',1);

  pitch = getSplitValue(str,'&',2);
  pitch = getSplitValue(pitch,'=',1);

  movex = getSplitValue(str,'&',3);
  movex = getSplitValue(movex,'=',1);

  movey = getSplitValue(str,'&',4);
  movey = getSplitValue(movey,'=',1);
  
  movez = getSplitValue(str,'&',5);
  movez = getSplitValue(movez,'=',1);
  actionRotate(yaw.toFloat(),roll.toFloat(),pitch.toFloat(),movex.toFloat(),movey.toFloat(),movez.toFloat());
  
}

 void initCurrentPin()
 {
    for(int i=0;i<4;i++){
      pinMode(currentPin[i],INPUT);
    }
    analogSetWidth(10);//12位采样，最大4096
 }
 void checkCurrent()
 {//按照数据手册上10A规格的电流芯片，330mv-2970mv之间并线性 132mV = 1A  VCC/2 == 0A， https://item.szlcsc.com/475734.html
    int mv,value;
    float cur;
    static short indexOfcur = 0;
    for(int i=0;i<4;i++){
      value = analogRead(currentPin[i]);
      mv = map(value,0,1024,330,2970);//转化成电压 
      mv = mv -(3300)/2;
      cur = (float)mv/132;
      Legs_current[i][indexOfcur] = cur;
//      if(i == 0)
//        Serial.printf("leg %d , value = %d, mv = %d, cur = %f\n",i,value,mv,cur);
    }
    indexOfcur++;
    if(indexOfcur>=num_avg)
      indexOfcur = 0;

//    Serial.printf("0 leg cur = %f\n",getLegCur(0));  
 }

 float getLegCur(int nLeg)
 {
    float sum = 0;
    for(int i=0;i<num_avg;i++)
    {
      sum += Legs_current[nLeg][i];
    }

    return sum/num_avg;
 }
 void attachServo()
 {
    isServoAttached = true;
    digitalWrite(oeSwitchPin,LOW);
 }

 void detachServo()
 {
    isServoAttached = false;
    digitalWrite(oeSwitchPin,HIGH);
 }

 void checkServoAttach()
 {
    if(isServoAttached)
      attachServo();
    else
      detachServo();
 }

 void checkLED()
 {
    digitalWrite(ledPin1,HIGH);
    digitalWrite(ledPin2,HIGH);
    digitalWrite(ledPin3,HIGH);
    digitalWrite(ledPin4,HIGH);            
 }

 void servoTest()
 {
  static float angle=0;
  static float angle2 = 0;
  static bool bAdd=true;
  static bool bAdd2=true; 
//  BeepWarning = 1;

    if(angle<=0 && !bAdd)
      bAdd = true;
    else if(angle>=180 && bAdd)
      bAdd = false; 
    for(int i=0;i<8;i++)
    {
      setAngle(i,angle);
    }

    if(bAdd)
      angle+=1;
    else
      angle-=1;
    //////
    if(angle2<=0 && !bAdd2)
      bAdd2 = true;
    else if(angle2>=300 && bAdd2)
      bAdd2 = false;   
    for(int i=8;i<16;i++)
    {
      setAngle300(i,angle2);
    }       
    if(bAdd2)
      angle2+=1;
    else
      angle2-=1;  
  //  Serial.printf("angle = %d\n",angle);
 }

void servoAutoTest()
{
  actionSit();
  delay(5000);
  actionStand();
  delay(5000);
}
 
