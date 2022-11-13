
short PS4_UP = 0;
short PS4_DOWN = 0;
short PS4_LEFT = 0;
short PS4_RIGHT = 0;
short PS4_UPRIGHT = 0;
short PS4_UPLEFT = 0;
short PS4_DOWNRIGHT = 0;
short PS4_DOWNLEFT = 0;
short PS4_SQUARE = 0;
short PS4_CROSS = 0;
short PS4_CIRCLE = 0;
short PS4_TRIANGLE = 0;
short PS4_L1 = 0;
short PS4_R1 = 0;
short PS4_L2 = 0;
short PS4_R2 = 0;
short PS4_L3 = 0;
short PS4_R3 = 0;
short PS4_SHARE = 0;
short PS4_OPTION = 0;
short PS4_PS = 0;
short PS4_TOUCH = 0;
short PS4_LSTICK_X = 0;
short PS4_LSTICK_Y = 0;
short PS4_RSTICK_X = 0;
short PS4_RSTICK_Y = 0;


int LStickXarray[num_avg]={0};
int LStickYarray[num_avg]={0};
int RStickXarray[num_avg]={0};
int RStickYarray[num_avg]={0};
int R2array[num_avg] = {0};
int L2array[num_avg] = {0};
int r = 255;
int g = 0;
int b = 0;

bool bSettingStandHigh = false;
bool bRotating = false;
bool bMpuInited = false;
// Calculates the next value in a rainbow sequence
void  nextRainbowColor()
{
  
  if (r > 0 && b == 0) {
    r--;
    g++;
  }
  if (g > 0 && r == 0) {
    g--;
    b++;
  }
  if (b > 0 && g == 0) {
    r++;
    b--;
  }
  
}

//send data to PS4
void feedbackToPS4()
{
  if (PS4.isConnected()) {
    // Sets the color of the controller's front light
    // Params: Red, Green,and Blue
    // See here for details: https://www.w3schools.com/colors/colors_rgb.asp
    PS4.setLed(r, g, b);
    nextRainbowColor();

    // Sets how fast the controller's front light flashes
    // Params: How long the light is on in ms, how long the light is off in ms
    // Range: 0->2550 ms, Set to 0, 0 for the light to remain on
    PS4.setFlashRate(PS4.LStickY() * 10, PS4.RStickY() * 10);

    // Sets the rumble of the controllers
    // Params: Weak rumble intensity, Strong rumble intensity
    // Range: 0->255
    PS4.setRumble(PS4.R2Value(), PS4.L2Value());

    // Sends data set in the above three instructions to the controller
    PS4.sendToController();

    // Don't send data to the controller immediately, will cause buffer overflow
//    delay(10); use vTaskDelay in freertos
  }  
}



void checkPS4()
{
    // Below has all accessible outputs from the controller
    if(!PS4.isConnected()) 
      return;
//    Serial.println("ps4connect.....");
    if(btStatusReported == 0)
    {
      btStatusReported = 1;
      cmdStr = "ps4connect";
      Serial.println("ps4connect");
    }
    static short indexOfarray = 0;
    
    if (PS4.Left()) {
      PS4_LEFT++;
    }
    else
      PS4_LEFT = 0;
      
    if (PS4.Right()) {
      PS4_RIGHT++;
    }
    else
      PS4_RIGHT = 0;
      
    if (PS4.Down()) {
      PS4_DOWN++;
    }
    else
      PS4_DOWN = 0;
      
    if (PS4.Up()) {
      PS4_UP++;
    }
    else
      PS4_UP = 0;

    if (PS4.UpRight()) {
      PS4_UPRIGHT++;
    }
    else
      PS4_UPRIGHT = 0;
      
    if (PS4.DownRight()) {
      PS4_DOWNRIGHT++;
    }
    else
      PS4_DOWNRIGHT = 0;
      
    if (PS4.UpLeft()) {
      PS4_UPLEFT++;
    }
    else
      PS4_UPLEFT = 0;
      
    if (PS4.DownLeft()){
      PS4_DOWNLEFT++;
    }
    else
      PS4_DOWNLEFT = 0;
     

    if (PS4.Cross()) {
      PS4_CROSS++;
    }
    else
      PS4_CROSS = 0;
    
    if (PS4.Triangle()) {
      PS4_TRIANGLE++;
    }
    else
      PS4_TRIANGLE = 0;


      
    if (PS4.Square()) {
      PS4_SQUARE++;
    }
    else
      PS4_SQUARE = 0;
    
    if (PS4.Circle()) {
      PS4_CIRCLE++;
    }
    else
      PS4_CIRCLE = 0;
      

    if (PS4.L1()) {
       Serial.println("L1 Button");
       PS4_L1++;
       if(PS4_L1 == 1)
       {
          cmdStr = "startTrot";     
       }

    }
    else
      PS4_L1 = 0;
      
    if (PS4.R1()) {
      Serial.println("R1 Button");
      PS4_R1++;
      if(PS4_R1 == 1)
      {
        cmdStr = "startWalk";
      }
    }
    else
      PS4_R1 = 0;

    if (PS4.Share())PS4_SHARE++;//Serial.println("Share Button");
    else PS4_SHARE = 0;

    if (PS4.Options()) PS4_OPTION++;//Serial.println("Options Button");
    else PS4_OPTION = 0;
    
    if (PS4.L3()) Serial.println("L3 Button");
    if (PS4.R3()) Serial.println("R3 Button");

    if(PS4.Share()&&PS4.Options())
    {
      if(PS4_SHARE>100 && PS4_OPTION>100)
      {
        if(OTAStarted == 0)
        {
          OTAStarted = 1;
          Serial.println("share and option Button press, restart and OTA");
          wifiConfigMode = 2;//重启后为OTA 模式 
          saveConfig(1);
          MakeBeep(2);
          ESP.restart();
        }
      }
    }
    
    if (PS4.PSButton()) {
      PS4_PS++;
      Serial.print("PS Button ");
      Serial.println(PS4_PS);
      if(PS4_PS>=100)//长按PS button 切换为wifi mode
      {
        Serial.printf("Battery Level : %d\n", PS4.Battery());
        Serial.println("ready for WIFI mode ");
        wifiConfigMode = 1;//重启后为wifi mode 
        saveConfig(1);
        MakeBeep(1);
        ESP.restart();
      }
    }
    else
      PS4_PS = 0;
      
    if (PS4.Touchpad()) 
    {
      PS4_TOUCH++;
      Serial.println("Touch Pad Button");
      if(PS4_TOUCH == 1){
        if(!isMpuWorking() && !bMpuInited)
        {
          bMpuInited = true;
          cmdStr="initMpu";
          BeepWarning = 2;
        }  
        else if(isMpuWorking())
        {
          stopMPU();
          bMpuInited = false;
          BeepWarning = 3;           
        }
      }
    }
    else
      PS4_TOUCH = 0;

    if (PS4.L2()) {
      if(!PS4_L2 && PS4.L2Value()<10)//ps4 的bug，每次启动PS4后第一次需要按到底再回位才能使用
        PS4_L2 = 1;
      if(PS4_L2)
      {
        if(!isTrotRunning()){
          long val = map(PS4.L2Value(), 0, 250, 180, 70);        
          directX = 0;
          directY = 0;
          directZ = val;
          cmdStr = "directTo&x="+String(directX)+"&y="+String(directY)+"&z="+String(directZ);
          bSettingStandHigh = true;
        }
        PS4_L2 = PS4.L2Value();
      }
    }
    else
      PS4_L2 = 0;

    
    if (PS4.R2() )
    {
      if(!PS4_R2 && PS4.R2Value()<10)//ps4 的bug，每次启动PS4后第一次需要按到底再回位才能使用，可能是为了防误操作
        PS4_R2 = 1;
      if(PS4_R2)
      {
        PS4_R2 = PS4.R2Value();
      }
    }
    else
      PS4_R2 = 0;


    if (abs(PS4.LStickX())>5) {
//      Serial.printf("Left Stick x at %d\n", PS4.LStickX());
      PS4_LSTICK_X = PS4.LStickX();
    }
    else
      PS4_LSTICK_X = 0;
    if (abs(PS4.LStickY())>5) {
//      Serial.printf("Left Stick y at %d\n", PS4.LStickY());
      PS4_LSTICK_Y = PS4.LStickY();
    }
    else
      PS4_LSTICK_Y = 0;
      

    if (abs(PS4.RStickX())>5) {
//      Serial.printf("Right Stick x at %d\n", PS4.RStickX());
      PS4_RSTICK_X = PS4.RStickX();
    }
    else
      PS4_RSTICK_X = 0;
      
    if (abs(PS4.RStickY())>5) {
//      Serial.printf("Right Stick y at %d\n", PS4.RStickY());
      PS4_RSTICK_Y = PS4.RStickY();
    }
    else
      PS4_RSTICK_Y = 0;


    if (PS4.Charging()) Serial.println("The controller is charging");
    if (PS4.Audio()) Serial.println("The controller has headphones attached");
    if (PS4.Mic()) Serial.println("The controller has a mic attached");
    
    RStickXarray[indexOfarray] = PS4_RSTICK_X;
    RStickYarray[indexOfarray] = PS4_RSTICK_Y;
    LStickXarray[indexOfarray] = PS4_LSTICK_X;
    LStickYarray[indexOfarray] = PS4_LSTICK_Y;
    R2array[indexOfarray] = PS4_R2;
    L2array[indexOfarray] = PS4_L2;    

    handlePS4DirectionButton();
    
    if(isTrotRunning())
    {
      doTrot();
    }
    else if(isWalkRunning())
    {
      doWalk();
    }
    else//静态的时候遥杆来控制姿态
    {
      doPose();
    }

  indexOfarray++;
  if(indexOfarray>=num_avg)
    indexOfarray = 0;
  
}

int getAvgStickLX()
{
  int sum = 0;
  for(int i=0;i<num_avg;i++)
  {
    sum += LStickXarray[i];
  }

  return sum/num_avg;
}

int getAvgStickLY()
{
  int sum = 0;
  for(int i=0;i<num_avg;i++)
  {
    sum += LStickYarray[i];
  }

  return sum/num_avg;
}

int getAvgStickRX()
{
  int sum = 0;
  for(int i=0;i<num_avg;i++)
  {
    sum += RStickXarray[i];
  }

  return sum/num_avg;
}

int getAvgStickRY()
{
  int sum = 0;
  for(int i=0;i<num_avg;i++)
  {
    sum += RStickYarray[i];
  }

  return sum/num_avg;
}

int getAvgL2()
{
  int sum = 0;
  for(int i=0;i<num_avg;i++)
  {
    sum += L2array[i];
  }
  return sum/num_avg;  
}

int getAvgR2()
{
  int sum = 0;
  for(int i=0;i<num_avg;i++)
  {
    sum += R2array[i];
  }
  return sum/num_avg;  
}

bool isRotating()
{
  return bRotating;
}

void doTrot()
{
  int x = getAvgStickLX();
  int y = getAvgStickLY();
  int xr = getAvgStickRX();
  int yr = getAvgStickRY(); 
  int l2 = getAvgL2();
  int r2 = getAvgR2();
  float rate = 0;
  float s = 0;
  float standDelta = 0;
  rate = float(sqrt(pow(x,2)+pow(y,2)))/128.0;
//  s = map(abs(xr),0,128,1000,100);
//  s =  s/1000;
  if(rate>1)rate = 1;
//  if(PS4_SQUARE == 1)//|| getStepStandHigh()>STAND_HIGH)//抬高,最多提高20
//    standDelta = (float)map(r2,0,255,0,-20*10)/10;
//  else
  standDelta = (float)map(r2,0,255,0,80*10)/10;//降低身高,最多降80
  setStepStandDelta(standDelta);

//  handlePS4DirectionButton();

  if(yr != 0){//俯仰行走
    s = (float)map(yr,-128,128,20*100,-20*100)/100.0;
    setBodyPitch(s);
  } 
  if(xr != 0)//侧倾行走
  {
    s = (float)map(xr,-128,128,-20*100,20*100)/100.0;       
    setBodyRoll(s);     
  }

  if(y == 0 && x != 0)//左右横行
  {
    if(PS4_CIRCLE>0)//调头
    {
      Serial.printf("PS4_CIRCLE>0");
      setTrotDirection(0);
      setTrotTheta(0);
      s = (float)map(abs(x),0,128,0,300)/10;
      setDeflectionAngle(s);      
      if(x<0)//左调头
      {
        setStartDeflection(-1);    
      }  
      else//右调头
      {
        Serial.printf("setStartDeflection(1)");
        setStartDeflection(1); 
      }
    }
    else{//横行
      setStartDeflection(0);
      if(x<0)//左横行
        setTrotTheta(M_PI/2);  
      else//右横行
        setTrotTheta(-M_PI/2);  
      rate = rate/2;//横行只能使用一半的步长，否则会侧翻
      s = (float)map(abs(x),0,128,100,0)/100;//1-0      
      if(PS4_TRIANGLE>0){//绕圈横行,头向中心点
        setTrotDirectionFR(s*rate,rate);         
      }
      else if(PS4_CROSS>0)//绕圈横行,尾向中心点
        setTrotDirectionFR(rate,s*rate);    
      else{//正常横行
        setTrotDirection(rate); 
      }      
    }
  }
  else if(y != 0)//前进后退，y != 0 ,x 随意
  {
    setStartDeflection(0);
    setTrotTheta(0); 
    if(y<0)//后退
      rate = -rate;

    s = (float)map(abs(x),0,128,100,50)/100;//1-0.5
    if(x<0)//左偏
      setTrotDirectionLR(s*rate,rate);
    else if(x>0)//右偏
      setTrotDirectionLR(rate,s*rate);
    else//正常前进后退
      setTrotDirection(rate);
  }

}

void doPose()
{
  //摇杆是右上为正
  //左摇杆，左右控制 yaw，上下控制pitch
  //右摇杆，左右控制 roll，上下控制伸头和缩头  
  int xl = getAvgStickLX();
  int yl = getAvgStickLY();
  int xr = getAvgStickRX();
  int yr = getAvgStickRY();  
  int r2 = getAvgR2(); 
  if(xl != 0 || yl != 0 || xr != 0 || yr != 0 || r2 !=0)
  {
    float yaw,roll,pitch,moveX,moveY,moveZ;
    bRotating = true;
    yaw = map(xl,-125,125,-max_yaw,max_yaw);
    pitch = map(yr,-125,125,max_roll,-max_roll);
    roll = map(xr,-125,125,-max_pitch,max_pitch);
    moveX = map(yl,-125,125,-50,50);//伸头距离
    moveY = 0;
    if(PS4_TRIANGLE>0)
    {
      yaw = 0;
      moveY = map(xl,-125,125,50,-50);
    }
    else if(PS4_CROSS>0)
    {
      moveX = 0;
      moveY = map(yl,-125,125,50,-50);
    }

    moveZ = map(r2,0,250,getStepStandHigh(),const_sit_high);
    moveZ = moveZ - getStepStandHigh();//下蹲范围，是一个差值，为负数 从-(getStepStandHigh()-const_sit_high)到0
    cmdStr = "rotate?y="+String(yaw)+"&r="+String(roll)+"&p="+String(pitch)+"&x="+String(moveX)+"&y="+String(moveY)+"&z="+String(moveZ);
//      Serial.println(cmdStr);
  }
  else if(bRotating)
  {
    bRotating = false;
    cmdStr = "rotate?y=0&r=0&p=0&x=0&y=0&z=0";
  }  
  else
  {
    if(PS4_CIRCLE>0)
      bNeedBalance = true;
    else
      bNeedBalance = false;    
  }
  
}

void doWalk()
{
  int x = getAvgStickLX();
  int y = getAvgStickLY();
  int xr = getAvgStickRX();
  int yr = getAvgStickRY(); 
  int dz = getAvgR2();

  float d = 0;
  float t = 1;
  d = (float)map(abs(y),0,255,500,1000)/1000;//取得方向 0-1
  t = (float)map(abs(xr),0,255,1000,500)/1000;//取得转向 1-0.5
  if(y == 0)
  {
    setWalkDirection(1);
  }
  else if(y>0)//前进
  {
    setWalkDirection(d);
  }
  else//后退
  {
    setWalkDirection(-d);
  }

  if(xr == 0)
    setWalkStepLengthRate(1,1);
  else if(xr <0)//左转
  {
    setWalkStepLengthRate(t,1);
  }
  else//右转
  {
    setWalkStepLengthRate(1,t);
  }
}

void handlePS4DirectionButton()//处理上下左右按键的作用
{
  if(isTrotRunning())
  {
    if(PS4_SQUARE >0)//调节周期和抬腿高度
    {
      if(PS4_UP==1)
      {
          new_lift_high+=5;
          if(new_lift_high>80)
          {
            new_lift_high = 80;
            BeepWarning = 2;
          } 
          else
           BeepWarning = 1;    
          #if WITHOUT_NEW 
          setLiftHigh(new_lift_high); 
          #endif
      }
      else if(PS4_DOWN==1)
      {
          new_lift_high-=5;
          if(new_lift_high<20)
          {
            new_lift_high = 20;
            BeepWarning = 2;
          }   
          else
           BeepWarning = 1;    
          #if WITHOUT_NEW 
          setLiftHigh(new_lift_high);   
          #endif       
      }
      else if(PS4_LEFT==1)
      {
        
          new_cycle_time-=50;
          if(new_cycle_time<200)
          {
            new_cycle_time = 200;
            BeepWarning = 2;
          }  
          else
           BeepWarning = 1; 
          #if WITHOUT_NEW 
          setCycleTime(new_cycle_time);
          #endif
      }
      else if(PS4_RIGHT==1) 
      {
          new_cycle_time+=50;
          if(new_cycle_time>3500){
            new_cycle_time = 3500;
            BeepWarning = 2;
          }   
          else
           BeepWarning = 1; 
          #if WITHOUT_NEW 
          setCycleTime(new_cycle_time);
          #endif
      }       
    }
    else
    {
      if(PS4_UP==1)
      {
        setTrotTheta(0); 
        addTrotDirection(true);       
      }
      else if(PS4_DOWN==1)
      {
        setTrotTheta(0); 
        addTrotDirection(false);            
      }
      else if(PS4_LEFT==1)
      {
        turnDirection(true);
      }
      else if(PS4_RIGHT==1) 
      {
        turnDirection(false);
      }
    }
  }
  else if(isWalkRunning())
  {

  }
  else//静态的时候遥杆来控制姿态
  {  
    if(PS4_SQUARE >0)//跳跃
    {
      if(PS4_UP==1)
        cmdStr = "jump?d=1";//前进
      else if(PS4_DOWN==1)
        //cmdStr = "jump?d=0";//原地跳
        cmdStr = "backflip";//后空翻
      else if(PS4_LEFT == 1)
        cmdStr = "jump?d=3";//左转跳
      else if(PS4_RIGHT == 1)
        cmdStr = "jump?d=4";//右转跳
    }
    else
    {
      if(PS4_UP==1)
        cmdStr = "stand";
      else if(PS4_DOWN==1)
        cmdStr = "sit";
      else if(PS4_LEFT==1)
        cmdStr = "bainian";  
      else if(PS4_RIGHT==1)
        cmdStr = "wait";          
    }
  }
}
