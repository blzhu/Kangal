#define USE_SPIFFS 0 //0 use EEPROM ,1 use SPIFFS
#define FORMAT_SPIFFS_IF_FAILED true
int currentConfigLeg = 0;
int currentConfigServo = 0;
const int eeprom_size =1000;
const int eeprom_addr0 = 0;//放 leg offset 地址0-200
const int eeprom_addr1 = 200;//放wifi mode,moveOffsetX 地址200-300 
const int eeprom_addr2 = 300;//放其他参数  地址300-1000 ，还未启用
const String fileName = "/config.ini";

void initFileSys()
{
#if USE_SPIFFS
    if(!SPIFFS.begin(FORMAT_SPIFFS_IF_FAILED)){
        Serial.println("SPIFFS Mount Failed");
        return;
    }
#else
    if (!EEPROM.begin(eeprom_size)) {
    Serial.println("Failed to initialise EEPROM");
    Serial.println("Restarting...");
    delay(1000);
    ESP.restart();
  }
#endif
}
void readConfig()
{
//    listDir(SPIFFS, "/", 0);
    String fileRead,strParam1;
#if USE_SPIFFS
    readFile(SPIFFS, fileName.c_str(),fileRead);
#else
    fileRead = EEPROM.readString(eeprom_addr0);
    strParam1 = EEPROM.readString(eeprom_addr1);
#endif
    if(fileRead.length()<=0)
    {
      Serial.println("no config file");
      saveConfig(0);
    }
    Serial.println(fileRead);
    Serial.println(strParam1);
    char splitChar = ';';
    int count = countSplitCharacters(fileRead,splitChar);
    if(count!= 12)
    {
      Serial.println("error config file");
      saveConfig(0);
    }
    String strSplit;
    for(int i = 0;i<count;i++)
    {
      strSplit = getSplitValue(fileRead,splitChar,i);
      Serial.println(strSplit.c_str());
      if(i>=0&&i<3)
        Legs_offset[0][i] = strSplit.toInt();
      else if(i<6)
        Legs_offset[1][i-3] = strSplit.toInt();
      else if(i<9)
        Legs_offset[2][i-6] = strSplit.toInt();    
       else if(i<12)
        Legs_offset[3][i-9] = strSplit.toInt();             
    }
    count = countSplitCharacters(strParam1,splitChar);
    if(count == 0)
      wifiConfigMode = strParam1.toInt();
    else if(count == 2)
    {
      strSplit = getSplitValue(strParam1,splitChar,0);
      wifiConfigMode = strSplit.toInt(); 
      strSplit = getSplitValue(strParam1,splitChar,1);
//      moveOffsetX = strSplit.toInt(); 
    }
      
    Serial.println("readConfig OK");

// verify
//    String fileText="";
//    for(int i=0;i<4;i++)
//    {
//      for(int j = 0;j<3;j++)
//      {
//        fileText+=String(Legs_offset[i][j]) + ";";
//      }
//    }  
//    Serial.println(fileText);
}
void saveAllconfig()
{
  saveConfig(0);
  saveConfig(1);
}
void saveConfig(int nRom)
{
    String fileText="";
    for(int i=0;i<4;i++)
    {
      for(int j = 0;j<3;j++)
      {
        fileText+=String(Legs_offset[i][j]) + ";";
      }
    }  
    Serial.println(fileText);
#if USE_SPIFFS
    writeFile(SPIFFS, fileName.c_str(), fileText.c_str());
#else
    if(nRom == 0){
//      fileText = "3;-6;-11;5;-1;-11;0;7;-15;5;9;-5;"; //可以设定默认值
    //3;-6;-11;5;-1;-11;0;7;-15;5;9;-5;PCB_v9 default
      EEPROM.writeString(eeprom_addr0, fileText);
    }
    else if(nRom == 1){
      fileText = "";
      fileText+=String(wifiConfigMode)+";";
      fileText+=String(moveOffsetX)+";";
      EEPROM.writeString(eeprom_addr1, fileText);
    }
    EEPROM.commit();
#endif
    Serial.println("saveConfig OK");
}
//void handleRoot(AsyncWebServerRequest *request) {
//  hadHandleRoot = 1;
//  request->send(200, "text/html", textHTML.c_str());
//}
//void notFound(AsyncWebServerRequest *request) {
//    request->send(404, "text/plain", "Not found");
//}

void uninitWifi()
{
  WiFi.softAPdisconnect(true);
//  server.end();
}
void initWifi()
{
  WiFi.softAP("Kangal");
  IPAddress myIP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(myIP);
//  initWebServer();
}
//void initWebServer()
//{
//  Serial.println("initWebServer");
//    getHTML();
//    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
// 
//    int paramsNr = request->params();
//    Serial.println(paramsNr);
// 
//    for(int i=0;i<paramsNr;i++){
// 
//        AsyncWebParameter* p = request->getParam(i);
//        Serial.print("Param name: ");
//        Serial.println(p->name());
//        Serial.print("Param value: ");
//        Serial.println(p->value());
//        Serial.println("------");
//        configLegs(p->name(),p->value());
//    }
//    handleRoot(request);
//  });
//
//
//  server.onNotFound(notFound);
//
//  server.begin();
//}

//void configLegs(String cmd,String value)
//{
//  bool bAdd = true;
//  if(cmd.equals("leg"))
//  {
//    if(value.equals("l1"))
//      currentConfigLeg = 0;
//    else if(value.equals("l2"))
//      currentConfigLeg = 1;    
//    else if(value.equals("l3"))
//      currentConfigLeg = 2;  
//    else if(value.equals("l4"))
//      currentConfigLeg = 3;      
//              
////    initLeg(currentConfigLeg);
//  }
//  else if(cmd.equals("ham"))
//  {
//    if(value.equals("inc"))
//      bAdd = true;
//    else 
//      bAdd = false;
//    adjustLeg(currentConfigLeg,0,bAdd);
//  }
//  else if(cmd.equals("arm"))
//  {
//    if(value.equals("inc"))
//      bAdd = true;
//    else 
//      bAdd = false;
//    adjustLeg(currentConfigLeg,1,bAdd);
//  }
//  else if(cmd.equals("shoulder"))
//  {
//    if(value.equals("inc"))
//      bAdd = true;
//    else 
//      bAdd = false;
//    adjustLeg(currentConfigLeg,2,bAdd);
//  }  
//  else if(cmd.equals("move"))
//  {
//    if(value.equals("inc")){
//      bAdd = true;
//      adjustMove(bAdd);
//    }
//    else if(value.equals("dec")){
//      bAdd = false;
//      adjustMove(bAdd);
//    }
//    else if(value.equals("sit")){
//      cmdStr = "sit";
//    }
//    else if(value.equals("stand")){
//      cmdStr = "stand";
//    }    
//  }    
//  else if(cmd.equals("key"))
//  {
//    if(value.equals("sc"))
//    {
//      saveAllconfig();
//    }
//    else if(value.equals("init"))
//    {
//        cmdStr = "init";
//    }
//    else if(value.equals("step"))
//    {
//       cmdStr = "startTrot";
////       setTrotDirection(0);
//    }
//  }
//}
