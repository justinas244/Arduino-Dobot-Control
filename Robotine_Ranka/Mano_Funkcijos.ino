void moveArm(double x, double y, double z, double r)
{
  gPTPCmd.ptpMode = MOVJ_XYZ;
  gPTPCmd.x = x;
  gPTPCmd.y = y;
  gPTPCmd.z = z;
  gPTPCmd.r = r;

  Serial.print("move to x:"); Serial.print(gPTPCmd.x); Serial.print(" y:"); Serial.print(gPTPCmd.y); Serial.print(" z:"); Serial.println(gPTPCmd.z);

  SetPTPCmd(&gPTPCmd, true, &gQueuedCmdIndex);
  ProtocolProcess();
  
  currentX = x;
  currentY = y;
  currentZ = z;
  currentR = r;
}

void movejoint(double joint1, double joint2, double joint3, double joint4)
{
  gPTPCmd.ptpMode = MOVJ_ANGLE;
  
  gPTPCmd.x = joint1;
  gPTPCmd.y = joint2;
  gPTPCmd.z = joint3;
  gPTPCmd.r = joint4;

  Serial.print("move to joint1:"); Serial.print(gPTPCmd.x); Serial.print(" joint2:"); Serial.print(gPTPCmd.y); Serial.print(" joint3:"); Serial.println(gPTPCmd.z);

  SetPTPCmd(&gPTPCmd, true, &gQueuedCmdIndex);
  ProtocolProcess();

  currentjoint1 = joint1;
  currentjoint2 = joint2;
  currentjoint3 = joint3;
  currentjoint4 = joint4;
  
}

void Vakumas(bool vacuumOn){
    if (endeffectorGripper == true) {
        if (vacuumOn == false && vacuumOn != currentVac) {
            Serial.println("Open GRIPPER");
            //delay(1000);
            SetEndEffectorSuctionCup(false, true, &gQueuedCmdIndex);
            ProtocolProcess(); //have command(s) executed by dobot
            delay(500);
            SetEndEffectorGripper(true, true, &gQueuedCmdIndex); // open gripper (compressed air on);
            ProtocolProcess(); //have command(s) executed by dobot
            delay(500);
            SetEndEffectorGripper(false, true, &gQueuedCmdIndex);  // stop activating gripper when it is openend (compressed air off)
            delay(500); 
            }
      }
      else{
      if (vacuumOn == false) SetEndEffectorSuctionCup(false, false, &gQueuedCmdIndex);
      }
      
      if (vacuumOn == true && vacuumOn != currentVac)SetEndEffectorSuctionCup(true, false, &gQueuedCmdIndex);
      ProtocolProcess();

      currentVac = vacuumOn;
}

void Griperis_atleisti(){
    if(suspausta == true){
      SetEndEffectorGripper(true, false, &gQueuedCmdIndex);  // iskleidziam bet oras dar ijungtas
      ProtocolProcess();
      delay(2000);
      suspausta == false;
    }
    SetEndEffectorGripper(false, true, &gQueuedCmdIndex);
    ProtocolProcess();
    suspausta == false;
    delay(2000);
}

void Siurbtukas(bool state){
  SetEndEffectorSuctionCup(state, false, &gQueuedCmdIndex);
  ProtocolProcess();
  delay(2000);
}

void HomeX(){
  
   while(true){
      startPosX++;
      moveArm(startPosX, startPosY, startPosZ, startPosR);
      xhome_mygtukas = digitalRead(xhome_mygtuko_pin);
      if(xhome_mygtukas == LOW){ break;}
      delay(200);
    }
    
}
void HomeY(){
  float joint1 = 0;
  while(true){
      //startPosY--;
      joint1+= 0.25;
      movejoint(joint1, 0 , 0, 0);
      yhome_mygtukas = digitalRead(yhome_mygtuko_pin);
      if(yhome_mygtukas == HIGH){ 
        break;
      }
      delay(200);
    }
}
void Home(){
  HomeY();

   djoint1_home = currentjoint1;
  Serial.println("-------------------------------------");
  Serial.println("Namu kordinates");
  Serial.print("X: ");
  Serial.println(currentX);
  Serial.print("Y: ");
  Serial.println(currentY);
  Serial.print("Z: ");
  Serial.println(currentZ);

  Serial.println("Namu JOINT kordinates");
  Serial.print("Joint1: ");
  Serial.println(currentjoint1);
  Serial.print("Joint2: ");
  Serial.println(currentjoint2);
  Serial.print("Joint3: ");
  Serial.println(currentjoint3);
  Serial.println("-------------------------------------");
  
  /*Serial.println("----------------------");
  Serial.print("Joint1 offsetas: ");
  Serial.println(joint1_ofset);
  Serial.println("----------------------");*/
 
  while(true){
   SetHomeCmd(true, &gQueuedCmdIndex);
    ProtocolProcess();
    delay(7000);
    break;
  }
  delay(3000);
  for(int i=0; i<3; i++){
    movejoint(darbinis_point[0], darbinis_point[1], darbinis_point[2], darbinis_point[3]);
    ProtocolProcess();
    delay(3000);
    movejoint(paeimimo_point[0], paeimimo_point[1], paeimimo_point[2], paeimimo_point[3]);
    ProtocolProcess();
    delay(3000);
    moveArm(Detuves_kordinates[i][0], Detuves_kordinates[i][1], Detuves_kordinates[i][2]+50, Detuves_kordinates[i][3]);
    ProtocolProcess();
    delay(3000);
  }
  delay(5000);
  
}
void sendWaitCommand(uint32_t duration) {
  // Start of message
  Serial1.write(0x84);
  Serial1.write(0x04);
  
  // Little-endian format
  Serial1.write(duration & 0xFF);
  Serial1.write((duration >> 8) & 0xFF);
  Serial1.write((duration >> 16) & 0xFF);
  Serial1.write((duration >> 24) & 0xFF);
  
  // Checksum
  uint8_t checksum = 0x84 + 0x04 + (duration & 0xFF) + ((duration >> 8) & 0xFF) + ((duration >> 16) & 0xFF) + ((duration >> 24) & 0xFF);
  Serial1.write(checksum);
}
 void detaliu_pozicija(int pirmas, int antras, int trecias){
  int skaicius[3] ={pirmas, antras, trecias};
   for(int i=0; i<=2; i++){
    if(skaicius[i] == 0){
        Detuve[i*3] = 0;
        Detuve[(i*3)+1] = 0;
        Detuve[(i*3)+2] = 0;
     }
     else if(skaicius[i] == 123){
        Detuve[i*3] = 1;
        Detuve[(i*3)+1] = 1;
        Detuve[(i*3)+2] = 1;
     }
     else if(skaicius[i] == 1){
        Detuve[i*3] = 1;
        Detuve[(i*3)+1] = 0;
        Detuve[(i*3)+2] = 0;
     }
     else if(skaicius[i] == 2){
        Detuve[i*3] = 0;
        Detuve[(i*3)+1] = 1;
        Detuve[(i*3)+2] = 0;
     }
     else if(skaicius[i] == 3){
        Detuve[i*3] = 0;
        Detuve[(i*3)+1] = 0;
        Detuve[(i*3)+2] = 1;
     }
     else if(skaicius[i] == 12){
        Detuve[i*3] = 1;
        Detuve[(i*3)+1] = 1;
        Detuve[(i*3)+2] = 0;
     }
     else if(skaicius[i] == 13){
        Detuve[i*3] = 1;
        Detuve[(i*3)+1] = 0;
        Detuve[(i*3)+2] = 1;
     }
     else if(skaicius[i] == 23){
        Detuve[i*3] = 0;
        Detuve[(i*3)+1] = 1;
        Detuve[(i*3)+2] = 1;
     }
   }
 }

void moveranka(float x, float y, float z, float r, bool vacuumOn)
{
  
        gPTPCmd.ptpMode = MOVJ_XYZ;
        gPTPCmd.x = x;
        gPTPCmd.y = y;
        gPTPCmd.z = z;
        gPTPCmd.r = r;
      
      Serial.print("move to x:"); Serial.print(gPTPCmd.x); Serial.print(" y:"); Serial.print(gPTPCmd.y); Serial.print(" z:"); Serial.println(gPTPCmd.r);
      
      SetPTPCmd(&gPTPCmd, true, &gQueuedCmdIndex);
      
      if (endeffectorGripper == true) {
        if (vacuumOn == false && vacuumOn != currentVac) {
            Serial.println("Open GRIPPER");
            //delay(1000);
            SetEndEffectorSuctionCup(false, true, &gQueuedCmdIndex);
            ProtocolProcess(); //have command(s) executed by dobot
            delay(500);
            SetEndEffectorGripper(true, true, &gQueuedCmdIndex); // open gripper (compressed air on);
            ProtocolProcess(); //have command(s) executed by dobot
            delay(500);
            SetEndEffectorGripper(false, true, &gQueuedCmdIndex);  // stop activating gripper when it is openend (compressed air off)
            delay(500); 
            }
      }
      else{
      if (vacuumOn == false) SetEndEffectorSuctionCup(false, true, &gQueuedCmdIndex);
      }
      
      if (vacuumOn == true && vacuumOn != currentVac)SetEndEffectorSuctionCup(true, true, &gQueuedCmdIndex);
      ProtocolProcess();
      
      currentX = x;
      currentY = y;
      currentZ = z;
      currentR = r;
      currentVac = vacuumOn;
}
void laukimas(int laikas){
    while(true){
      delay(laikas);
      break;
    }
}




