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

void Vakumas(bool states){
   SetEndEffectorGripper(states, states, &gQueuedCmdIndex); // open gripper (compressed air on);
   ProtocolProcess(); //have command(s) executed by dobot
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
    Vakumas(true);
    delay(3000);
    movejoint(paeimimo_point[0], paeimimo_point[1], paeimimo_point[2], paeimimo_point[3]);
    ProtocolProcess();
    delay(3000);
    Vakumas(false);
    delay(3000);
    moveArm(Detuves_kordinates[i][0], Detuves_kordinates[i][1], Detuves_kordinates[i][2]+50, Detuves_kordinates[i][3]);
    ProtocolProcess();
    delay(3000);
  }
  delay(3000);
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
void belt_on(){
  Serial2.println("M311 -80");
}

void belt_off(){
  Serial2.println("M311 0");
}


void rusiavimas(int spalvos_kodas){
  if(spalvos_kodas !=0){
    belt_on();
    while(true){
      rusiavimo_optika = digitalRead(rusiavimo_optikos_pin);
      if(rusiavimo_optika == LOW){ break; }
    }
   belt_off();
   delay(500); 
   movejoint(rusiavimo_darbinis[0], rusiavimo_darbinis[1], rusiavimo_darbinis[2], rusiavimo_darbinis[3]);
   ProtocolProcess();
   delay(2000);
   for(int i =0; i<=3; i++){
     if(rusiavimo_tvarka[i] == spalvos_kodas){
        moveArm(rusiavimo_kordinates[i][0], rusiavimo_kordinates[i][1], (rusiavimo_kordinates[i][2]+(kubelio_skirtumas*kubeliu_skaicius_rusiavime[i])+50), rusiavimo_kordinates[i][3]);
        ProtocolProcess();
        delay(4000);
        moveArm(rusiavimo_kordinates[i][0], rusiavimo_kordinates[i][1], rusiavimo_kordinates[i][2]+(kubelio_skirtumas*kubeliu_skaicius_rusiavime[i]), rusiavimo_kordinates[i][3]);
        ProtocolProcess();
        delay(4000);
        Vakumas(false);
        delay(3000);
        moveArm(rusiavimo_kordinates[i][0], rusiavimo_kordinates[i][1], (rusiavimo_kordinates[i][2]+(kubelio_skirtumas*kubeliu_skaicius_rusiavime[i])+50), rusiavimo_kordinates[i][3]);
        ProtocolProcess();
        delay(2000);
        kubeliu_skaicius_rusiavime[i]++;
     }
   }
   
   movejoint(rusiavimo_darbinis[0], rusiavimo_darbinis[1], rusiavimo_darbinis[2], rusiavimo_darbinis[3]);
   ProtocolProcess();
   delay(1000);
  }
}
void nuskaitymas_spalvos(){
  // Setting RED (R) filtered photodiodes to be read
  digitalWrite(S2,LOW);
  digitalWrite(S3,LOW);
  digitalWrite(led, HIGH);
  // Reading the output frequency
  redFrequency = pulseIn(sensorOut, LOW);
  
   // Printing the RED (R) value
  Serial.print("R = ");
  Serial.print(redFrequency);
  delay(100);
  
  // Setting GREEN (G) filtered photodiodes to be read
  digitalWrite(S2,HIGH);
  digitalWrite(S3,HIGH);
  
  // Reading the output frequency
  greenFrequency = pulseIn(sensorOut, LOW);
  
  // Printing the GREEN (G) value  
  Serial.print(" G = ");
  Serial.print(greenFrequency);
  delay(100);
 
  // Setting BLUE (B) filtered photodiodes to be read
  digitalWrite(S2,LOW);
  digitalWrite(S3,HIGH);
  
  // Reading the output frequency
  blueFrequency = pulseIn(sensorOut, LOW);
  
  // Printing the BLUE (B) value 
  Serial.print(" B = ");
  Serial.println(blueFrequency);
  delay(100);
  
  if(buvo_aptikta == false){ aptikta_spalva =spalva(redFrequency,greenFrequency,blueFrequency);}
  
  if(aptikta_spalva >0){ buvo_aptikta = true; }
  
  if(aptikta_spalva == 1){
    Serial.println("Aptikta RAUDONA");
  }
  else if(aptikta_spalva == 2){
    Serial.println("Aptikta GELTONA");
  }
  else if(aptikta_spalva == 3){
    Serial.println("Aptikta ZALIA");
  }
  else if(aptikta_spalva == 4){
    Serial.println("Aptikta MELYNA");
  }
  else if(aptikta_spalva == 0){
    Serial.println("Nera SPALVOS TOKIOS");
  }
  delay(2000);
  digitalWrite(led, LOW);
  delay(2000);
}

int spalva(int r, int g, int b){
   if((r >= raudona[0] && r <= raudona[1])&&(g >= raudona[2] && g <= raudona[3])&&(b >= raudona[4] && b <= raudona[5])){
    return 1;
   }
   else if((r >= geltona[0] && r <= geltona[1])&&(g >= geltona[2] && g <= geltona[3])&&(b >= geltona[4] && b <= geltona[5])){
    return 2;
   }
   else if((r >= zalia[0] && r <= zalia[1])&&(g >= zalia[2] && g <= zalia[3])&&(b >= zalia[4] && b <= zalia[5])){
    return 3;
   }
   else if((r >= melyna[0] && r <= melyna[1])&&(g >= melyna[2] && g <= melyna[3])&&(b >= melyna[4] && b <= melyna[5])){
    return 4;
   }
   else{
    return 0;
   }
}


