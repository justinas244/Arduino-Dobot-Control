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
      joint1-= 0.25;
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
    delay(2000);
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
 
void laukimas(int laikas){
    while(true){
      delay(laikas);
      break;
    }
}
void belt_on(){
  Serial2.println("M311 -100");
}

void belt_off(){
  Serial2.println("M311 0");
}

void rankos_online(){
   byte error, address;
   int nDevices;
 
   Serial.println("Scanning...");
 
  nDevices = 0;
  for(address = 1; address < 127; address++ )
  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
 
    if (error == 0)
    {
      Serial.print("I2C device found at address 0x");
      if (address<16)
            Serial.print("0");
          Serial.print(address,HEX);
          Serial.println("  !");
     
          nDevices++;
          if(address == pirmos_rankos_adresas){
            Serial.println("Pirma ranka online");
            pirma_ranka_dirba = true;
          }
          if(address == wifi_adresas){
            Serial.println("wifi online");
            wifi_dirba = true;
          }
    }
    else if (error==4)
    {
      Serial.print("Unknown error at address 0x");
      if (address<16)
        Serial.print("0");
      Serial.println(address,HEX);
    }    
  }
  if (nDevices == 0)
    Serial.println("No I2C devices found\n");
  else
    Serial.println("done\n");
 
  delay(2000);           // wait 5 seconds for next scan
}





