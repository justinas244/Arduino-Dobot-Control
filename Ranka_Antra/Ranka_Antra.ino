/****************************************Copyright(c)*****************************************************
**                            Shenzhen Yuejiang Technology Co., LTD.
**
**                                 http://www.dobot.cc
**
**--------------File Info---------------------------------------------------------------------------------
** File name:           main.cpp
** Latest modified Date:2016-10-24
** Latest Version:      V2.0.0
** Descriptions:        main body
**
**--------------------------------------------------------------------------------------------------------
** Modify by:           Edward
** Modified date:       2016-11-25
** Version:             V1.0.0
** Descriptions:        Modified,From DobotDemoForSTM32
**--------------------------------------------------------------------------------------------------------
*********************************************************************************************************/
#include "stdio.h"
#include "Protocol.h"
#include "command.h"
#include "FlexiTimer2.h"
#include <Wire.h> 

//Set Serial TX&RX Buffer Size
#define SERIAL_TX_BUFFER_SIZE 64
#define SERIAL_RX_BUFFER_SIZE 256

//#define JOG_STICK
/*********************************************************************************************************
** Global parameters
*********************************************************************************************************/
EndEffectorParams gEndEffectorParams;

JOGJointParams  gJOGJointParams;
JOGCoordinateParams gJOGCoordinateParams;
JOGCommonParams gJOGCommonParams;
JOGCmd          gJOGCmd;

PTPCoordinateParams gPTPCoordinateParams;
PTPCommonParams gPTPCommonParams;
PTPCmd          gPTPCmd;


uint64_t gQueuedCmdIndex;
bool suspausta = false;


float startPosX = 160;       
float startPosY = 0;      
float startPosZ = 30;       
float startPosR = 0;       //Start position when powering up


////////////////// DABARTINES KORDINATES ///////////////////////////////
float currentX = 0;     
float currentY = 0;   
float currentZ = 0;   
float currentR = 0;
bool currentVac = false;
bool endeffectorGripper = true; // indicate type of end effector: true for gripper, false for vacuum cup

float currentjoint1 = 0;     
float currentjoint2 = 0;   
float currentjoint3 = 0;   
float currentjoint4 = 0;
/////////////////////////////////////////////////////////////////////////


//////// Prie kokiu pirminiu kordinaciu suprogramuota sistema  ////////////////////////////
double dx_home = 212.2 ;     
double dy_home = 0;   
double dz_home = 199.8;   
double dr_home = 0;

double djoint1_home = 0.0;     
double djoint2_home = 0;   
double djoint3_home = 0;   
double djoint4_home = 0;
////////////////////////////////////////////////////////////////////////

double joint1_ofset = 0;

const byte kCmdSetHOMEParams = 13;
const byte kCmdHOMECmd = 15;
const int kDataLength = 21;

int xhome_mygtuko_pin = 22;
int yhome_mygtuko_pin = 24;

int xhome_mygtukas;
int yhome_mygtukas;
int led = 13;

double paeimimo_point[4] = {-41.3705, 1.1584 , -2.8497 ,0};  // JOINT

double darbinis_point[4] = {-33.3961, 42.1274 , -11.6285, 0};  // JOINT 

double darbinis_pointas_antras[4] = {65.9845, 13.2287 , 0.6487, 0};  // JOINT 

double paeimimo_point_antrinis[4] = {128.5447,  194.3605 , 5.0 ,0};  // kordinates
double paeimimo_point_pirminis[4] = {159.3841, -211.9012 , -1.5994  ,0};  // kordinates

int Detuve[] = {0, 0, 0, 0, 0, 0, 0, 0, 0};

int sensor_pin = 7;
bool start_komanda = true;

// kordinates

double point1[4] = {215.1404, 124.0499 , -45.00   , 0};
double point2[4] = {175.6577, 125.5617 , -45.00   , 0};
double point3[4] = {137.4507, 128.8269 , -45.00   , 0};
double point4[4] = {216.8926, 164.9008 , -45.00   , 0};
double point5[4] = {177.3893, 168.3094 , -45.00   , 0};
double point6[4] = {137.2848, 169.9191 , -45.00   , 0};
double point7[4] = {216.3909, 204.7939 , -46.00   , 0};
double point8[4] = {179.3857, 207.6078 , -46.00   , 0};
double point9[4] = {138.1540, 209.4336 , -49.00   , 0};


double Detuves_kordinates[9][4] = {        //cartesian cordinate
  {point1[0], point1[1], point1[2], point1[3]},
  {point2[0], point2[1], point2[2], point2[3]},
  {point3[0], point3[1], point3[2], point3[3]},
  {point4[0], point4[1], point4[2], point4[3]},
  {point5[0], point5[1], point5[2], point5[3]},
  {point6[0], point6[1], point6[2], point6[3]},
  {point7[0], point7[1], point7[2], point7[3]},
  {point8[0], point8[1], point8[2], point8[3]},
  {point9[0], point9[1], point9[2], point9[3]},
 };


/*********************************************************************************************************
** Function name:       setup
** Descriptions:        Initializes Serial
** Input parameters:    none
** Output parameters:   none
** Returned value:      none
*********************************************************************************************************/

int pirmos_rankos_adresas = 0x68;
int wifi_adresas = 0x67;

bool pirma_ranka_dirba = false;
bool wifi_dirba = false;


#define ARRAY_SIZE 8     // Number of values in the array
int gavimo[ARRAY_SIZE];  // Array to store received values

int belt_on_pin = 25;

void setup() {
  Serial.begin(115200);
  Serial1.begin(115200);
  Serial2.begin(115200);
  
  printf_begin();
  //Set Timer Interrupt
  FlexiTimer2::set(100, Serialread);
  FlexiTimer2::start();
  Serial.println("Atnaujinta 2023-05-03 17:53");

  pinMode(xhome_mygtuko_pin, INPUT_PULLUP);
  pinMode(yhome_mygtuko_pin, INPUT_PULLUP);
  pinMode(sensor_pin, INPUT_PULLUP);
  pinMode(belt_on_pin,INPUT);
  pinMode(led,OUTPUT);
  
    Serial2.println("M310 1");
    delay(4000);
    Serial2.println("M313 100");
    
  InitRAM();

  ProtocolInit();

  SetJOGJointParams(&gJOGJointParams, true, &gQueuedCmdIndex);

  SetJOGCoordinateParams(&gJOGCoordinateParams, true, &gQueuedCmdIndex);

  SetJOGCommonParams(&gJOGCommonParams, true, &gQueuedCmdIndex);

  printf("\r\n======Main Program loop started======\r\n");

  SetPTPCmd(&gPTPCmd, true, &gQueuedCmdIndex);
   ProtocolProcess();
   
   delay(1000);
   Vakumas(true);
   delay(3000);
   Vakumas(false);
   delay(1000);
   Home();
   delay(1000);

    Serial2.println("M310 1");
    delay(4000);
    Serial2.println("M313 100");

    Wire.begin(0x69);
    while(true){
      rankos_online();
      if(pirma_ranka_dirba == true){ break; }
    }
}

/*********************************************************************************************************
** Function name:       Serialread
** Descriptions:        import data to rxbuffer
** Input parametersnone:
** Output parameters:
** Returned value:
*********************************************************************************************************/
void Serialread()
{
  while (Serial1.available()) {
    uint8_t data = Serial1.read();
    if (RingBufferIsFull(&gSerialProtocolHandler.rxRawByteQueue) == false) {
      RingBufferEnqueue(&gSerialProtocolHandler.rxRawByteQueue, &data);
    }
  }
}
/*********************************************************************************************************
** Function name:       Serial_putc
** Descriptions:        Remap Serial to Printf
** Input parametersnone:
** Output parameters:
** Returned value:
*********************************************************************************************************/
int Serial_putc( char c, struct __file * )
{
  Serial.write( c );
  return c;
}

/*********************************************************************************************************
** Function name:       printf_begin
** Descriptions:        Initializes Printf
** Input parameters:
** Output parameters:
** Returned value:
*********************************************************************************************************/
void printf_begin(void){
  fdevopen( &Serial_putc, 0 );
}

/*********************************************************************************************************
** Function name:       InitRAM
** Descriptions:        Initializes a global variable
** Input parameters:    none
** Output parameters:   none
** Returned value:      none
*********************************************************************************************************/
void InitRAM(void)
{
    //Set JOG Model
    gJOGJointParams.velocity[0] = 100;
    gJOGJointParams.velocity[1] = 100;
    gJOGJointParams.velocity[2] = 100;
    gJOGJointParams.velocity[3] = 100;
    gJOGJointParams.acceleration[0] = 80;
    gJOGJointParams.acceleration[1] = 80;
    gJOGJointParams.acceleration[2] = 80;
    gJOGJointParams.acceleration[3] = 80;

    gJOGCoordinateParams.velocity[0] = 100;
    gJOGCoordinateParams.velocity[1] = 100;
    gJOGCoordinateParams.velocity[2] = 100;
    gJOGCoordinateParams.velocity[3] = 100;
    gJOGCoordinateParams.acceleration[0] = 80;
    gJOGCoordinateParams.acceleration[1] = 80;
    gJOGCoordinateParams.acceleration[2] = 80;
    gJOGCoordinateParams.acceleration[3] = 80;

    gJOGCommonParams.velocityRatio = 50;
    gJOGCommonParams.accelerationRatio = 50;
   
    gJOGCmd.cmd = AP_DOWN;
    gJOGCmd.isJoint = JOINT_MODEL;

    //Set PTP Model
    gPTPCoordinateParams.xyzVelocity = 100;
    gPTPCoordinateParams.rVelocity = 100;
    gPTPCoordinateParams.xyzAcceleration = 100;
    gPTPCoordinateParams.rAcceleration = 100;

    gPTPCommonParams.velocityRatio = 20;
    gPTPCommonParams.accelerationRatio = 20;

    gPTPCmd.ptpMode = MOVJ_XYZ;
    
    gQueuedCmdIndex = 0;
   moveArm(startPosX, startPosY, startPosZ, startPosR);
    //moveranka(startPosX, startPosY, startPosZ, startPosR, true);
}
/*********************************************************************************************************
** Function name:       loop
** Descriptions:        Program entry
** Input parameters:    none
** Output parameters:   none
** Returned value:      none
*********************************************************************************************************/

void loop(){
    int mygtukas = digitalRead(sensor_pin);
    int belt = digitalRead(belt_on_pin);
    
      if(mygtukas == LOW){
        while(true){
          belt_off();
          Serial.println("Vygdom");
          moveArm(paeimimo_point_pirminis[0], paeimimo_point_pirminis[1], paeimimo_point_pirminis[2]+50, paeimimo_point_pirminis[3]);
          ProtocolProcess();
          delay(3000);
          
          moveArm(paeimimo_point_pirminis[0], paeimimo_point_pirminis[1], paeimimo_point_pirminis[2], paeimimo_point_pirminis[3]);
          ProtocolProcess();
          delay(3000);
          
          Vakumas(true);
          delay(3000);
          
          moveArm(paeimimo_point_pirminis[0], paeimimo_point_pirminis[1], paeimimo_point_pirminis[2]+50, paeimimo_point_pirminis[3]);
          ProtocolProcess();
          delay(3000);
          
          movejoint(darbinis_point[0], darbinis_point[1], darbinis_point[2], darbinis_point[3]);
          ProtocolProcess();
          delay(3000);
          
          movejoint(darbinis_pointas_antras[0], darbinis_pointas_antras[1], darbinis_pointas_antras[2], darbinis_pointas_antras[3]);
          ProtocolProcess();
          delay(3000);
          
          moveArm(paeimimo_point_antrinis[0], paeimimo_point_antrinis[1], paeimimo_point_antrinis[2]+50, paeimimo_point_antrinis[3]);
          ProtocolProcess();
          delay(3000);
          
          moveArm(paeimimo_point_antrinis[0], paeimimo_point_antrinis[1], paeimimo_point_antrinis[2], paeimimo_point_antrinis[3]);
          ProtocolProcess();
          delay(3000);
          
           Vakumas(false);
           delay(3000);

          moveArm(paeimimo_point_antrinis[0], paeimimo_point_antrinis[1], paeimimo_point_antrinis[2]+50, paeimimo_point_antrinis[3]);
          ProtocolProcess();
          delay(3000);
          break;
         }
         belt_off();
       }  
    else{
      movejoint(darbinis_point[0], darbinis_point[1], darbinis_point[2], darbinis_point[3]);
     // belt_on();
    }

      if(belt == HIGH){
         belt_on();
         digitalWrite(led,HIGH);
       }
      else{
        digitalWrite(led,LOW);
          belt_off();
          
      }
      
    delay(100);
  
}

