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
bool endeffectorGripper = false; // indicate type of end effector: true for gripper, false for vacuum cup

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

int xhome_mygtuko_pin = 7;
int yhome_mygtuko_pin = 6;

int xhome_mygtukas;
int yhome_mygtukas;
int led = 13;

Pose robotPose;

double darbinis_point[4] = {-1.8697, 15.4835 , 0.4057, 0};  // JOINT 
double paeimimo_point[4] = {42.2502, 15.4835 , 0.4057 ,0};  // JOINT

double spalvos_check_point[4] = {272.4153, -94.6378, 24.6840, 0};      //kordinates
double belt_point[4] = {198.0025, -85.9966, 20.2917, 0};                //kordinates

int Detuve[] = {0, 0, 0, 0, 0, 0, 0, 0, 0};

int pirma_eile = 1;
int antra_eile = 2;
int trecia_eile = 3;

bool start_komanda = true;

double point1[4] = {217.6357, 120.1758 , -43.00   , 0};
double point2[4] = {178.5357, 122.0088 , -43.00   , 0};
double point3[4] = {138.6986, 124.6019 , -43.00   , 0};
double point4[4] = {218.9079, 159.9332 , -43.00   , 0};
double point5[4] = {180.2213, 162.9806 , -43.00   , 0};
double point6[4] = {140.9093, 165.2853 , -43.00   , 0};
double point7[4] = {220.0648, 200.6643 , -43.00   , 0};
double point8[4] = {181.4996, 203.4047 , -43.00   , 0};
double point9[4] = {140.9395, 203.4053 , -43.00   , 0};

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
void setup() {
  Serial.begin(115200);
  Serial1.begin(115200);
  printf_begin();
  //Set Timer Interrupt
  FlexiTimer2::set(100, Serialread);
  FlexiTimer2::start();
  Serial.println("Atnaujinta 2023-03-28 14:21");

  pinMode(xhome_mygtuko_pin, INPUT_PULLUP);
  pinMode(yhome_mygtuko_pin, INPUT_PULLUP);
  pinMode(led,OUTPUT);


  InitRAM();

  ProtocolInit();

  SetJOGJointParams(&gJOGJointParams, true, &gQueuedCmdIndex);

  SetJOGCoordinateParams(&gJOGCoordinateParams, true, &gQueuedCmdIndex);

  SetJOGCommonParams(&gJOGCommonParams, true, &gQueuedCmdIndex);

  printf("\r\n======Main Program loop started======\r\n");

  SetPTPCmd(&gPTPCmd, true, &gQueuedCmdIndex);
  
   ProtocolProcess();
   delay(2000);
   Home();
   //moveArm(180, 50, 30, 30);
   detaliu_pozicija(pirma_eile, antra_eile, trecia_eile);
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
}
/*********************************************************************************************************
** Function name:       loop
** Descriptions:        Program entry
** Input parameters:    none
** Output parameters:   none
** Returned value:      none
*********************************************************************************************************/

void loop(){
    movejoint(darbinis_point[0], darbinis_point[1], darbinis_point[2], darbinis_point[3]);
    // ProtocolProcess();
    /*movejoint(paeimimo_point[0], paeimimo_point[1], paeimimo_point[2], paeimimo_point[3]);
    moveranka(Detuves_kordinates[0][0], Detuves_kordinates[0][1], Detuves_kordinates[0][2], Detuves_kordinates[0][3], true);
    while(true){
       ProtocolProcess();
      delay(5000);
      break;
    }*/
    //moveranka(Detuves_kordinates[0][0], Detuves_kordinates[0][1], Detuves_kordinates[0][2]+50, Detuves_kordinates[0][3], false);
    //delay(5000);
    
    /* delay(1000);
     moveArm(Detuves_kordinates[0][0], Detuves_kordinates[0][1], Detuves_kordinates[0][2]+50, Detuves_kordinates[0][3]);
     movejoint(paeimimo_point[0], paeimimo_point[1], paeimimo_point[2], paeimimo_point[3]);
     movejoint(darbinis_point[0], darbinis_point[1], darbinis_point[2], darbinis_point[3]);
     movejoint(spalvos_check_point[0], spalvos_check_point[1], spalvos_check_point[2], spalvos_check_point[3]);*/
     digitalWrite(led,HIGH);

     if(start_komanda == true){
        for(int i=0; i<=8; i++){
           if(Detuve[i] == 1){
             movejoint(paeimimo_point[0], paeimimo_point[1], paeimimo_point[2], paeimimo_point[3]);
             ProtocolProcess();
             delay(100);
             
             moveArm(Detuves_kordinates[i][0], Detuves_kordinates[i][1], Detuves_kordinates[i][2]+50, Detuves_kordinates[i][3]);
             ProtocolProcess();
             delay(100);
             
             moveArm(Detuves_kordinates[i][0], Detuves_kordinates[i][1], Detuves_kordinates[i][2], Detuves_kordinates[i][3]);
             ProtocolProcess();
             delay(3000);
             
             Vakumas(true);
             ProtocolProcess();
             delay(100);
             
             moveArm(Detuves_kordinates[i][0], Detuves_kordinates[i][1], Detuves_kordinates[i][2]+50, Detuves_kordinates[i][3]);
             ProtocolProcess();
             delay(100);
             
             movejoint(paeimimo_point[0], paeimimo_point[1], paeimimo_point[2], paeimimo_point[3]);
             ProtocolProcess();
             delay(100);
             
             movejoint(darbinis_point[0], darbinis_point[1], darbinis_point[2], darbinis_point[3]);
             ProtocolProcess();
             delay(100);
             
             moveArm(spalvos_check_point[0], spalvos_check_point[1], spalvos_check_point[2], spalvos_check_point[3]);
             ProtocolProcess();
             delay(3000);
             
             moveArm(spalvos_check_point[0], spalvos_check_point[1], spalvos_check_point[2]+25, spalvos_check_point[3]);
             ProtocolProcess();
             delay(100);
             
             moveArm(belt_point[0], belt_point[1], belt_point[2], belt_point[3]);
             ProtocolProcess();
             delay(100);
     
             Vakumas(false);
             ProtocolProcess();
             delay(3000);

             moveArm(belt_point[0], belt_point[1], belt_point[2]+25, belt_point[3]);
             ProtocolProcess();
             delay(100);
             
             movejoint(darbinis_point[0], darbinis_point[1], darbinis_point[2], darbinis_point[3]); 
             ProtocolProcess();
             delay(100);
             
             Detuve[i] = 0;
           }
        }
        
        start_komanda = false;
        Serial.println("Baigtas procesas");
     }
     
     
//      while(true){
//        if(Detuve[i] == 1){
//             movejoint(paeimimo_point[0], paeimimo_point[1], paeimimo_point[2], paeimimo_point[3]);
//             moveArm(Detuves_kordinates[i][0], Detuves_kordinates[i][1], Detuves_kordinates[i][2], Detuves_kordinates[i][3]);
//             delay(10000);
//             Vakumas(true);
//             moveArm(Detuves_kordinates[i][0], Detuves_kordinates[i][1], Detuves_kordinates[i][2]+50, Detuves_kordinates[i][3]);
//             movejoint(paeimimo_point[0], paeimimo_point[1], paeimimo_point[2], paeimimo_point[3]);
//             movejoint(darbinis_point[0], darbinis_point[1], darbinis_point[2], darbinis_point[3]);
//             moveArm(spalvos_check_point[0], spalvos_check_point[1], spalvos_check_point[2], spalvos_check_point[3]);
//             moveArm(spalvos_check_point[0], spalvos_check_point[1], spalvos_check_point[2]+25, spalvos_check_point[3]);
//             moveArm(belt_point[0], belt_point[1], belt_point[2], belt_point[3]);
//             Vakumas(false);
//             moveArm(belt_point[0], belt_point[1], belt_point[2]+25, belt_point[3]);
//             movejoint(darbinis_point[0], darbinis_point[1], darbinis_point[2], darbinis_point[3]); 
//           }
//         i++;
//           if(i>8){ 
//              start_komanda = false;
//              Serial.println("Baigtas procesas");
//              i= 0; 
//              break;
//           }
//      }
      
     delay(5000);
    
    
}

