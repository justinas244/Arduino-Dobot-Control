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
int rusiavimo_optikos_pin = 2;
int rusiavimo_optika = 0;

double darbinis_point[4] = {-91.7907, 1.1584 , -2.8497, 0};  // JOINT 
double paeimimo_point[4] = {-41.3705, 1.1584 , -2.8497 ,0};  // JOINT

double spalvos_check_point[4] = {-96.5552, -311.0453, 21.0647, 0};       //kordinates
double belt_point[4] = {-107.0058, -221.1881, 6.6209, 0};                //kordinates

double rusiavimo_point[4] = {89.5724, 2.1488, -6.6158, 0};   //Joint
double rusiavimo_darbinis[4] = {7.5669, 16.2814, -9.7413, 0};   //Joint

double rusiavimo_paemimas[4] = {2.5839, 218.6746, -3.1779, 0};   //kordinates

double kubelio_skirtumas = 24.5; 

int rusiavimo_tvarka[4] = {1,2,3,4};
int kubeliu_skaicius_rusiavime[4] = {0,0,0,0}; 
   
// Spalvu kodai
// 1 - raudona
// 2 - geltona
// 3 - zalia
// 4 - melyna

double rusiavimo_point1[4] = {238.1382, -17.9048 , -39.2941   , 0};
double rusiavimo_point2[4] = {238.9785,  19.6073 , -37.4695   , 0};
double rusiavimo_point3[4] = {240.0355,  55.1753 , -37.5413   , 0};
double rusiavimo_point4[4] = {241.9576,  92.3277 , -37.5419   , 0};

double rusiavimo_kordinates[4][4] {        //cartesian cordinate
  {rusiavimo_point1[0], rusiavimo_point1[1], rusiavimo_point1[2], rusiavimo_point1[3]},
  {rusiavimo_point2[0], rusiavimo_point2[1], rusiavimo_point2[2], rusiavimo_point2[3]},
  {rusiavimo_point3[0], rusiavimo_point3[1], rusiavimo_point3[2], rusiavimo_point3[3]},
  {rusiavimo_point4[0], rusiavimo_point4[1], rusiavimo_point4[2], rusiavimo_point4[3]},
 };

int Detuve[] = {0, 0, 0, 0, 0, 0, 0, 0, 0};

int pirma_eile = 12;
int antra_eile = 12;
int trecia_eile = 12;

bool start_komanda = false;

// kordinates

double point1[4] = {131.7854, -212.5771, -40.0   , 0};
double point2[4] = {131.7833, -172.2545, -40.0   , 0};
double point3[4] = {132.0465, -133.6148, -40.0   , 0};
double point4[4] = {172.6083, -212.9910, -40.0   , 0};
double point5[4] = {171.4071, -172.4565, -40.0   , 0};
double point6[4] = {172.2950, -133.8883, -40.0  , 0};
double point7[4] = {212.3032, -212.4667, -40.0   , 0};
double point8[4] = {212.3538, -171.9321, -40.0   , 0};
double point9[4] = {212.3299, -132.7646, -40.0   , 0};

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
// TCS230 or TCS3200 pins wiring to Arduino
#define S0 4
#define S1 5
#define S2 6
#define S3 7
#define sensorOut 8
#define led 9

// Stores frequency read by the photodiodes
int redFrequency = 0;
int greenFrequency = 0;
int blueFrequency = 0;


int raudona[6]={15,45,45,110,38,90};

int geltona[6]={15,30,20,35,28,65};

int zalia[6]={40,80,30,55,48,85};

int melyna[6]={35,95,40,75,15,42};


int aptikta_spalva = 0;
bool buvo_aptikta = false;
int kk = 1;

int start_pin = 28;
int belt_on_pin = 31;

int antros_rankos_adresas = 0x69;
int wifi_adresas = 0x67;

bool antra_ranka_dirba = false;
bool wifi_dirba = false;

#define ARRAY_SIZE 8     // Number of values in the array
int dataArray[ARRAY_SIZE] = {0, 0, 0, 0, 0, 0, 0, 0};  // Array to be sent

void setup() {
  Serial.begin(115200);
  Serial1.begin(115200);
  Serial2.begin(115200);

  delay(1000);
  Serial2.println("M310 1");
  delay(4000);
  Serial2.println("M313 100");
  
  printf_begin();
  //Set Timer Interrupt
  FlexiTimer2::set(100, Serialread);
  FlexiTimer2::start();
  Serial.println("Atnaujinta 2023-03-28 14:21");

  pinMode(xhome_mygtuko_pin, INPUT_PULLUP);
  pinMode(yhome_mygtuko_pin, INPUT_PULLUP);
  pinMode(rusiavimo_optikos_pin, INPUT_PULLUP);
  
  pinMode(start_pin, INPUT_PULLUP);
  
  pinMode(led,OUTPUT);
   // Setting the outputs
  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);
  pinMode(led, OUTPUT);
  pinMode(belt_on_pin,OUTPUT);
  delay(50);
  digitalWrite(belt_on_pin,LOW);
    
  // Setting the sensorOut as an input
  pinMode(sensorOut, INPUT);
  
  // Setting frequency scaling to 20%
  digitalWrite(S0,HIGH);
  digitalWrite(S1,LOW);
   // Begins serial communication 

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
   detaliu_pozicija(pirma_eile, antra_eile, trecia_eile);
   delay(1000);
    Serial2.println("M310 1");
    delay(4000);
    Serial2.println("M313 -140");
    delay(2000);

    Wire.begin(0x68);
    //Wire.onReceive (receiveEvent); 
    while(true){
      rankos_online();
      if(antra_ranka_dirba == true){ break; }
    }
   digitalWrite(belt_on_pin,LOW);
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
  int startas = digitalRead(start_pin);
   
   if(startas == LOW){
     start_komanda = true;
   }
    
    movejoint(darbinis_point[0], darbinis_point[1], darbinis_point[2], darbinis_point[3]);
     
     if(start_komanda == true){
       //belt_on();
        for(int i=0; i<=8; i++){
           if(Detuve[i] == 1){
             
             movejoint(paeimimo_point[0], paeimimo_point[1], paeimimo_point[2], paeimimo_point[3]);
             ProtocolProcess();
             delay(1000);
             
             moveArm(Detuves_kordinates[i][0], Detuves_kordinates[i][1], Detuves_kordinates[i][2]+50, Detuves_kordinates[i][3]);
             ProtocolProcess();
             delay(1000);
             
             moveArm(Detuves_kordinates[i][0], Detuves_kordinates[i][1], Detuves_kordinates[i][2], Detuves_kordinates[i][3]);
             ProtocolProcess();
             delay(3000);
             
             Vakumas(true);
             delay(2000);
             
             moveArm(Detuves_kordinates[i][0], Detuves_kordinates[i][1], Detuves_kordinates[i][2]+50, Detuves_kordinates[i][3]);
             ProtocolProcess();
             delay(1000);
             
             movejoint(paeimimo_point[0], paeimimo_point[1], paeimimo_point[2], paeimimo_point[3]);
             ProtocolProcess();
             delay(1000);
             
             movejoint(darbinis_point[0], darbinis_point[1], darbinis_point[2], darbinis_point[3]);
             ProtocolProcess();
             delay(1000);
             
             moveArm(spalvos_check_point[0], spalvos_check_point[1], spalvos_check_point[2], spalvos_check_point[3]);
             ProtocolProcess();
             delay(3000);
          
             nuskaitymas_spalvos();
       
             moveArm(spalvos_check_point[0], spalvos_check_point[1], spalvos_check_point[2]+18, spalvos_check_point[3]);
             ProtocolProcess();
             delay(2000);
             
             
             moveArm(belt_point[0], belt_point[1], belt_point[2], belt_point[3]);
             ProtocolProcess();
             delay(4000);
             
             digitalWrite(belt_on_pin,HIGH);
                      
             Vakumas(false);
             delay(3000);

             moveArm(belt_point[0], belt_point[1], belt_point[2]+25, belt_point[3]);
             ProtocolProcess();
             delay(1000);
             
             movejoint(darbinis_point[0], darbinis_point[1], darbinis_point[2], darbinis_point[3]); 
             ProtocolProcess();
             
             delay(5500);
             digitalWrite(belt_on_pin,LOW);
             
//            if(kk>4){
//              kk =1;
//             }
             
             rusiavimas(aptikta_spalva);
             //kk++;
              
             //Detuve[i] = 0;
             buvo_aptikta = false;
             
           }
        }
        
        start_komanda = false;
        Serial.println("Baigtas procesas");

        for(int i=0; i<=3; i++){
          kubeliu_skaicius_rusiavime[i]=0;
        }
     }
     delay(100);
    
}

