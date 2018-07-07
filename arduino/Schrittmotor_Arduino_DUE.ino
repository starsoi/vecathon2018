



#include <SPI.h>
#include "Ucglib.h"

#include <AccelStepper.h>

Ucglib_ILI9341_18x240x320_HWSPI ucg(/*cd=*/ 26, /*cs=*/ 24, /*reset=*/ 22); //at Mega and Due

#define Display_Debug

// The Right Stepper pins  
#define STEPPER1_DIR_PIN 3
#define STEPPER1_STEP_PIN 2
// The Left stepper pins
#define STEPPER2_DIR_PIN 7
#define STEPPER2_STEP_PIN 6

//Command ID
#define IDENT 1234


//Deadtime
#define DEADTIME 500

//Deadstep
#define DEADSTEP 0

// Setup Coefficient Rotation / Distance
#define K_M1_Rotation 13
#define K_M2_Rotation 13

#define K_M1_Distance 58//15.8
#define K_M2_Distance 58//15.8

#define DISTANCE_MAX 1000000
#define ROTATION_MAX 180

#define M1_SPEED_MAX 600
#define M1_ACCEL_MAX 100

#define M2_SPEED_MAX 600
#define M2_ACCEL_MAX 100

#define START_ROTATION 0
#define START_DISTANCE 0


long Rotation = 0; // minus: turn left; plus: turn right;
long pre_Rotation = 0;
long Distance = 0; // minus: backwards; plus: forwards
long pre_Distance = 0;

long Fire = 0;

int Command_ID = 0;

bool Rotation_Completed = true;
bool Distance_Completed = true;

bool Flag_PC_Command = false;

//SoftwareSerial mySerial(10, 11); // Rx 10 ; Tx 11

// Define some steppers and the pins the will use
AccelStepper stepper1(AccelStepper::DRIVER, STEPPER1_STEP_PIN, STEPPER1_DIR_PIN);
AccelStepper stepper2(AccelStepper::DRIVER, STEPPER2_STEP_PIN, STEPPER2_DIR_PIN);

void SetRotation(long Rotation){
  //if (Rotation < -ROTATION_MAX) Rotation = -ROTATION_MAX; else if (Rotation > ROTATION_MAX) Rotation = ROTATION_MAX;

   stepper1.moveTo(-K_M1_Rotation * Rotation);
   stepper2.moveTo(-K_M2_Rotation * Rotation);
/*
   Serial.print("Rotation Stepper1: ");
   Serial.print(stepper1.distanceToGo(), DEC);
   Serial.print("; Rotation Stepper2: ");
   Serial.println(stepper2.distanceToGo(), DEC);
*/
  if ((stepper1.distanceToGo() <= DEADSTEP) && (stepper2.distanceToGo() <= DEADSTEP) && (stepper1.distanceToGo() >= -DEADSTEP) && (stepper2.distanceToGo() >= -DEADSTEP)) {

      Rotation_Completed = true;
      
      pre_Rotation = Rotation;
         
      Serial1.print(" Rot:");
      Serial1.print( Rotation, DEC);
      Serial1.println(";");
      delay(DEADTIME);
      
      //ready to go forwards or backwards
      stepper1.setCurrentPosition(-K_M1_Distance * pre_Distance);
      stepper2.setCurrentPosition(K_M2_Distance * pre_Distance);
    }  
}

void SetDistance(long Distance){
  if (Distance < -DISTANCE_MAX) Distance = -DISTANCE_MAX; else if (Distance > DISTANCE_MAX) Distance = DISTANCE_MAX;

   stepper1.moveTo(-K_M1_Distance * Distance);
   stepper2.moveTo(K_M2_Distance * Distance);
  /*
   Serial.print("Distance Stepper1: ");
   Serial.print(stepper1.distanceToGo(), DEC);
   Serial.print("; Distance Stepper2: ");
   Serial.println(stepper2.distanceToGo(), DEC);
   */
  
 if ((stepper1.distanceToGo() <= DEADSTEP) && (stepper2.distanceToGo() <= DEADSTEP) && (stepper1.distanceToGo() >= -DEADSTEP) && (stepper2.distanceToGo() >= -DEADSTEP)) {

      Distance_Completed = true;
      pre_Distance = Distance;
      
      Serial1.print(" Dist:");
      Serial1.print( Distance, DEC);
      Serial1.println(";"); 
      Serial1.println("OK!");     

/*
      if (Flag_PC_Command == true) {
        mySerial.println("Movement Completed!");
        Flag_PC_Command = false;
      }
      */

      delay(DEADTIME);
    }  

  
}

void ReadCommand(void){
      if (Serial1.available() > 0) {
         // read the incoming byte:

 //        while (Serial1.available())
 //        ucg.print(Serial1.readString());
    
         Command_ID = Serial1.parseInt();


         
         if(Command_ID == IDENT){
         Rotation = Serial1.parseInt();
         Distance = Serial1.parseInt();
         Fire = Serial1.parseInt();    
          
               
         stepper1.stop();
         stepper2.stop();
         // say what you got:
         Serial1.print("received Rotation: ");
         Serial1.print(Rotation, DEC);
         Serial1.print(";   received Distance: ");
         Serial1.print(Distance, DEC);
         Serial1.println(";");
         
         
         // turn first, so load the last rotation
         stepper1.setCurrentPosition(-K_M1_Rotation * pre_Rotation);
         stepper2.setCurrentPosition(-K_M2_Rotation * pre_Rotation);
         
         //Set Flag
         Rotation_Completed = false;
         Distance_Completed = false;
         }

         else {
          Serial1.println("Command is wrong!");
          Rotation = pre_Rotation;
          Distance = pre_Distance;   
         }
     }
         /*
         //NodeMCU Software Serial
         if (mySerial.available() > 0) {
         
         // read the incoming byte:
         Command_ID = mySerial.parseInt();
         Rotation = Serial.parseInt();
         Distance = Serial.parseInt(); 
         
         if(Command_ID == IDENTITY){ 

         
         stepper1.stop();
         stepper2.stop();
         // say what you got:
         Serial.print("received Rotation from PC: ");
         Serial.print(Rotation, DEC);
         Serial.print(";   received Distance from PC: ");
         Serial.print(Distance, DEC);
         Serial.println(";");
         
         // turn first, so load the last rotation
         stepper1.setCurrentPosition(-K_M1_Rotation * pre_Rotation);
         stepper2.setCurrentPosition(-K_M2_Rotation * pre_Rotation);
         
         //Set Flag
         Rotation_Completed = false;
         Distance_Completed = false;
         Flag_PC_Command = true;
         }

         else {
          mySerial.println("Command from PC is wrong!");
          Rotation = pre_Rotation;
          Distance = pre_Distance;  
         }
     }
     */
     
}

void Display(void){
    ucg.clearScreen();
    ucg.setPrintPos(25,25);
    ucg.print("Current Rotation = ");
    ucg.print(Rotation);

    
    ucg.setPrintPos(25,50);
    ucg.print("Current Distance = ");
    ucg.print(Distance);

    ucg.setPrintPos(25,75);
    ucg.print("Fire = ");
    ucg.print(Fire);
}

void Display_Init(void){
    ucg.begin(UCG_FONT_MODE_TRANSPARENT);
    ucg.clearScreen();
    ucg.setFont(ucg_font_ncenR12_tr);
    ucg.setColor(255, 0, 255);
    ucg.setColor(1, 100, 0,0);
    ucg.setRotate90();
    ucg.setRotate90();
}
void setup()
{  
    Serial1.begin(9600);
    Serial1.println("System start");

    #ifdef Display_Debug 
    Display_Init();
    #endif

    

    Rotation = START_ROTATION;
    Distance = START_DISTANCE;

    pre_Rotation = Rotation;
    pre_Distance = Distance;

    #ifdef Display_Debug 
    Display();
    #endif
    
        
    stepper1.setMaxSpeed(M1_SPEED_MAX);
    stepper1.setAcceleration(M1_ACCEL_MAX);
    stepper1.setCurrentPosition(-K_M1_Rotation * Rotation);
    //stepper1.moveTo(0);

  
    stepper2.setMaxSpeed(M2_SPEED_MAX);
    stepper2.setAcceleration(M2_ACCEL_MAX);
    stepper2.setCurrentPosition(-K_M2_Rotation * Rotation);
    //stepper2.moveTo(0);


     
   Serial1.print("Rot:");
   Serial1.print(Rotation);
   Serial1.print(";Dist:");
   Serial1.print(Distance, DEC);
   Serial1.println(";");
   
   Serial1.println("OK!");   

   
    
}

void loop()
{

    //stepper.stop(); // Stop as fast as possible: sets new target
    
    ReadCommand();
    
    #ifdef Display_Debug 
    Display();
    #endif

    if( Rotation_Completed == false) SetRotation(Rotation); 
    else
    {
      if( Distance_Completed == false) SetDistance(Distance);
    }
    
    stepper1.run();
    stepper2.run();
    
    // Change direction at the limits
    //if (stepper1.distanceToGo() == 0)	;//stepper1.moveTo(-stepper1.currentPosition());  
    //if (stepper2.distanceToGo() == 0) ;//stepper2.moveTo(-stepper2.currentPosition());
   /*
   Rotation = 25;
   Distance = 20;
   Fire = 10;
   */
   
}
