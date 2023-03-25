#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_Sensor.h>
#include <QMC5883LCompass.h>
#include <AccelStepper.h>
#include <Servo.h>
//#include <NewPing.h>
#include <HCSR04.h>
QMC5883LCompass compass;
#define SCREEN_WIDTH 128 // OLED display width,  in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels


////////////////////////////////// Tower of hanoi variables /////////////////////////////////////////////

#define NUM_SENSORS 12   //For floor pattern
#define NUM_VALUES 6   //for floor pattern
#define motorPin1  51      // IN1 on the ULN2003 driver
#define motorPin2  49      // IN2 on the ULN2003 driver
#define motorPin3  47     // IN3 on the ULN2003 driver
#define motorPin4  45     // IN4 on the ULN2003 driver



////////////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////// color sensor pins//////////////////////////////
#define S0_PIN_box 9
#define S1_PIN_box 10
#define S2_PIN_box 11
#define S3_PIN_box 12
#define OUT_PIN_box  13

#define S0_PIN_floor 5
#define S1_PIN_floor 4
#define S2_PIN_floor 7
#define S3_PIN_floor 6
#define OUT_PIN_floor  8


int red = 0;  
int green = 0;  
int blue = 0; 


/////////////////////////////////////////////////////////////////////////////////

// Define the AccelStepper interface type; 4 wire motor in half step mode:
#define MotorInterfaceType 8

AccelStepper stepper = AccelStepper(MotorInterfaceType, motorPin1, motorPin3, motorPin2, motorPin4);
Servo Gripper;

// declare an SSD1306 display object connected to I2C
Adafruit_SSD1306 oled(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

/////////////////////////// ultrasonic //////////////////////////////////

int REFERENCE_DISTANCE=10;
//float distance1,error1,error2,errorD1,errorD2,previous_errorD1,previous_errorD2,integral1,integral2,derivative1,derivative2,error3,error4,PID_ErrorD1,PID_ErrorD2;
float distance1,distance3,distance4,distance5,distance6,error1,error2,errorD1,errorD,previous_errorD1,previous_errorD,integral1,integral,derivative1,derivative,error3,error4,PID_ErrorD1,PID_ErrorD2,PID_ErrorD;
float ultrasonic_distance[7];
int left_motor_speed,right_motor_speed,initial_speed=150,MAX_SPEED=225,MIN_SPEED=80;
float ultrasonic_list1[5];
int ultrasonic_count=0;
int ultrasonic_part=0;


const int trigPin1 = 27;//distance1
const int echoPin1 = 26;
const int trigPin2 = 29;//distance2
const int echoPin2 = 28;
const int trigPin3 = 31;//distance3
const int echoPin3 = 30;
const int trigPin4 = 23;//distance4
const int echoPin4 = 22;
const int trigPin5 = 25;//distance5
const int echoPin5 = 24;
const int trigPin6 = 33;//distance6
const int echoPin6 = 32;
const int trigPin7 = 35;//distance6
const int echoPin7 = 34;
/////////////////////////////////////////////////////////////////////////

//////////////////////// floorPattern for tower of hanoi //////////////////////////////////
int floorCounter = 0; 
String junctionType = "None";
bool junctionDetected = false; 
String linePathPattern = "None"; 
bool pathCrossing = false;
int sensorValues[NUM_SENSORS][NUM_VALUES];  // For 12 line sensors to store 10 previous values
int lineSensorCount[3] = {0, 0, 0} ;
///////////////////////////////////////////////////////////////////////

////////////////////////////// Tower of hanoi ///////////////////////////////////////////
//Junction mapping
int thJunc[3][4] = {{ 48,  8,  3,  100 },
                    { 32,  4,  2,  12  },
                    { 16,  0,  1,  8   }};     // 100 means finished  and 0 means No path

int thCurrentJuncIndex = 0;   //Current junction   0: 12, 1: 32, 2: 16
int thCurrentJunc[4] = {0, 0, 0, 0}; //Get a copy of current junction and do the shifting
int thCurrentLocation = 100;  //FOr automatic routing to store current place
int thDestination ;
bool thDestinationReached = false;
//int thDirection = 0;
bool thStarted = true;
int thLocationsToMove[100]  = {0, 0, 0, 0};   //5 means the end of the list
int thLocationIndex = 0;
bool thComplete = false;
int thMainDirection = 0;
int thStage = 0;
int thUserInput01 = 2; //Change these
int thUserInput02 = 1; // Change thses
int thPlacingTheBox = 0;

int thExploredBoxes[3] = {48, 32, 16}; // 0: largest box 1: middle box 2: Smallest box
int thGripperCommands[10][3] = {{0,5,1},{0,6,2},{0,4,2},{1,6,2},{0,3,2},{2,6,2}};                     // vertical position :: gripper position :: vertical position
int thGripperCommandCounter = 0;
bool thBoxGrab = true;

bool thRedBoxDetected  = false;
bool thGreenBoxDetected  = false;
//bool thBlueBoxDetected  = false;

  //String thJuncType;
/////////////////////////////////////////////////////////////////////////////////////////////


///////////////////////////// IR sensor pins /////////////////////////////////////////////

const int ProxSensor_1=A0;       // 6 Right
const int ProxSensor_2=A1;       // 5 Right
const int ProxSensor_3=A2;       // 4 Right
const int ProxSensor_4=A3;       // 3 Right
const int ProxSensor_5=A4;       // 2 Right
const int ProxSensor_6=A5;       // 1 Right
const int ProxSensor_7=A6;       // 1 left
const int ProxSensor_8=A7;       // 2 left
const int ProxSensor_9=A8;       // 3 left
const int ProxSensor_10=A9;       // 4 left
const int ProxSensor_11=A10;       // 5 left
const int ProxSensor_12=A11;       // 6 left

///////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////// Dip switch pins //////////////////////////
int switchPinOne = 46; 
int switchPinTwo = 44; 
int switchPinThree = 42; 

int buttonOnePin = 52;     //change these values
int buttonTwoPin = 50;  
int buttonThreePin = 48; 

////////////////////////////////////////////////////////////////////


int inputVal[12] = {0,0,0,0,0,0,0,0,0,0,0,0};

  // Motor A connections
  int enA = 2;
  int in1 = 39;    //5   
  int in2 = 37;   //4 
  // Motor B connections
  int enB = 3;
  int in3 = 43;     //7
  int in4 = 41;    //6

  //Buzzer pin
  int buzzerPin = 36; // digital pin 8

  int count = 0;  //delete later

  int error_list[5] = {0,0,0,0,0};  //For both pid and straight line pid functions (Change if necessary)

  //////////   For straight Lines   ///////// //////
  float kpS = 0.55;
  float kdS = 0.3;
  float kiS = 0.0000;

  ////////////  For curved paths   //////////////////
  float kp = 0.6;
  float kd = 0.3;
  float ki = 0.0000;

  ///////  for ultrasonic ///////////////

  float USKp = 1.6;
  float USKd = 0.3;
  float USKi = 0.0000;

  ////////////// For both curve and straight pids ///////
  int error = 0;
  int d_error = 0;
  int i_error = 0;


  ////////////  Variables for Arrow Follow  //////////////////
  float AF_kp = 1.8;
  float AF_kd =0.8;

  float AF_ki = 0.0000;

  bool AF_whiteDetected = false;
  bool AF_blackAgain = true;

  int AF_leftSensorVal = 0;
  int AF_rightSensorVal = 0;

  int AF_currentAllIR = 0;
  int AF_currentLeftIR = 0;
  int AF_currentRightIR = 0;
  bool AF_start = true;
  bool AF_end = true;



///////////////////////////////////////////////////////ISHARA ////////////////////////////////////////

char MZcurentDerection = 'r'; //at the green squre
char MZjunctionType='I';
char MZpreviousMode;
int MZX=0;//cordinates at the grean squre
int MZY=0;
int MZpreviousAction=0;// 4 for go left////////////6 for go right//////////////8 for go up////2 for go down///(look for keyboard)
int MZturn180=80 ;// used to turn 180
int MZturn90=40;// used to turn 90
int MZrotatedAngle=0;
String MZfloorColor = " ";//color of the floor
//int MZstrtrted=0; //indecate the starting
int LFSensor[12]={0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
float direct;

/////////////////////////////////////////////////////////////////////////////////////////////////////////// ISHARA ////////////////////////
int MZi=0;
int MZpart = 0;// if maze solved this will be 2 and yet to be sloved, this will be 1 (used in the loop)
char MZpath[100] = " ";
char MZmode = ' ';
int MZpid;
unsigned char MZpathLength = 0; // the length of the path
int MZpathIndex = 0; // used to reach an specific array element.

unsigned int MZstat = 0; // solving = 0; reach Maze End = 1
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////// MAIN CONTROLLER /////////////////////////////////////////////////////////////////////

int HelpingStage=0; 

bool buttonOne = false;
bool buttonTwo = false;
bool buttonThree = false;

//////////////////////////// Functions //////////////////////////////////

///////////////////////  for arrow following ///////////////////////////

void ArrowFollowerLoop(){

  AF_arrowReadLineSensors();  

  if (AF_start == true){  
    AF_arrowReadLineSensors();        
    forward(150,135);
    delay(1000);
    AF_start = false;    
  }

  
  
  
  if (AF_whiteDetected == true){

    if(AF_blackAgain == true){
      Stop();
    delay(1000);
    AF_blackAgain = false ;
    printToOled(10, "black");
    }
    
    printToOled(10, "AF_whitedetected");
    if (AF_currentAllIR >= 8){
    printToOled(10, "ramp");
    Stop();
    delay(1000);    
    HelpingStage = 22;
  }
      
    if(AF_currentAllIR > 5){
      turnLeft(180);
      delay(600);
    }
     
    
    else if(AF_currentAllIR>=3){
      if (AF_leftSensorVal > AF_rightSensorVal){
        turnLeft(180);
        delay(1000);
        Stop() ; 
        delay(200)    ;     
        AF_pidArrowFollower();
        delay(1000);
      
        AF_leftSensorVal = 0;
        AF_rightSensorVal = 0;  
        
        printToOled(10, "two");
        Serial.println("two");
    }

    else if (AF_leftSensorVal > AF_rightSensorVal){
        turnRight(180);
        delay(1000);   
        Stop() ; 
        delay(200)    ;     
        AF_pidArrowFollower();
        delay(1000);
        
        AF_leftSensorVal = 0;
        AF_rightSensorVal = 0; 
        
        
        printToOled(10, "three");  
        Serial.println("three");                 
    }    
    else{
      forward(120,110);
      delay(200);
      AF_pidArrowFollower();
    }

    }
    
    
    
  }


  if (AF_whiteDetected == false){
    forward(150,135);    
    AF_blackAgain = true ;
    AF_leftSensorVal = 0;
    AF_rightSensorVal = 0;   
    printToOled(10, "end")  ;
    Serial.println("end");       
  }  

  
}

void RampUpLoop(){
  AF_currentAllIR = 0;
  
  if (AF_end = true){
    readLineSensors();
    
    for (int i = 0; i<12;i++){
      if (inputVal[i] == 1){
          AF_currentAllIR += 1;
      }
      
    }
    
    if (AF_currentAllIR < 4){
      AF_end = false;   
      forward(150,135);    
                  
    }
  } 
  
  else{
    pidLineFollower();
    for (int i = 0; i<12;i++){
      AF_currentAllIR += 1;
    }
    if (AF_currentAllIR > 6){
      Stop(); 
      delay(3000); 
      HelpingStage = 20;     
    }    
  } 
  
}

void RampDOwnLoop(){
  AF_currentAllIR = 0;
  
  if (AF_end = true){
    readLineSensors();
    
    for (int i = 0; i<12;i++){
      if (inputVal[i] == 1){
          AF_currentAllIR += 1;
      }
      
    }
    
    if (AF_currentAllIR < 4){
      AF_end = false;   
      forward(150,135);    
                  
    }
  } 
  
  else{
    pidLineFollower();
    for (int i = 0; i<12;i++){
      AF_currentAllIR += 1;
    }
    if (AF_currentAllIR > 6){
      Stop(); 
      delay(3000);      
    }    
  } 
  
}

void AF_arrowReadLineSensors(){            
                          
  inputVal[0]  = analogRead(ProxSensor_1);
  inputVal[1]  = analogRead(ProxSensor_2);
  inputVal[2]  = analogRead(ProxSensor_3);
  inputVal[3]  = analogRead(ProxSensor_4);
  inputVal[4]  = analogRead(ProxSensor_5);
  inputVal[5]  = analogRead(ProxSensor_6);
  inputVal[6]  = analogRead(ProxSensor_7);
  inputVal[7]  = analogRead(ProxSensor_8);
  inputVal[8]  = analogRead(ProxSensor_9);
  inputVal[9]  = analogRead(ProxSensor_10);
  inputVal[10] = analogRead(ProxSensor_11);
  inputVal[11] = analogRead(ProxSensor_12);



  for (int i = 0; i<12; i++){  //Convert analog inputs to digital                       
    if(inputVal[i]<500){
          inputVal[i] = 1;            
    }
    else{
          inputVal[i] = 0;
    }
    Serial.print(inputVal[i]);
    Serial.print(" "); 

  }
  oled.clearDisplay();          // clear display
  oled.setTextSize(1);          // text size
  oled.setTextColor(WHITE);     // text color
  oled.setCursor(0, 40);

  AF_whiteDetected = false;
    
    AF_currentAllIR = 0;
    AF_currentLeftIR = 0;
    AF_currentRightIR = 0;
 

  for (int i = 0 ; i<12 ; i++){
    
      if ((inputVal[i] == 1) && (i<9) && (i>2)){
      AF_whiteDetected = true;
      }
     
      if (inputVal[i] == 1){
         if (i%6 == 0){
        if (i<4){
            AF_leftSensorVal += 1;  
        }
        
        AF_currentLeftIR += 1;       
      }     
      
      else{
        if (i>7){
            AF_rightSensorVal += 1;
        }        
        AF_currentRightIR += 1;
      }    
      AF_currentAllIR =AF_currentLeftIR+AF_currentRightIR;
    }
                  
    oled.print(inputVal[i]); // text to display   
    oled.print("");    
         
    
  
 }
                   
      
  oled.println(""); 
  oled.println(AF_whiteDetected); // text to display
  oled.display();               // show on OLED 
  Serial.println();  

  // shift values
  for (int i = 0; i < NUM_SENSORS; i++) {
    for (int j = NUM_VALUES - 1; j > 0; j--) {
      sensorValues[i][j] = sensorValues[i][j-1];
    }
  } 
              
  for (int i = 0; i < NUM_SENSORS; i++) {
    //sensorValues[i][0] = analogRead(i);
    sensorValues[i][0] = inputVal[i];
  }

  for (int i = 0; i<3; i++){      // for 3 sensor regions
    lineSensorCount[i] = 0; //initial Value
    for(int j = 0; j<4;  j++){
        lineSensorCount[i] += inputVal[4*i+j];
    }
  
  }
}

void AF_pidArrowFollower(){
  //floorPattern();
  AF_arrowReadLineSensors();
  //Error function
  error = 10*(inputVal[0]*6 +inputVal[1]*5 +inputVal[2]*4 + inputVal[3]*3 +inputVal[4]*2 + inputVal[5] -(inputVal[11]*6 +inputVal[10]*5 +inputVal[9]*4 + inputVal[8]*3 +inputVal[7]*2 + inputVal[6]) ) ; //(2(inputVal[0]+inputVal[1]+inputVal[2]+inputVal[3]+inputVal[4]+inputVal[5]+inputVal[6]));

  for (int i = 0; i<4 ; i++){   //Shift and store the error
    error_list[i] = error_list[i+1];
  }
  error_list[4] = error;

  d_error = error_list[4] - error_list[3];   //derivative

  i_error = i_error + error;                 //integral
  int pid = AF_kp*error + AF_kd*d_error + AF_ki*i_error;  

  oled.clearDisplay();
  oled.setTextSize(1);       
  oled.setTextColor(WHITE);
  oled.setCursor(0, 40);
  for(int j=0; j<12; j++){
        oled.print(inputVal[j]);
  }    
  
  
  oled.display();

  int base_speed = 110;  
  int plus_speed = 110;
  int min_speed = 110;

  //base_speed = 80;  
  plus_speed = base_speed + pid;
  min_speed = base_speed - pid - 10;

  

  //Set maimum and minimum values for pid
  if (base_speed + pid > 250){
    plus_speed =250;    
  }

  else if(base_speed - pid > 250){
    min_speed = 250;
  }

  if (base_speed + pid < 50){
    plus_speed = 50;
  }

  else if (base_speed - pid < 50){
    min_speed = 50;  
  }
  
  oled.setTextSize(1);       
  oled.setTextColor(WHITE);
  oled.setCursor(0, 50);     
  oled.print(pid);
  oled.print("    ");
  oled.print(plus_speed);
  oled.print("    ");
  oled.println(min_speed);
  oled.display();
  
  forward(plus_speed,min_speed);
  
}

  ///////////////Inserion Sort//////////////////////////////////////////////
 void insertionSort(float arr[], int n) {
  int i, key, j;
  for (i = 1; i < n; i++) {
    key = arr[i];
    j = i - 1;
    while (j >= 0 && arr[j] > key) {
      arr[j + 1] = arr[j];
      j = j - 1;
    }
    arr[j + 1] = key;
  }
}
 ////////////////Selection sort//////////////////////////
 void selectionSort(float arr[], int n) {
  for (int i = 0; i < n-1; i++) {
    int minIndex = i;
    for (int j = i+1; j < n; j++) {
      if (arr[j] < arr[minIndex]) {
        minIndex = j;
      }
    }
    float temp = arr[i];
    arr[i] = arr[minIndex];
    arr[minIndex] = temp;
  }
}
//////////////////////////////////////////////////////

////////////////Get the distance//////////////////////
float get_distance(int x,int y){
 
    for(int i=0;i<5;i++){
    HCSR04 hc_front(x,y);
    distance1=hc_front.dist();
    ultrasonic_list1[i]=distance1;
    
    }
    for (int j=0;j<5;j++){
    /*Serial.print(ultrasonic_list1[j]);
    Serial.print(" ");*/
   }
  // Serial.println();
    selectionSort(ultrasonic_list1,5);
  // Serial.print("Sorted list:");
    for (int j=0;j<5;j++){ 
    //  Serial.print(ultrasonic_list1[j]);
    // Serial.print(" ");
      
  }

  return ultrasonic_list1[2];
  //Serial.println();
 /* if (m==0){
    if (ultrasonic_list1[2]>=100){
      ultrasonic_list1[2]=100;
      
      }
    
    }
  if (m==1){
    if (ultrasonic_list1[2]>=60){
      ultrasonic_list1[2]=60;
      
      }
    }
    if (m==2){
    if (ultrasonic_list1[2]>=60){
      ultrasonic_list1[2]=60;
      
      }
    }
    if (m==3){
    if (ultrasonic_list1[2]>=80){
      ultrasonic_list1[2]=80;
      
      }
    }
    if (m==4){
    if (ultrasonic_list1[2]>=80){
      ultrasonic_list1[2]=80;
      
      }
    }
    if (m==5){
    if (ultrasonic_list1[2]>=80){
      ultrasonic_list1[2]=80;
      
      }
    }
    if (m==6){
    if (ultrasonic_list1[2]>=80){
      ultrasonic_list1[2]=80;
      
      }
      
    }
    */
   
    
    
  
}


// void ultrasonic_array(){
//   ultrasonic_distance[0]=get_distance(trigPin1,echoPin1);
//   if ((ultrasonic_distance[0]>=100)||(ultrasonic_distance[0]==0)){
//       ultrasonic_distance[0]=100;
    
//     }
//     /*
//   Serial.print("distance 1:");
//   Serial.print(ultrasonic_distance[0]);
//   Serial.println();
//   */
  
//   ultrasonic_distance[1]=get_distance(trigPin2,echoPin2);
//    if ((ultrasonic_distance[1]>=150)||(ultrasonic_distance[1]==0)){
//     ultrasonic_distance[1]=150;
    
//     }
//     /*
//   Serial.print("distance 2:");
//   Serial.print(ultrasonic_distance[1]);
//   Serial.println();
//  */
//   ultrasonic_distance[2]=get_distance(trigPin3,echoPin3);
//    if ((ultrasonic_distance[2]>=150)||(ultrasonic_distance[2]==0)){
//     ultrasonic_distance[2]=150;
    
//     }
//    /* 
//   Serial.print("distance 3:");
//   Serial.print(ultrasonic_distance[2]);
//   Serial.println();
//   */
  
//   ultrasonic_distance[3]=get_distance(trigPin4,echoPin4);
//    if ((ultrasonic_distance[3]>=100)||(ultrasonic_distance[3]==0)){
//     ultrasonic_distance[3]=100;
    
//     }
//     /*
//   Serial.print("distance 4:");
//   Serial.print(ultrasonic_distance[3]);
//   Serial.println();
//   */
//   ultrasonic_distance[4]=get_distance(trigPin5,echoPin5);
//    if ((ultrasonic_distance[4]>=100)||(ultrasonic_distance[4]==0)){
//     ultrasonic_distance[4]=100;
    
//     }
//     /*
//   Serial.print("distance 5:");
//   Serial.print(ultrasonic_distance[4]);
//   Serial.println();
//   */
  
//   ultrasonic_distance[5]=get_distance(trigPin6,echoPin6);
//    if ((ultrasonic_distance[5]>=100)||(ultrasonic_distance[5]==0)){
//     ultrasonic_distance[5]=100;
    
//     }
//     /*
//   Serial.print("distance 6:");
//   Serial.print(ultrasonic_distance[5]);
//   Serial.println();
//   */
  
//   ultrasonic_distance[6]=get_distance(trigPin7,echoPin7);
//    if ((ultrasonic_distance[6]>=100)||(ultrasonic_distance[6]==0)){
//     ultrasonic_distance[6]=100;
    
//     }
//     /*
//   Serial.print("distance 7:");
//   Serial.print(ultrasonic_distance[6]);
//   Serial.println();
//   */
 
  
  
// } 

void ultrasonic_array(){
  ultrasonic_distance[0]=get_distance(trigPin1,echoPin1);
  if ((ultrasonic_distance[0]>=100)||(ultrasonic_distance[0]==0)){
      ultrasonic_distance[0]=100;
    
    }
    /*
  Serial.print("distance 1:");
  Serial.print(ultrasonic_distance[0]);
  Serial.println();
  */
  
  ultrasonic_distance[1]=get_distance(trigPin2,echoPin2);
   if ((ultrasonic_distance[1]>=13)||(ultrasonic_distance[1]==0)){
    ultrasonic_distance[1]=12;
    
    }

    /*
  Serial.print("distance 2:");
  Serial.print(ultrasonic_distance[1]);
  Serial.println();
 */
  ultrasonic_distance[2]=get_distance(trigPin3,echoPin3);
   if ((ultrasonic_distance[2]>=13)||(ultrasonic_distance[2]==0)){
    ultrasonic_distance[2]=12;
    
    }
   /* 
  Serial.print("distance 3:");
  Serial.print(ultrasonic_distance[2]);
  Serial.println();
  */
  
  ultrasonic_distance[3]=get_distance(trigPin4,echoPin4);
   if ((ultrasonic_distance[3]>=5)||(ultrasonic_distance[3]==0)){
    ultrasonic_distance[3]=5;
    
    }
    /*
  Serial.print("distance 4:");
  Serial.print(ultrasonic_distance[3]);
  Serial.println();
  */
  ultrasonic_distance[4]=get_distance(trigPin5,echoPin5);
   if ((ultrasonic_distance[4]>=5)||(ultrasonic_distance[4]==0)){
    ultrasonic_distance[4]=5;
    
    }
    /*
  Serial.print("distance 5:");
  Serial.print(ultrasonic_distance[4]);
  Serial.println();
  */
  
  ultrasonic_distance[5]=get_distance(trigPin6,echoPin6);
   if ((ultrasonic_distance[5]>=50)||(ultrasonic_distance[5]==0)){
    ultrasonic_distance[5]=50;
    
    }
    /*
  Serial.print("distance 6:");
  Serial.print(ultrasonic_distance[5]);
  Serial.println();
  */
  
  ultrasonic_distance[6]=get_distance(trigPin7,echoPin7);
   if ((ultrasonic_distance[6]>=50)||(ultrasonic_distance[6]==0)){
    ultrasonic_distance[6]=50;
    
    }
    /*
  Serial.print("distance 7:");
  Serial.print(ultrasonic_distance[6]);
  Serial.println();
  */
 
  
  
}


//Function for sorting an array
void sort(int a[], int len) {
  for (int i = 0; i < (len - 1); i++) {
    bool flag = true;
    for (int o = 0; o < (len - (i + 1)); o++) {
      if (a[o] > a[o + 1]) {
        int t = a[o];
        a[o] = a[o + 1];
        a[o + 1] = t;
        flag = false;
      }
    }
    if (flag)break;
  }
}
void MZmotorStop(){

  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
  delay(700);

}
void go(int x, int y) {   //right-x left-y
  
  //go forward in a curved path
  analogWrite(enA, x);
  analogWrite(enB, y);
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW) ;
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW) ;

  if(x>y){
    Serial.println("Left");
  }
  else if(x<y){
    Serial.println("Right");
  }
  else{
    Serial.println("Forward");
  }
 delay(120);
}




void MZrunExtraInch(void)
{
  go(140,140);//to move extra length need to change the delay
  go(140,140);//to move extra length need to change the delay
  MZmotorStop();//need to define
  delay(300);
}

void MZgoAndTurn( int Degrees){
  if( Degrees==270){
  analogWrite(enA, 200);
  analogWrite(enB, 200);
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW) ;
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH) ; 
  delay(720);
  }
  if( Degrees==180){
  analogWrite(enA, 200);
  analogWrite(enB, 200);
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH) ;
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW) ;  
  delay(1560);
  }
  if( Degrees==90){
  analogWrite(enA, 200);
  analogWrite(enB, 200);
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH) ;
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW) ; 
  delay(720);
  }
}

void MZmazeTurn (char dir) 
{
       
  if (dir=='L')
  {
      // Turn Left
       MZgoAndTurn (270);
       //lineFollow();      
  }   
    
   else if(dir=='R'){ // Turn Right
       MZgoAndTurn (90);
       //lineFollow();     
   }   
       
   else if(dir== 'B'){ // Turn Back
       MZgoAndTurn (180);
       //lineFollow();     
   }   
       
    else if(dir== 'S'){ // Go Straight
       MZrunExtraInch();
      // lineFollow(); 
    }
}



void MZmazeEnd(void)
{
  MZmotorStop();
  delay(1000);

       oled.setTextSize(1);       
       oled.setTextColor(WHITE);
       oled.setCursor(0, 40);    
       oled.print(6);
       oled.print(MZmode);
       oled.display();
  
  Serial.print("  pathLenght == ");
  Serial.println(MZpathLength);
  //stat = 1;it will trurn back
  delay(1000);
  //mode = "AE";
}

void MZreadLFSsensors()
{
  
  if((inputVal[0]==0)&&(inputVal[1]==0)&&((inputVal[2]==0)&&(inputVal[3]==0)&&(inputVal[4]==0)&&(inputVal[5]==0)&&(inputVal[6]==0)&&(inputVal[7]==0)&&(inputVal[8]==0)&&(inputVal[9]==0)&&(inputVal[10]==0)&&(inputVal[11]==0))){
    MZmode = 'n';//No line
  }
  else if((inputVal[3]==1)&&(inputVal[4]==1)&&(inputVal[5]==1)&&(inputVal[6]==1)&&(inputVal[7]==1)&&(inputVal[8]==1)){
    MZmode = 'c';//cross line
  }
  else if(((inputVal[1]==0)&&(inputVal[10]==1))&&((inputVal[3]==1)||(inputVal[4]==1)||(inputVal[5]==1)||(inputVal[6]==1)||(inputVal[7]==1)||(inputVal[8]==1))){
    MZmode = 'l';//Right line Turn (w r t robot)
  }
  else if(((inputVal[1]==1)&&(inputVal[10]==0))&&((inputVal[3]==1)||(inputVal[4]==1)||(inputVal[5]==1)||(inputVal[6]==1)||(inputVal[7]==1)||(inputVal[8]==1))){
    MZmode = 'r';//Left line Turn (w r t robot)
  }
  else{
    MZmode = 'f';//following line
  }
  
  /*if     ((LFSensor[0]== 1 )&&(LFSensor[1]== 1 )&&(LFSensor[2]== 1 )&&(LFSensor[3]== 1 )&&(LFSensor[4]== 1 )&&(LFSensor[5]== 1 )&&(LFSensor[6]== 1 ))  {mode = 'n'; }//n= no line
  else if((LFSensor[0]== 0)&&(LFSensor[1]== 0 )&&(LFSensor[5]== 1 )&&(LFSensor[6]== 1 )) {mode = 'l';}//left turn
  else if((LFSensor[0]== 1)&&(LFSensor[1]== 1 )&&(LFSensor[5]== 0 )&&(LFSensor[6]== 0 )) {mode = 'r';}//right turn
  else if((LFSensor[0]== 0 )&&(LFSensor[1]== 0 )&&(LFSensor[2]== 0 )&&(LFSensor[3]== 0 )&&(LFSensor[4]== 0 )&&(LFSensor[5]== 0 )&&(LFSensor[6]== 0 ))  {mode = 'c'; }// cont line
 // else if((LFSensor[0]== 1 )&&(LFSensor[1]== 1 )&&(LFSensor[2]== 0 )&&(LFSensor[3]== 0 )&&(LFSensor[4]== 0 ))  {mode = 'l'; }//after the new 2 sensor used we dont want to use this
 // else if((LFSensor[0]== 0 )&&(LFSensor[1]== 0 )&&(LFSensor[2]== 0 )&&(LFSensor[3]== 1 )&&(LFSensor[4]== 1 ))  {mode = 'r'; }
  else {mode = 'f';}//following line*/
    
}
///////////////////////////////////////////////////////////////////////////////////// from gridMaze ///////////////////////
void readLineSensors(){
  int tempInputValue_0[3] = {10,10,10};
  int tempInputValue_1[3] = {10,10,10};
  int tempInputValue_2[3] = {10,10,10};
  int tempInputValue_3[3] = {10,10,10};
  int tempInputValue_4[3] = {10,10,10};
  int tempInputValue_5[3] = {10,10,10};
  int tempInputValue_6[3] = {10,10,10};
  int tempInputValue_7[3] = {10,10,10};
  int tempInputValue_8[3] = {10,10,10};
  int tempInputValue_9[3] = {10,10,10};
  int tempInputValue_10[3] = {10,10,10};
  int tempInputValue_11[3] = {10,10,10};

  for (int i = 0 ; i < 3 ; i++) {
    tempInputValue_0[i] = analogRead(ProxSensor_1);
    tempInputValue_1[i] = analogRead(ProxSensor_2);
    tempInputValue_2[i] = analogRead(ProxSensor_3);
    tempInputValue_3[i] = analogRead(ProxSensor_4);
    tempInputValue_4[i] = analogRead(ProxSensor_5);
    tempInputValue_5[i] = analogRead(ProxSensor_6);
    tempInputValue_6[i] = analogRead(ProxSensor_7);
    tempInputValue_7[i] = analogRead(ProxSensor_8);
    tempInputValue_8[i] = analogRead(ProxSensor_9);
    tempInputValue_9[i] = analogRead(ProxSensor_10);
    tempInputValue_10[i] = analogRead(ProxSensor_11);
    tempInputValue_11[i] = analogRead(ProxSensor_12);
  }
   sort(tempInputValue_0,3);
   sort(tempInputValue_1,3);
   sort(tempInputValue_2,3);
   sort(tempInputValue_3,3);
   sort(tempInputValue_4,3);
   sort(tempInputValue_5,3);
   sort(tempInputValue_6,3);
   sort(tempInputValue_7,3);
   sort(tempInputValue_8,3);
   sort(tempInputValue_9,3);
   sort(tempInputValue_10,3);
   sort(tempInputValue_11,3);

  inputVal[0] = tempInputValue_0[2];
  inputVal[1] = tempInputValue_1[2];
  inputVal[2] = tempInputValue_2[2];
  inputVal[3] = tempInputValue_3[2];
  inputVal[4] = tempInputValue_4[2];
  inputVal[5] = tempInputValue_5[2];
  inputVal[6] = tempInputValue_6[2];
  inputVal[7] = tempInputValue_7[2];
  inputVal[8] = tempInputValue_8[2];
  inputVal[9] = tempInputValue_9[2];
  inputVal[10] = tempInputValue_10[2];
  inputVal[11] = tempInputValue_11[2];

  for (int i = 0; i<12; i++){  //Convert analog inputs to digital
       if(inputVal[i]<200){
          inputVal[i] = 1;            
        }
        else{
            inputVal[i] = 0;
            }
           Serial.print(inputVal[i]);
           Serial.print(" ");
  
          }
          

   for (int i = 0; i<3; i++){      // for 3 sensor regions
                  lineSensorCount[i] = 0; //initial Value
                  for(int j = 0; j<4;  j++){
                      lineSensorCount[i] += inputVal[4*i+j];
                  }
            }  


   ///////////////////////////////////////////////////////////////////////////////////////////
  oled.clearDisplay(); // clear display
  oled.setTextSize(1);          // text size
  oled.setTextColor(WHITE);     // text color
  oled.setCursor(0, 50);        // position to display
  //oled.println(inputVal[6]); // text to display
  oled.println("Read line sensors");
  oled.display();               // show on OLED
  //////////////////////////////////////////////////////////////////////////////////////////
}


void MZsettleLine(){
  int MazeError=1;
  for( int k=0;k<=1;k++){
    go(140,140);
    go(140,140);
    go(140,140);
  while(!(MazeError==0)){
  readLineSensors();

  
   ///////////////////////////////////////////////////////////////////////////////////////////
  oled.clearDisplay(); // clear display
  oled.setTextSize(1);          // text size
  oled.setTextColor(WHITE);     // text color
  oled.setCursor(0, 50);        // position to display
  // text to display
  oled.println("Settle line");
  oled.display();               // show on OLED
  //////////////////////////////////////////////////////////////////////////////////////////

  MazeError = -200*inputVal[0]-100*inputVal[1]-50*inputVal[2]-10*inputVal[3]-5*inputVal[4]+5*inputVal[7]+10*inputVal[8]+50*inputVal[9]+100*inputVal[10]+200*inputVal[11];
   
    if (MazeError>50){
        turnLeft(160);
        //delay(110);
        //go(80,220);
        //go(70,220);
        //motorStop();
        //delay(180);
  }
    else if (MazeError<-50){
      turnRight(160);
      //delay(110);
      //go(220,80);
      //go(220,70);
      //motorStop();
      //delay(180);
      
  }
   else{
     //Stop();
     //printToOled(10,"Settled");
     MZmotorStop();
     break;
         
   }
  }
  }
}
/////////////////////////////////////////////////////////////////////////////////////from gridMaze///////////////////////

void MZrecIntersection(char Direction)
{
  MZpath[MZpathLength] = Direction; // Store the intersection in the path variable.
  MZpathLength ++;
  //  simplifyPath(); // Simplify the learned path.
            oled.setTextSize(1);       
            oled.setTextColor(WHITE);
            oled.setCursor(0, 50);    
            oled.print(5);
            oled.print(MZmode);
            oled.display();
}

void MZlinefollow(){ 

  //floorPattern();
  readLineSensors();
  
  error = 10*( inputVal[3]*6 +inputVal[4]*4 + inputVal[5]*2 - ( + inputVal[8]*6 +inputVal[7]*4 + inputVal[6]*2) ) ; //(2*(inputVal[0]+inputVal[1]+inputVal[2]+inputVal[3]+inputVal[4]+inputVal[5]+inputVal[6]));

  for (int i = 0; i<4 ; i++){
    error_list[i] = error_list[i+1];
  }

  error_list[4] = error;
  d_error = error_list[4] - error_list[3];

  i_error = i_error + error;
  int pid = kpS*error + kdS*d_error + kiS*i_error;  

  oled.clearDisplay();
  oled.setTextSize(1);       
  oled.setTextColor(WHITE);
  oled.setCursor(0, 40);
  for(int j=0; j<12; j++){
        oled.print(inputVal[j]);
  }    
  
  
  oled.display();

  int base_speed = 80;  
  int plus_speed = 80;
  int min_speed = 80;

  //base_speed = 80;  
  //plus_speed = base_speed + pid;
 // min_speed = base_speed - pid;


  base_speed = 130; 
  plus_speed = base_speed + pid;
  min_speed = base_speed - pid;

  if (base_speed + pid > 220){
    plus_speed =220;    
  } 

  else if(base_speed - pid > 220){
    min_speed = 220;
  } 

  if (base_speed + pid < 90){
    plus_speed = 90;
  }

  else if (base_speed - pid < 90){
    min_speed =90;  
  } 
  

  oled.setTextSize(1);       
  oled.setTextColor(WHITE);
  oled.setCursor(0, 50);     
  oled.print(pid);
  oled.print("    ");
  oled.print(plus_speed);
  oled.print("    ");
  oled.println(min_speed);
  oled.display();
  forward(plus_speed,min_speed);
    
}

void MZmazeSolve(void)
{           oled.setTextSize(1);       
            oled.setTextColor(WHITE);
            oled.setCursor(0, 40);    
            oled.print(1);
            oled.print("maze solve");
            oled.display(); 
      
       if (MZmode=='n')
        {   

            MZmotorStop();
            delay(500);
            MZgoAndTurn (180);
            MZmotorStop();
            delay(500);
            MZsettleLine();
            MZrecIntersection('B');
        }
            
          
         else if(MZmode== 'c'){          
           MZmotorStop();
            delay(500);
            
///////////////////////////////////////////////////////////////////////////////////////////
  oled.clearDisplay(); // clear display
  oled.setTextSize(1);          // text size
  oled.setTextColor(WHITE);     // text color
  oled.setCursor(0, 50);        // position to display
  // text to display
  oled.print(" maze solwing ");
  oled.println(MZmode);
  oled.display();               // show on OLED
//////////////////////////////////////////////////////////////////////////////////////////
            go(140,160);
            go(140,160);
            go(140,160);
            go(140,160);
            
            MZmotorStop();
            delay(250);
           readLineSensors();
           MZreadLFSsensors();
           if((MZmode=='n')||(MZmode=='f'))
           {MZgoAndTurn (270);MZmotorStop();delay(200);MZsettleLine(); MZrecIntersection('L');} // or it is a "T" or "Cross"). In both cases, goes to LEFT
           
           MZfloorColor = "NO COLOR CHECKING ";
           if (MZmode == 'c'){
              go(140,170);
              go(140,170);
              //go(140,170);
              //go(140,170);
              MZmotorStop();
              delay(2000);
              MZfloorColor = FindColorFloor();
              printToOled(10,MZfloorColor);
              MZmotorStop();
              delay(1000);
              //go(140,140);
              
           }


          if(MZfloorColor=="Colour Blue"){
            MZgoAndTurn (180);
            MZmotorStop();
            delay(200);
            MZsettleLine();
            MZrecIntersection('B');
            
           }
           else if(MZfloorColor=="Colour Green"){
            MZgoAndTurn (180);
            MZmotorStop();
            delay(200);
            MZsettleLine();
            MZrecIntersection('B');
            
           }

           else if(MZfloorColor=="Colour Red"){//end of the Maze
            
            MZmotorStop();
            delay(3000);
            turnLeft(200);
            delay(500);
            Stop();
            forward(150,130);
            delay(200);
            settleLine();
            for(int i =0; i<4;i++){
              pidLineFollower();
            }
            settleLine();
            for(int i =0; i<4;i++){
              pidLineFollower();
            }
            settleLine();
            
            /////////////////////////////////////////////// NEXT CODE WILL CALL ///////////////////
            HelpingStage = 15;
            //settleLine();
            //recIntersection('B');
            
           }
            else{
                             
            MZgoAndTurn (180);
            MZmotorStop();
            delay(200);
            MZsettleLine();
            MZrecIntersection('B');
            go(140,170);
            go(140,170);
            delay(1000);
            go(140,170);
            go(140,170);
           }           
           
           }
         
            
         else if(MZmode== 'r'){

 

             MZmotorStop();
             delay(500);
             //go(140,140);
             go(140,160);
             go(140,160);
             go(140,160);
            
             readLineSensors();
             MZreadLFSsensors();
           
            if (MZmode == 'n') {MZgoAndTurn (90);MZmotorStop();delay(500);MZsettleLine(); MZrecIntersection('R');}
            else {
            go(130,150);
            go(140,160);
            go(140,160);  
            MZrecIntersection('S');
            }
         }  
            
          else if( MZmode=='l'){
            MZmotorStop();
            delay(500);
            go(140,160);
            go(140,160);
            go(140,160);
            MZgoAndTurn (270);
            //motorStop();
            //delay(500);
            MZsettleLine();
            //lineFollow(); 
            MZrecIntersection('L');
          }   
         
         else if (MZmode== 'f'){
            MZlinefollow();
                
        
         }
         
}
//////////////////////////////////////////////////////////////// COLOR SENSOR /////////////////////////////////
////////////////////////////////////fun 1 ///////////////////

String FindColorBox(){
  int r, g, b;
  String color = " ";
  r = process_red_value_Box();
  delay(200);
  g = process_green_value_Box();
  delay(200);
  b = process_blue_value_Box();
  delay(200);
  Serial.print("r = ");
  Serial.print(r);
  Serial.print(" ");
  Serial.print("g = ");
  Serial.print(g);
  Serial.print(" ");
  Serial.print("b = ");
  Serial.print(b);
  Serial.print(" ");
  Serial.println();

    //////////////////////////////////////////////
    oled.clearDisplay();
  oled.setTextSize(1);      
  oled.setTextColor(WHITE); 
  //oled.println("Team Spectro");
  oled.setCursor(0, 30);    
  oled.print(r);
  oled.print(" ");
   oled.print(g);
    oled.print(" ");
    oled.print(b);
     oled.println(" "); 
  oled.display();
  ////////////////////////////////////////////
    if(((r>75)&&(r<90))&&((g >65)&&(g<86))&&((b<75)&&(b>55)))
  {
    Serial.println("Green");
    color = "Green";
  }
    else if (((r>70)&&(r<95))&&((g >68)&&(g<95))&&((b<65)&&(b>40)))
  {
    Serial.println("Blue");
    color = "Blue";
  }



    else if (((r>50)&&(r<75))&&((g >65)&&(g<100))&&((b<75)&&(b>48)))
  {
    Serial.println("Red");
    color = "Red";
  }
  else 
  {
    Serial.println("B Colour Red");
    color = "Red";
  }
 
  return(color);
}



////////////////////////////////////fun 2 ///////////////////
int process_red_value_Box()
{
  digitalWrite(S2_PIN_box, LOW);
  digitalWrite(S3_PIN_box, LOW);
  int pulse_length = pulseIn(OUT_PIN_box, LOW);
  return pulse_length;
}

////////////////////////////////////fun 3 ///////////////////
int process_green_value_Box()
{
  digitalWrite(S2_PIN_box, HIGH);
  digitalWrite(S3_PIN_box, HIGH);
  int pulse_length = pulseIn(OUT_PIN_box, LOW);
  return pulse_length;
}
////////////////////////////////////fun 4 ///////////////////
int process_blue_value_Box()
{
  digitalWrite(S2_PIN_box, LOW);
  digitalWrite(S3_PIN_box, HIGH);
  int pulse_length = pulseIn(OUT_PIN_box, LOW);
  return pulse_length;
}
//////////////////////////////////////////////////////// floor color //////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////
/*
String FindColorFloor(){
  int r, g, b;
  String color = " ";
  r = process_red_value_Floor();
  delay(200);
  g = process_green_value_Floor();
  delay(200);
  b = process_blue_value_Floor();
  delay(200);
  Serial.print("r = ");
  Serial.print(r);
  Serial.print(" ");
  Serial.print("g = ");
  Serial.print(g);
  Serial.print(" ");
  Serial.print("b = ");
  Serial.print(b);
  Serial.print(" ");
  Serial.println();
  //////////////////////////////////////////////
  oled.clearDisplay();
  oled.setTextSize(1);      
  oled.setTextColor(WHITE); 
  //oled.println("Team Spectro");
  oled.setCursor(0, 30);    
  oled.print(r);
  oled.print(" ");
  oled.print(g);
  oled.print(" ");
  oled.print(b);
  oled.println(" "); 
  oled.display();
  ////////////////////////////////////////////
  if((r<400)&&(g<400)&&(b<400)){
    color = "Colour white";
  }
  else if ((r>1500)&&(g>1500)&&(b>1500))
  {
    Serial.println("F Colour Black");
    color = "Colour Black";
  }
  
  else if (((r>1390)&&(r<1700))&&((g >880)&&(g<1150))&&((b<650)&&(b>400)))
  {
    Serial.println("F Colo;ur Blue");
    color = "Colour Blue";
  }

  else if (((r>400)&&(r<600))&&((g>1350)&&(g<1550)))
  {
    Serial.println("F Colour Red");
    color = "Colour Red";
  }
    else if (((r>1150)&&(r<1400))&&((g>700)&&(g<850))&&((b>900)&&(b<1020)))
  {
    Serial.println("F Colour Green");
    color = "Colour Green";
  }
  else 
  {
    Serial.println("F Colour Green");
    color = "Colour Blue";
  }

  return(color);

}
////////////////////////////////////fun 2 ///////////////////
int process_red_value_Floor()
{
  digitalWrite(S2_PIN_floor, LOW);
  digitalWrite(S3_PIN_floor, LOW);
  int pulse_length = pulseIn(OUT_PIN_floor, LOW);
  return pulse_length;
}

////////////////////////////////////fun 3 ///////////////////
int process_green_value_Floor()
{
  digitalWrite(S2_PIN_floor, HIGH);
  digitalWrite(S3_PIN_floor, HIGH);
  int pulse_length = pulseIn(OUT_PIN_floor, LOW);
  return pulse_length;
}
////////////////////////////////////fun 4 ///////////////////
int process_blue_value_Floor()
{
  digitalWrite(S2_PIN_floor, LOW);
  digitalWrite(S3_PIN_floor, HIGH);
  int pulse_length = pulseIn(OUT_PIN_floor, LOW);
  return pulse_length;
}
 */
//}

String FindColorFloor(){
  String floorColor = " ";
  color(); 
  ////Serial.print("R Intensity:");  
  ////Serial.print(red, DEC);  
  //Serial.print(" G Intensity: ");  
  //Serial.print(green, DEC);  
  //Serial.print(" B Intensity : ");  
  //Serial.print(blue, DEC);  
  ////Serial.println();  
 
  if (red < blue && red < green && red < 530)
  {  
   //Serial.println(" - (Red Color)");
   floorColor = "Colour Red"; 
   //delay(500);
 
  }  
 
  else if (blue < red  && (red-green)>200)   
  {  
   //Serial.println(" - (Blue Color)"); 
   floorColor = "Colour Blue"; 
   //delay(500);

  }  
 
  // else if (green < red && green < blue)  
  // {  
  //  //Serial.println(" - (Green Color)"); 
  //  floorColor = "Colour Green"; 
  //  //delay(500); 
  // }  
  else{
  //Serial.println(" not detected");
  floorColor = "Colour Green";
  //delay (500); 
  }   
  return floorColor;
}

void color()  
{    
  digitalWrite(S2_PIN_floor, LOW);  
  digitalWrite(S3_PIN_floor, LOW);  
  red = pulseIn(OUT_PIN_floor, digitalRead(OUT_PIN_floor) == HIGH ? LOW : HIGH);  
  digitalWrite(S3_PIN_floor, HIGH);  
  //count OUT, pBLUE, BLUE  
  blue = pulseIn(OUT_PIN_floor, digitalRead(OUT_PIN_floor) == HIGH ? LOW : HIGH);  
  digitalWrite(S0_PIN_floor, HIGH);  
  //count OUT, pGreen, GREEN  
  green = pulseIn(OUT_PIN_floor, digitalRead(OUT_PIN_floor) == HIGH ? LOW : HIGH);  
}


////////////////////////////////////////////////////////////////// COLOR SENSOR ENDS ///////////////////////////////////////

/////////////////////////////////////////////////////////////////  BASIC FUNCTIONS   ///////////////////////////////////////

void reverse(int lspeed,int rspeed){
  analogWrite(enA,lspeed);
  analogWrite(enB,rspeed);
  digitalWrite(in1,LOW);
  digitalWrite(in2,HIGH);
  digitalWrite(in3,LOW);
  digitalWrite(in4,HIGH);
  
}

void forward(int lSpeed,int rSpeed){
  analogWrite(enA, lSpeed);
  analogWrite(enB, rSpeed);
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW) ;
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW) ;
}

void turnLeft(int turningSpeed){
  analogWrite(enA, turningSpeed);
  analogWrite(enB, turningSpeed);
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW) ;
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH) ;   
}

void turnRight(int turningSpeed){
  analogWrite(enA, turningSpeed);
  analogWrite(enB, turningSpeed);
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH) ;
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW) ; 
  
}


void Stop(){
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW) ;
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW) ;  
}

void rightTurn90(){
  turnRight(200);
  delay(500);
  settleLine();   //#
  Stop();
}

void leftTurn90(){
  turnLeft(200);
  delay(500);
  settleLine();  //#
  Stop();
}

void leftTurn180(){
  turnLeft(200);
  delay(1200);
  Stop();
}


void pidLineFollower(){
  //floorPattern();
  readLineSensors();
  //Error function
  error = 10*(inputVal[0]*6 +inputVal[1]*5 +inputVal[2]*4 + inputVal[3]*3 +inputVal[4]*2 + inputVal[5] -(inputVal[11]*6 +inputVal[10]*5 +inputVal[9]*4 + inputVal[8]*3 +inputVal[7]*2 + inputVal[6]) ) ; //(2*(inputVal[0]+inputVal[1]+inputVal[2]+inputVal[3]+inputVal[4]+inputVal[5]+inputVal[6]));

  for (int i = 0; i<4 ; i++){   //Shift and store the error
    error_list[i] = error_list[i+1];
  }
  error_list[4] = error;

  d_error = error_list[4] - error_list[3];   //derivative

  i_error = i_error + error;                 //integral
  int pid = kp*error + kd*d_error + ki*i_error;  

  oled.clearDisplay();
  oled.setTextSize(1);       
  oled.setTextColor(WHITE);
  oled.setCursor(0, 40);
  for(int j=0; j<12; j++){
        oled.print(inputVal[j]);
  }    
  
  
  oled.display();

  int base_speed = 130;  
  int plus_speed = 80;
  int min_speed = 80;

  //base_speed = 80;  
  plus_speed = base_speed + pid;
  min_speed = base_speed - pid;

  //Check for Higher pid values to get to the line
  if(pid>100){
    turnRight(200);   //200
    delay(100);      //250
  }
  else if(pid<-100){
    turnLeft(200);    //200
    delay(100);      //250
  } 
  else{
  base_speed = 120;  
  plus_speed = base_speed + pid;
  min_speed = base_speed - pid;

  //Set maimum and minimum values for pid
  if (base_speed + pid > 250){
    plus_speed =250;    
  }

  else if(base_speed - pid > 250){
    min_speed = 250;
  }

  if (base_speed + pid < 50){
    plus_speed = 50;
  }

  else if (base_speed - pid < 50){
    min_speed =50;  
  }
  } 
  oled.setTextSize(1);       
  oled.setTextColor(WHITE);
  oled.setCursor(0, 50);     
  oled.print(pid);
  oled.print("    ");
  oled.print(plus_speed);
  oled.print("    ");
  oled.println(min_speed);
  oled.display();
  forward(plus_speed,min_speed);
  
}

void pidStraightLineFollower(){
  //floorPattern();
  readLineSensors();
  //2 3 6
  error = 10*( inputVal[3]*6 +inputVal[4]*4 + inputVal[5]*2 - ( + inputVal[8]*6 +inputVal[7]*4 + inputVal[6]*2) ) ; //(2*(inputVal[0]+inputVal[1]+inputVal[2]+inputVal[3]+inputVal[4]+inputVal[5]+inputVal[6]));

  for (int i = 0; i<4 ; i++){
    error_list[i] = error_list[i+1];
  }

  error_list[4] = error;
  d_error = error_list[4] - error_list[3];

  i_error = i_error + error;
  int pid = kpS*error + kdS*d_error + kiS*i_error;  

  oled.clearDisplay();
  oled.setTextSize(1);       
  oled.setTextColor(WHITE);
  oled.setCursor(0, 40);
  for(int j=0; j<12; j++){
        oled.print(inputVal[j]);
  }    
  
  
  oled.display();

  int base_speed = 80;  
  int plus_speed = 80;
  int min_speed = 80;

  //base_speed = 80;  
  //plus_speed = base_speed + pid;
 // min_speed = base_speed - pid;


  base_speed = 130; 
  plus_speed = base_speed + pid;
  min_speed = base_speed - pid;

  if (base_speed + pid > 250){
    plus_speed =250;    
  } 

  else if(base_speed - pid > 250){
    min_speed = 250;
  } 

  if (base_speed + pid < 50){
    plus_speed = 50;
  }

  else if (base_speed - pid < 50){
    min_speed =50;  
  } 
  

  oled.setTextSize(1);       
  oled.setTextColor(WHITE);
  oled.setCursor(0, 50);     
  oled.print(pid);
  oled.print("    ");
  oled.print(plus_speed);
  oled.print("    ");
  oled.println(min_speed);
  oled.display();
  forward(plus_speed,min_speed);
  
      


}



/////////////////////////// th functions /////////////////////////


void settleLine(){
  int MazeError=1;
  while(!(MazeError==0)){
  readLineSensors();
  // if ((inputVal[6]==0)&&(inputVal[7]==0)){
  //     MazeError=0;
  //     break;
  // }
  MazeError = -200*inputVal[0]-100*inputVal[1]-50*inputVal[2]-10*inputVal[3]-5*inputVal[4]+5*inputVal[7]+10*inputVal[8]+50*inputVal[9]+100*inputVal[10]+200*inputVal[11];
  
  // if eror is +, take as robot has turn left(rigt sensors on the line)
  // considerd when line goes toward the right the eror as positive (robot going out from the line to left)
  // for positive error robot need to turn right  
  // for negative error robot need to turn left
    
  //if (!((inputVal[6]==0)&&(inputVal[7]==0))){
    //make adjusments to settle on the line
  printToOled(20,String(MazeError));    
    if (MazeError>10){
      turnLeft(160); 
  }
    else if (MazeError<-10){
      turnRight(160); 
  }
   else{
     Stop();
     printToOled(10,"Settled");
     break;
         
   }
  }
}

 /*
void readLineSensors(){
            
                          
            inputVal[0]  = analogRead(ProxSensor_1);
            inputVal[1]  = analogRead(ProxSensor_2);
            inputVal[2]  = analogRead(ProxSensor_3);
            inputVal[3]  = analogRead(ProxSensor_4);
            inputVal[4]  = analogRead(ProxSensor_5);
            inputVal[5]  = analogRead(ProxSensor_6);
            inputVal[6]  = analogRead(ProxSensor_7);
            inputVal[7]  = analogRead(ProxSensor_8);
            inputVal[8]  = analogRead(ProxSensor_9);
            inputVal[9]  = analogRead(ProxSensor_10);
            inputVal[10]  = analogRead(ProxSensor_11);
            inputVal[11]  = analogRead(ProxSensor_12);

            for (int i = 0; i<12; i++){  //Convert analog inputs to digital
                    if(inputVal[i]<500){   //200
                          inputVal[i] = 1;            
                    }
                    else{
                          inputVal[i] = 0;
                    }
                    Serial.print(inputVal[i]);
                    Serial.print(" ");

            }
            Serial.println();

            

            // shift values
            for (int i = 0; i < NUM_SENSORS; i++) {
                  for (int j = NUM_VALUES - 1; j > 0; j--) {
                        sensorValues[i][j] = sensorValues[i][j-1];
              }
            }            
            for (int i = 0; i < NUM_SENSORS; i++) {
                        //sensorValues[i][0] = analogRead(i);
                        sensorValues[i][0] = inputVal[i];
              }

            for (int i = 0; i<3; i++){      // for 3 sensor regions
                  lineSensorCount[i] = 0; //initial Value
                  for(int j = 0; j<4;  j++){
                      lineSensorCount[i] += inputVal[4*i+j];
                  }
            }  
}   */

void floorPattern(){
            //delay(500);
            //Serial.println("-------------------------------------------");
            //Serial.print("pattern:  ");
            Serial.print(linePathPattern);
            Serial.print("  ");
            //Serial.print("Junction detected:  ");
            Serial.println(junctionDetected);
            //Serial.println("--------------------------------------------");

            readLineSensors();

            //Detect patterns
            if(not pathCrossing){
                      printToOled(30, linePathPattern);
                      if((lineSensorCount[0] >2) and (lineSensorCount[1] >2) and (lineSensorCount[2] >2)){
                              linePathPattern = "WhiteLine";
                              //forward(50,50);
                              pathCrossing = true;                              
                      }
                     /* else if((lineSensorCount[0] <2 ) and (lineSensorCount[1] >=1) and (lineSensorCount[2] < 2 )){
                              linePathPattern = "Line";
                              //pathCrossing = true;
                      } */
                      else if((lineSensorCount[0] >1 ) and (lineSensorCount[1] >=1) and (lineSensorCount[2] == 0 )){
                              linePathPattern = "HardRightTurn";
                              //forward(50,50);
                              pathCrossing = true;
                      }
                      else if((lineSensorCount[0] == 0 ) and (lineSensorCount[1] >=1) and (lineSensorCount[2] >1 )){
                      //else{
                              linePathPattern = "HardLeftTurn";
                              //forward(50,50);
                              pathCrossing = true;
                      }

                      else if((lineSensorCount[0] <2 ) and (lineSensorCount[1] >=2) and (lineSensorCount[2] < 2 )){
                              linePathPattern = "Line";
                              //pathCrossing = true;
                      }

                      else {
                              linePathPattern = "Not detected";
                              //pathCrossing = true;
                      }

            }
            else{
                      pathCrossing = false;
                      forward(150,135);  //Change this if needed
                      delay(250);
                      Stop();
                
                      //forward(50,50);
                      String prePathPattern = linePathPattern;
                      //delay(100); // Change this
                      readLineSensors();
                      /*for(int j =0; j<4 ; j++){
                              readLineSensors();
                      } */
                      
                      if((lineSensorCount[0] >2) and (lineSensorCount[1] >3) and (lineSensorCount[2] >2)){
                              linePathPattern = "WhiteLine";
                              //pathCrossing = true;                              
                      }
                      else if((lineSensorCount[0] <2 ) and (lineSensorCount[1] >=1) and (lineSensorCount[2] < 2 )){
                              linePathPattern = "Line";
                              //junctionDetected = true;
                              //pathCrossing = true;
                      }
                      else if((lineSensorCount[0] <1 ) and (lineSensorCount[1] <1) and (lineSensorCount[2] < 1 )){
                              linePathPattern = "BlackLine";
                              //junctionDetected = true;
                              //pathCrossing = true;
                      }
                      
                      

                      if((prePathPattern=="WhiteLine") and (linePathPattern=="WhiteLine")){
                              linePathPattern = "WhiteBox";
                      }
                      else if((prePathPattern=="WhiteLine") and (linePathPattern=="Line")){
                              linePathPattern = "Junction";
                              junctionDetected = true;
                      }
                      else if((prePathPattern=="WhiteLine") and (linePathPattern=="BlackLine")){
                              linePathPattern = "Junction";
                              junctionDetected = true;
                      }
                      else if((prePathPattern=="HardRightTurn") and (linePathPattern=="Line")){
                              linePathPattern = "Junction";
                              junctionDetected = true;
                      }
                      else if((prePathPattern=="HardLeftTurn") and (linePathPattern=="Line")){
                              linePathPattern = "Junction";
                              junctionDetected = true;
                      }
                      //Only for tower of hanoi
                      else if((prePathPattern=="HardLeftTurn") and (linePathPattern=="BlackLine")){
                              linePathPattern = "Junction";
                              junctionDetected = true;
                      }
                      else if((prePathPattern=="HardRightTurn") and (linePathPattern=="BlackLine")){
                              linePathPattern = "Junction";
                              junctionDetected = true;
                      }

                      

            }
}

void thRightShift_90(){   //Right shift Junction mapping
          //copy original list to currrent junction list
          for(int j =0; j<4; j++){
            thCurrentJunc[j] = thJunc[thCurrentJuncIndex][j];
          }

          int temp = thCurrentJunc[3];  //last element of current junction
          for (int i =3; i>0 ; i--){
            thCurrentJunc[i] = thCurrentJunc[i-1];
          }
          thCurrentJunc[0] =  temp;
}

void thLeftShift_90(){   //Left shift Junction mapping
          //copy original list to currrent junction list
          for(int j =0; j<4; j++){
            thCurrentJunc[j] = thJunc[thCurrentJuncIndex][j];
          }

          int temp = thCurrentJunc[0];  //first element of current junction
          for (int i =0; i<3 ; i++){
            thCurrentJunc[i] = thCurrentJunc[i+1];
          }
          thCurrentJunc[3]= temp;
}

void thShift_180(){
          //copy original list to currrent junction list
          for(int j =0; j<4; j++){
            thCurrentJunc[j] = thJunc[thCurrentJuncIndex][j];
          }

          int temp = thCurrentJunc[3];  //last element of current junction
          for (int i =3; i>0 ; i--){
            thCurrentJunc[i] = thCurrentJunc[i-1];
          }
          thCurrentJunc[0] =  temp;

          temp = thCurrentJunc[3];  //last element of current junction
          for (int i =3; i>0 ; i--){
            thCurrentJunc[i] = thCurrentJunc[i-1];
          }
          thCurrentJunc[0] =  temp;



}

void thAutomaticRouting(int num){      //Take the decision at junctions
                  thDestination = num;
                  //Update floor mappings with respect to directions
                  // num is the location to reach
                  // Write speeds to motors to rotate
                  //delay
                  //thPathFinder()

                  /* /////////////////////////////////////////////////////
                  for(int i =0; i<4; i++){
                    thCurrentJunc[i] = thJunc[thCurrentJuncIndex][i];   //Get the current junction from the list
                  }
                  //Shift according to the direction
                  ////////////////////////////////////////////////////////// */

                  thUpdateJunction();

                  Serial.println(thArraySearch(num, thCurrentJunc));
                  if(thArraySearch(num,thCurrentJunc) != -1){  //Enter only if num available in the junction data
                    //Destination is in the junction data
                    thCurrentLocation = num;
                    int mainNum = thArraySearch(num, thCurrentJunc);
                    if(mainNum == 0){
                      //turnLeft
                      Serial.println("Turn Left");
                      printToOled(10,"Left");
                      leftTurn90();
                      settleLine();   //#
                      //delay(1000);
                    }
                    else if(mainNum == 1){  //not useful
                      //forward
                      Serial.println("Forward");
                      printToOled(10,"going forward");
                      //forward(150,150);
                      
                      //delay(1000);

                    //Update this function
                    }

                    else if(mainNum == 2){  
                      //turnRight
                      Serial.println("Turn Right");
                      printToOled(10,"Right");
                      rightTurn90();
                      settleLine();  //#
                    //Update this function
                    }
                    else{
                      //turnleft by 180
                      Serial.println("Turn Back");
                      printToOled(10,"back");
                      leftTurn180();
                      settleLine();  //#
                    //Update this function
                      
                      //delay(1000);
                    }
                      
                  }          
                  else{
                      int thSubNum = 0;     //If junction does not contain the searching number
                      if(num>12){
                        thSubNum = num/4;    //Get the number of middle point
                      }
                      else{
                        thSubNum = num*4;
                      }
                      int mainNum = thArraySearch(thSubNum, thCurrentJunc);
                      if(mainNum != -1){
                            //Sub-Destination is in the junction data
                            thCurrentLocation = thSubNum;
                            if(mainNum == 0){
                              Serial.println("Turn Left");
                              printToOled(10,"Left");
                              leftTurn90();
                              settleLine();  //#
                              //thCurrentJuncIndex++;
                              thIncrementJunc();
                              //delay(1000);
                              //turnLeft
                            }
                            else if(mainNum == 1){
                            //forward
                            Serial.println("Forward");
                            printToOled(10,"Forward");
                            forward(150,150);
                            settleLine(); //#
                            //thCurrentJuncIndex++;    //Change this (update junction)
                            thIncrementJunc();
                            //delay(1000);
                            }
                            else{
                            //turnRight
                            Serial.println("Turn Right");
                            printToOled(10,"Right");
                            rightTurn90();
                            settleLine();  //#
                            //thCurrentJuncIndex++;
                            thIncrementJunc();
                            //delay(1000);
                          }
                      }
                      else{
                      int mainNum = thArraySearch(8, thCurrentJunc);  // 8 is definitely in the array
                      if(mainNum != -1){
                            //Destination is in the junction data
                            thCurrentLocation = 8;
                            if(mainNum == 0){
                              Serial.println("Turn Left");
                              printToOled(10,"Left");
                              leftTurn90();
                              settleLine();
                              //thCurrentJuncIndex++;
                              thIncrementJunc();
                              //delay(1000);
                              //turnLeft
                            }
                            else if(mainNum == 1){
                            //forward
                            Serial.println("Forward");
                            printToOled(10,"Forward");
                            forward(150,150);
                            settleLine();
                            //thCurrentJuncIndex++;    //Change this (update junction)
                            thIncrementJunc();
                            //delay(1000);
                            }
                            else{
                            //turnRight
                            Serial.println("Turn Right");
                            printToOled(10,"Right");
                            rightTurn90();
                            settleLine();
                            //thCurrentJuncIndex++;
                            thIncrementJunc();
                            //delay(1000);
                          }
                      }

                      }
                  }

                  for(int j = 0; j<5; j++){
                    readLineSensors();
                    pidStraightLineFollower();
                    //delay(10);
                  }
                  Stop();
          
                  //Serial.println("Location Reached");
                  //delay(2000);
          
          

}    

int thArraySearch(int num , int arr[4]){
  for(int k =0; k<4; k++){
    if(num== arr[k]){
      return k;
    }
  } 
  return -1;
    
  
}

void printToOled(int y, String text){
          oled.clearDisplay(); // clear display
          oled.setTextSize(1);          // text size
          oled.setTextColor(WHITE);     // text color
          oled.setCursor(0, y);        // position to display
          oled.println(text); // text to display
          oled.display();               // show on OLED
}


int thGoTo(int num){
    ////////////////////////////////////////////////////////////////////////////////////////////////////
    printToOled(20,String(thCurrentJuncIndex));
    printToOled(30,String(thCurrentLocation));
     floorPattern();
     if(junctionDetected){
        ///////////////////////////////////////////////////
        oled.clearDisplay(); // clear display
          oled.setTextSize(1);          // text size
          oled.setTextColor(WHITE);     // text color
          oled.setCursor(0, 0);        // position to display
          for(int k =0 ; k<4; k++){
            oled.print(String(thCurrentJunc[k]));
            oled.print(" ");
          }
          oled.println(""); // text to display
          oled.display();               // show on OLED
        ////////////////////////////////////////////////////


       Stop();
       if(not thDestinationReached){
            //Do the required operation
            thAutomaticRouting(num);      
            Serial.println("***************************************************************");
            //delay(5000);
       }
       else{  //Do node processing
               Serial.println("node");

       }
       if(thDestination == thCurrentLocation){
         Serial.println("____________Location Reached____________");
         thDestinationReached = true;
         printToOled(30,"Reached");
         delay(500);

       }
       junctionDetected = false;
      
     }  
     else{
       pidStraightLineFollower();     
     } 
     //////////////////////////////////////////////////////////////////////////////////////////////////////////
}

void turnedAngle(int requiredAngle , char turningDirection){

      int initialAngle = readMagAngle();
      int destAngle;
          
      if(turningDirection == 'R'){
        destAngle = initialAngle + requiredAngle;
        if(destAngle>360){
          destAngle= destAngle-360;
        }
        //turnLeft(150);
        Serial.println("Right");
      }  
      else{
        destAngle = initialAngle - requiredAngle;
        if(destAngle<0){
          destAngle= destAngle + 360;
        }
        Serial.println("Left");
        //turnRight(150);
      }
      int turnedAngle = 0;
      while(not ((turnedAngle>destAngle-3) and (turnedAngle<destAngle+3))){
        turnedAngle = readMagAngle();
        if(turningDirection == 'L'){
          turnLeft(200);
          Serial.println("turning left");
        }  
        else{
          turnRight(200);
          Serial.println("turning right");
        }
      }
      Stop();
      Serial.println("Stopped");
      delay(500);

}

int readMagAngle(){
  compass.read();
	int MAngle = compass.getAzimuth();
	//delay(100);
  return MAngle;

}

void thNodeAnalysis(){
   //////////////////////////////////////////////////////////
 
  int thBoxIRCounter = 0;
  //thBoxIRCounter = inputVal[3] + inputVal[4] + inputVal[5] +  inputVal[6] +  inputVal[7] +  inputVal[8] +  inputVal[9] ;
  // for(int i =0; i<2;i++){
  //    readLineSensors();
  //    pidStraightLineFollower();
  //    delay(50);

  // }
  // Stop();

  //Stage 01 preparations of the arm
  if(thStage == 1){
      horizontalGripper(6);
      delay(1000);
      moveVerticalGripper(0);
      delay(2000);
  }

  
  while(thBoxIRCounter<4){
    readLineSensors();
    pidStraightLineFollower();
    thBoxIRCounter = inputVal[3] + inputVal[4] + inputVal[5] +  inputVal[6] +  inputVal[7] +  inputVal[8] +  inputVal[9] ;
  }
  
  Stop();
          //reverse the robot
  // reverse(150,130);
  // delay(1000);
  // settleLine();
  // Stop();
  

  if(thStage == 1){
          forward(150,130);
          delay(80);
          Stop();
          delay(1000);
          //read color sensors
          String colorBox = FindColorBox();////////////////////////////check color//////////////////////
          if((colorBox=="Red") and (not thRedBoxDetected)){
            thExploredBoxes[0] = thCurrentLocation;
            thRedBoxDetected = true;
          }
          else if((colorBox == "Green") and (not thGreenBoxDetected)){
            thExploredBoxes[1] = thCurrentLocation;
            thGreenBoxDetected = true;
           }
          else{
            thExploredBoxes[2] = thCurrentLocation;
          }

          //Update explored boxes list


          //String colorFloor = FindColorFloor();////////////////////////////check color//////////////////////
          oled.clearDisplay();
          oled.setTextSize(1);      
          oled.setTextColor(WHITE); 
          //oled.println("Team Spectro");
          oled.setCursor(0, 10);    
          oled.print(colorBox); 
          oled.display();
          delay(3000);
          moveVerticalGripper(2);
          delay(1000);
          reverse(150,140);
          delay(250);
          Stop();



          


  }




  ////////////////////////////////////////////
  // int nodeCounter = 0;
  // while(nodeCounter<28){
  //     pidStraightLineFollower();
  //     nodeCounter++;
  // }
  // //delay(2000);
  // nodeCounter = 0;
  // Stop();
  // delay(1000);
  if(thStage==3){  
      //if(thPlacingTheBox == 0){   //lifting the box when 0
      if(thBoxGrab){
          reverse(150,140);
          delay(200);
          Stop();
          delay(100);
          //settleLine();
  
      }
    

      moveVerticalGripper(thGripperCommands[thGripperCommandCounter][0]);
      delay(2000);

      if(thBoxGrab){
          forward(150,140);
          delay(600);     //Change this
          Stop();
          delay(100);
          
      }
      

      horizontalGripper(thGripperCommands[thGripperCommandCounter][1]);
      delay(1000);
      if(not thBoxGrab){
          //thBoxGrab = true;
          reverse(150,140);
          delay(400);
          Stop();
          thBoxGrab = true; 
      }
      else{
          thBoxGrab = false; 
      }
      moveVerticalGripper(thGripperCommands[thGripperCommandCounter][2]);
      delay(2000);
        
      


          thGripperCommandCounter ++;  
          reverse(150,140);
          delay(250);
          Stop();                  
     // }
      /*else if(thPlacingTheBox == 1){   //placing the box when 1
          moveVerticalGripper(1);
          delay(1500);
          horizontalGripper(6);
          delay(1000);
          moveVerticalGripper(2);
          delay(1500);

     */ } 
    if(thStage==4){
      Stop();
      beepBuz();
      delay(10000);
    }
    
  /*thPlacingTheBox++;
  if(thPlacingTheBox>1){
    thPlacingTheBox = 0;
    }
  } */

  leftTurn180();
  forward(150,130);
  delay(150);
  Stop();
  settleLine();
  // check this part
  reverse(150,140);
  delay(450);
  Stop();
  settleLine();

  // for better line following
  for (int i = 0; i<5; i++){
    readLineSensors();
    pidStraightLineFollower();
    delay(10);
  }
  settleLine();
  
  
  
  //while(nodeCounter<15){
  //    pidStraightLineFollower();
  //    nodeCounter++;
  //}
  
  //pidStraightLineFollower();
}

void thPathFinder(){
  thMainDirection = readMagAngle();
  Serial.print("Main angle: ");
  Serial.println(thMainDirection);

  while(not thComplete){
      thStageManager();
      if(not thDestinationReached){
              thGoTo(thLocationsToMove[thLocationIndex]);
        }
      else{   //Change this
            //pidStraightLineFollower();
            thNodeAnalysis();
            Stop();
            printToOled(10,"moving to next location");
            delay(500);
            thLocationIndex++;
            if(thLocationsToMove[thLocationIndex]==5){
              printToOled(10,"TH Finished");
              delay(100);
              printToOled(20,String(thExploredBoxes[0]));
              delay(100);
              printToOled(20,String(thExploredBoxes[1]));
              delay(100);
              printToOled(20,String(thExploredBoxes[2]));
              delay(100);

              thStage++; // Going to next stage
              delay(1000);
              printToOled(20,String(thStage));
              thDestinationReached = false;  //Update later
              thLocationIndex = 0;  //go to begining of new list
              //delay(1000);
              //break;
            }
            thDestinationReached = false;
      }
  }
}

int thCheckOrientation(int mainDirection){
    int thCurrentAngle =  readMagAngle();
    int thAngleLimits[4];
    //Initialize limits
    thAngleLimits[0] = mainDirection + 45;
    for(int i = 1; i<4; i++){
      thAngleLimits[i] = thAngleLimits[i-1] + 90;
    }
    for(int k =0; k<4; k++){   //Keeps values betweeen 
      if(thAngleLimits[k]>360){
        thAngleLimits[k] = thAngleLimits[k] - 360;
      }
    }
    //Check for the required sector
    for(int j=1; j<4; j++){
        if(thAngleLimits[j-1] < thAngleLimits[j]){   //No 360 jump
            if((thAngleLimits[j-1]<=thCurrentAngle) and (thCurrentAngle<thAngleLimits[j])){
                return  j;
            }
        }
        else{
          
            if(((thAngleLimits[j-1]<=thCurrentAngle) and (thCurrentAngle<360) ) or ((thCurrentAngle<thAngleLimits[j]) and (thCurrentAngle>0))){
                return  j;
            }

        }
    }
    return 0; // if not in any sectors

}

void thUpdateJunction(){
        int thSector = thCheckOrientation(thMainDirection);
          printToOled(10,String(thMainDirection));
          printToOled(20,String(thSector));
          if(thSector == 0){
            for(int j =0; j<4; j++){
                  thCurrentJunc[j] = thJunc[thCurrentJuncIndex][j];
                }
          }
          else if(thSector == 1){
            thLeftShift_90();
          }
          else if(thSector == 2){
            thShift_180();
          }
          else if(thSector == 3){
            thRightShift_90();
          }
          
         
}

void thIncrementJunc(){
    int thAllignment = thCheckOrientation(thMainDirection);
    if(thAllignment == 0){
        thCurrentJuncIndex++;
    }
    else if(thAllignment == 2){
        thCurrentJuncIndex--;
    }

    if(thCurrentJuncIndex==3){
        thCurrentJuncIndex = 2;
        leftTurn180();
        forward(150,130);
        delay(200);
        Stop();
    }
    else if(thCurrentJuncIndex == -1){
        thCurrentJuncIndex = 0;
        leftTurn180();
        settleLine();
                
    }
    
}

void thStageManager(){
      if(thStage == 0){   //0 - Exploration phase
          thLocationsToMove[0] = 48;
          thLocationsToMove[1] = 32;
          thLocationsToMove[2] = 16;
          thLocationsToMove[3] = 5;
          //__________________________Update thExplored boxes by node analysis in stage == 0_______________________________
          thStage++;       // 1 - Idle phase
      } 
      else if(thStage == 2){  //2 - Tower of hanoi phase 01 building first tower at user input 01
          //Run tower of hanoi
          //Update thLocationsToMove
          for(int i =0; i<3; i++){   //update moving loctions for bulding the tower
                        thLocationsToMove[i*2] = thExploredBoxes[i];
                        thLocationsToMove[i*2+1] = thUserInput01;

          }
          thLocationsToMove[6] = 5;  //Indicates end of array
          thStage++;       //3 - Idle stage
      } 

} 


void moveVerticalGripper(int pos){  
    int upDownGripperLocation = 0;
    if(pos==0){
          upDownGripperLocation = 0;
    }     
    else if(pos == 1){
          upDownGripperLocation = -3500;
    }
    else if(pos == 2){
          upDownGripperLocation = -7000;
    }
    stepper.moveTo(upDownGripperLocation);  //-4000 maximum height and 0 is minimum height 0
    stepper.runToPosition();
}


void horizontalGripper(int pos){
  if (pos==6){
    Gripper.write(15);   //20
    delay(200);
  }
  else if (pos==5){
    Gripper.write(100);   //115
    delay(200);
  }
  else if (pos==4){
    Gripper.write(110);   //120
    delay(200);
  }
  else if (pos==3){
    Gripper.write(170);   //180
    delay(200);
  }
}

bool dipSwitchCheck(int switchPin){
  int state = digitalRead(switchPin); // read the state of the switch

  if (state == LOW) {
    Serial.println("Switch is ON");
    return true;
  } 
  else {
    Serial.println("Switch is OFF");
    return false;
  }

}

void beepBuz(){
  tone(buzzerPin, 1000); // play a tone with frequency 440 Hz for half a second
  delay(200);
  noTone(buzzerPin); // stop playing the tone
  delay(200); // wait for a short time before playing the next tone
  tone(buzzerPin, 1000); // play a tone with frequency 880 Hz for half a second
  delay(300);
  noTone(buzzerPin); // stop playing the tone
  delay(1000); // wait for a longer time before playing the next tone

} 




/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void setup() 
{    
  Serial.begin(9600);           
  Gripper.attach(38); //servo gripper         
  //Initialize magnetometer 
  compass.init();
   // initialize OLED display with address 0x3C for 128x64

  pinMode(switchPinOne, INPUT_PULLUP); // set the switch pin as input with internal pull-up resistor
  pinMode(switchPinTwo, INPUT_PULLUP); // set the switch pin as input with internal pull-up resistor
  pinMode(switchPinThree, INPUT_PULLUP); // set the switch pin as input with internal pull-up resistor

  pinMode(buttonOnePin, INPUT_PULLUP); // set the switch pin as input with internal pull-up resistor
  pinMode(buttonTwoPin, INPUT_PULLUP); // set the switch pin as input with internal pull-up resistor
  pinMode(buttonThreePin, INPUT_PULLUP); // set the switch pin as input with internal pull-up resistor

  //buzzer
  pinMode(buzzerPin, OUTPUT); // set the buzzer pin as output

  if (!oled.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
          Serial.println(F("SSD1306 allocation failed"));
          while (true);
  }      


   ///////////////////// floorPattern //////////////////////////
   
  // initialize sensor values to 0
  for (int i = 0; i < NUM_SENSORS; i++) {
        for (int j = 0; j < NUM_VALUES; j++) {
          sensorValues[i][j] = 0;
        }
  }

  ///////////////////////////////////////////////////////////// 

  //////////////////// Gripper ///////////////////////////////
  // Set the maximum steps per second:
  stepper.setMaxSpeed(1500);
  // Set the maximum acceleration in steps per second^2:
  stepper.setAcceleration(500);
  ///////////////////////////////////////////////////////////

  
  pinMode(ProxSensor_1,INPUT);    
  pinMode(ProxSensor_2,INPUT);    
  pinMode(ProxSensor_3,INPUT);    
  pinMode(ProxSensor_4,INPUT);
  pinMode(ProxSensor_5,INPUT);
  pinMode(ProxSensor_6,INPUT);    
  pinMode(ProxSensor_7,INPUT);    
  pinMode(ProxSensor_8,INPUT);    
  pinMode(ProxSensor_9,INPUT);
  pinMode(ProxSensor_10,INPUT);
  pinMode(ProxSensor_11,INPUT);    
  pinMode(ProxSensor_12,INPUT); 

  ///////////////// ultrasonic sensors /////////////////////////   
  pinMode(trigPin1,OUTPUT);
  pinMode(echoPin1,INPUT);
  pinMode(trigPin2,OUTPUT);
  pinMode(echoPin2,INPUT);
  pinMode(trigPin3,OUTPUT);
  pinMode(echoPin3,INPUT);
  pinMode(trigPin4,OUTPUT);
  pinMode(echoPin4,INPUT);
  pinMode(trigPin5,OUTPUT);
  pinMode(echoPin5,INPUT);
  pinMode(trigPin6,OUTPUT);
  pinMode(echoPin6,INPUT);
  pinMode(trigPin7,OUTPUT);
  pinMode(echoPin7,INPUT);

  

  // Set all the motor control pins to outputs
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  
  // Turn off motors - Initial state
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);

  //////////////////////////////////
    // Set the S0, S1, S2, S3 Pins as Output
  pinMode(S0_PIN_floor, OUTPUT);
  pinMode(S1_PIN_floor, OUTPUT);
  pinMode(S2_PIN_floor, OUTPUT);
  pinMode(S3_PIN_floor, OUTPUT);

  pinMode(S0_PIN_box, OUTPUT);
  pinMode(S1_PIN_box, OUTPUT);
  pinMode(S2_PIN_box, OUTPUT);
  pinMode(S3_PIN_box, OUTPUT);
  //Set OUT_PIN as Input
  pinMode(OUT_PIN_floor, INPUT);
  pinMode(OUT_PIN_box, INPUT);
  // Set Pulse Width scaling to 20%
  digitalWrite(S0_PIN_floor, HIGH);
  digitalWrite(S1_PIN_floor, LOW);
  digitalWrite(S0_PIN_box, HIGH);
  digitalWrite(S1_PIN_box, LOW);
  // Enabl UART for Debugging
  
  delay(2000);         // wait for initializing
  oled.clearDisplay(); // clear display
  oled.setTextSize(1);          // text size
  oled.setTextColor(WHITE);     // text color
  oled.setCursor(0, 10);        // position to display
  oled.println("TEAM SPECTRO"); // text to display
  oled.display();               // show on OLED

  int count = 0;
   //Remove this

  ////////Get dip switch inputs
  if(dipSwitchCheck(switchPinThree)){
        thUserInput01 = 3;
  }
  else if(dipSwitchCheck(switchPinTwo)){
        thUserInput01 = 2;
  }
  else{
        thUserInput01 = 1;
  }

  /////////////////////////////////
  
  buttonOne = dipSwitchCheck(buttonOnePin);
  buttonTwo = dipSwitchCheck(buttonTwoPin);
  buttonThree = dipSwitchCheck(buttonThreePin);

  oled.setCursor(0, 40);    
  oled.print(buttonOne);
  oled.print(" ");
  oled.print(buttonTwo);
  oled.print(" ");
  oled.print(buttonThree);
  oled.println(" ");
  oled.display();
  delay(2000);
 if(not buttonTwo){ 
  moveVerticalGripper(2); //change later
  delay(1500);
  horizontalGripper(6);    //reset positions of the gripper
  delay(1000);
 }




  thMainDirection = readMagAngle();
  //beepBuz();  
}

void loop(){ 
//HelpingStage = 15; //for th
if(buttonTwo){  // FOr calibration of magnetometer
  // int a = thCheckOrientation(thMainDirection);
  // printToOled(10,String(a));
  // delay(100);
  String a = FindColorFloor();
  printToOled(10,a);
  oled.setCursor(0, 30);    
  oled.print(red);
  oled.print(" ");
  oled.print(green);
  oled.print(" ");  
  oled.print(blue);
  oled.print(" ");
  oled.display();
}  
else{


  readLineSensors();  
  oled.clearDisplay();
  oled.setTextSize(1);      
  oled.setTextColor(WHITE);
  oled.setCursor(0, 10);   
  oled.setCursor(0, 30);    
  oled.print(MZmode);   
  oled.display();
//////////////////////////////////////////  HelpingStage 1 - line follow untill green box ////////////////////////////////////////////  
  if (HelpingStage==0){
    MZreadLFSsensors();
    if(MZmode=='c'){ 
        go(140,160);
        delay(200);
    }
    else{
      // has come to the line
      HelpingStage=1;
    }
    
  }

  ///////////////////////////////////////////////// Curve line following ////////////////////
    if (HelpingStage==1){
    // MZreadLFSsensors();
    
    // if(MZmode=='c'){ 
    //     go(140,160);
    //     go(140,160);
    //     go(140,160);
    //     go(140,160);
    //     MZmotorStop();
    //     delay(5000);   //Check color if needed
    //     MZgoAndTurn (90);
    //     MZmotorStop();
    //     delay(1000);
    //     HelpingStage = 5;
    //     //delay(200);
    // }
    // else{
    //   pidLineFollower();
    // }

    readLineSensors();
    int clFloorSensors = inputVal[2]+ inputVal[3]+ inputVal[4]+ inputVal[5]+ inputVal[6]+ inputVal[7]+ inputVal[8]+ inputVal[9];
    if(clFloorSensors>4){
      Stop();
      forward(150,130);
      delay(500);
      Stop();
      delay(500);
      //Check color sensor
      HelpingStage = 2;  // go to tower of hanoi

    }
    pidLineFollower();
  
  }

  if(HelpingStage == 2){  //If green detected
      forward(150,130);
      delay(400);
      if(buttonOne){
          turnRight(200);
          HelpingStage = 5;  // go to tower of hanoi
      }
      else{
          turnLeft(200);
          HelpingStage = 10; //Go to maze solving
          //beepBuz();
          MZpart = 1;

      }
      delay(500);
      Stop();
      forward(150,130);
      delay(500);
      Stop();
      settleLine();
      for(int j =0 ; j<3; j++){
        pidStraightLineFollower();
      }
      settleLine();
      for(int j =0 ; j<3; j++){
        pidStraightLineFollower();
      }
      settleLine();
      for(int j =0 ; j<3; j++){
        pidStraightLineFollower();
      }
      settleLine();
        
      
      

      
}  
  
 

 //////////////////////////////////////////  HelpingStage 5 - Tower Of Hanoi /////////////////////////////////////////////////////////

  if(HelpingStage == 5){
     beepBuz();
     thPathFinder();
     
    
  }

  
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////  HelpingStage 3 - Maze Area ///////////////////////////////////////////////////////////////
if(HelpingStage == 10){

  //beepBuz();
  oled.clearDisplay(); // clear display
  oled.setTextSize(1);          // text size
  oled.setTextColor(WHITE);     // text color
  oled.setCursor(0, 10);        // position to display
  oled.print(inputVal[0]);
  oled.print(inputVal[1]);
  oled.print(inputVal[2]);
  oled.print(inputVal[3]);
  oled.print(inputVal[4]);
  oled.print(inputVal[5]);
  oled.print(inputVal[6]);
  oled.print(inputVal[7]);
  oled.print(inputVal[8]);
  oled.print(inputVal[9]);
  oled.print(inputVal[10]);
  oled.print(inputVal[11]);
  oled.display();               // show on OLED
  ////////////////////////////////////////////////////////////////////////////////////////// 
  
  MZreadLFSsensors(); 
  if (MZmode=='c'||MZmode=='r'||MZmode=='l'||MZmode=='n'){
    if(MZpart==0){
    MZmotorStop();
    delay(500);
    go(140,160);
    go(140,160);
    go(140,160);
      
    }
    MZmotorStop();
    delay(500);
    //go(155,150);
    //go(95,100);
    //go(120,120);
    MZi=MZi+1;
    MZfloorColor = "NO COLOR CHECKING";
    if((MZpart==0)&& (MZmode=='c')){
      MZmotorStop();
      delay(5000);
      go(140,160);
      MZfloorColor = FindColorFloor();
  ///////////////////////////////////////////////////////////////////////////////////////////
  oled.clearDisplay(); // clear display
  oled.setTextSize(1);          // text size
  oled.setTextColor(WHITE);     // text color
  oled.setCursor(0, 50);        // position to display
  // text to display
  oled.println(MZfloorColor);
  oled.display();               // show on OLED
  //////////////////////////////////////////////////////////////////////////////////////////
      delay(5000);
      
    }
    

       
      if((MZpart==0)&& (MZmode=='n')){
        MZmotorStop();
        delay(500);
        MZi=0;
      }
      if ((MZpart==0)&&(MZfloorColor == "Colour Green")){//maze started
            MZmotorStop();
            delay(500);
            go(145,160);
            go(145,160);
            go(145,160);
            //go(95,100);
            //go(100,100);
            
            //MZmotorStop();
            //delay(500);
            readLineSensors();
            MZreadLFSsensors();
            //if(mode=='c'){
            //go(145,150);
            //go(145,150);
            //go(95,100);
              
             MZpart=1;//maze started
             //}
             MZi=0;
             }
       if ((MZpart==1)&&(MZstat==0)){
       //readLineSensors();
       //MZreadLFSsensors();
       MZmazeSolve();
       oled.setTextSize(1);       
       oled.setTextColor(WHITE);
       oled.setCursor(0, 40);    
       oled.print(" after maze solve call ");
       oled.print(MZpath);
       oled.display();
      }

       if ((MZstat==1)&&(MZpart==1)){
         
       }
      if (MZpart==2){
        /*mazeOptimization();
        i=0;

        // Second Pass: run the maze as fast as possible
        //mode = STOPPED;
        }
       if ((stat==1)&&(part==2)){
         //pathIndex = 0;
         //wallfollow()
         stage=1;
         stat = 0;
         part=0;
         i=0;
       }*/
   }
  }
 else{
 MZlinefollow();
  }
}
//////////////////////////////////////////  HelpingStage 4 - Object Avoidance /////////////////////////////////////////////////////////
if(HelpingStage==15){
  // object awoidance
  Stop();
  delay(2000);
  ultrasonic_array();
  while(ultrasonic_distance[0]>20){
    ultrasonic_array();
    forward(150,150);
  }
  
  HCSR04 hc_front_1(trigPin4,echoPin4);
  distance3=hc_front_1.dist();
  HCSR04 hc_front_21(trigPin5,echoPin5);
  distance4=hc_front_21.dist();
  
  if((distance3==100)||(distance3==0)){
    distance3=100;
  }
  if((distance4==100)||(distance4==0)){
    distance4=100;
  }
   oled.clearDisplay();
  oled.setTextSize(1);      
  oled.setTextColor(WHITE);
  oled.setCursor(0, 10);   
  oled.println("Team Spectro");
  oled.setCursor(0, 30); 
  oled.print(distance3);
  oled.print("  ");
  oled.print(distance4);
  oled.print("  ");
  oled.display();
  if (distance3<distance4){
    turnRight(175);
    delay(700);
  }
  if (distance3>distance4){
    turnLeft(175);
    delay(700);
  }
  while (1!=0){
    readLineSensors();
    if((inputVal[0]==1)||(inputVal[11]==1)){
      ultrasonic_part=1;
      break;

    }
    // readLineSensors();
    // for (int i=0;i<12;i++){
    //   ultrasonic_count+=inputVal[i];
    // }
    // if (ultrasonic_count>=6){
    //   Stop();
    //   if((inputVal[0]==1)&&(inputVal[11]==0)){
    //       turnRight(175);
    //       delay(30);

    //   }
    //   if ((inputVal[11]==1)&&(inputVal[0]==0)){
    //     turnLeft(175);
    //     delay(30);
    //   }
    // }
    // if (ultrasonic_count==12){
    //   forward(150,150);
      
    //     if((inputVal[0]==0)||(inputVal[11]==0)){
    //       Stop();
    //       delay(500);
    //        distance6=get_distance(trigPin7,echoPin7);
    //        distance5=get_distance(trigPin6,echoPin6);
    //        if((distance5==50)||(distance5==0)){
    //          distance5==50;
    //        }
    //        if((distance6==50)||(distance6==0)){
    //          distance6==50;
    //        }
    //        if (distance5<distance6){
    //          turnRight(175);
    //          delay(700);
    //          readLineSensors();
    //          while (inputVal[11]>=0){
    //             readLineSensors();
    //             forward(150,150);
    //          }
    //          if(inputVal[11]==1){
    //            Stop();
    //            turnLeft(175);
    //            delay(700);
               
    //          }
             
    //        }
    //        if (distance5>distance6){
    //          turnLeft(175);
    //          delay(700);
    //          readLineSensors();
    //          while (inputVal[0>=0]){
    //            readLineSensors();
    //            forward(150,150);
    //          }
    //          if (inputVal[0]==1){
    //            Stop();
    //            turnRight(175);
    //            delay(700);
               
    //          }
    //        }
           
        
    //   }

    // }
    
      // else{
      //   distance6=get_distance(trigPin7,echoPin7);
      //   distance5=get_distance(trigPin6,echoPin6);
      //   if((distance5==50)||(distance5==0)){
      //       distance5==50;
      // }
      // if((distance6==50)||(distance5==0)){
      //       distance6==50;
      // }
     
  
  ultrasonic_array();
  
 
  
  
   if (ultrasonic_distance[0]<=25){
        Stop();
        delay(600);
      if((ultrasonic_distance[3]==10)&&(ultrasonic_distance[4]==10)){
       if (ultrasonic_distance[1]<ultrasonic_distance[2]){
        // reverse(130,130);
        // delay(300);
        turnRight(175);
        delay(10);

        // while(ultrasonic_distance[5]!=50){
        //   forward(150,150);
        // }
        // turnLeft(175);
        // delay(700);
                

      }
        

        
         else if (ultrasonic_distance[1]>ultrasonic_distance[2]){
          // reverse(130,130);
          // delay(300);
          turnLeft(175);
          delay(10);
          
        }
       
      }
      else if((ultrasonic_distance[1]<5)||(ultrasonic_distance[2]<5)){
        reverse(150,150);
      }
          
   
        
       else if (ultrasonic_distance[3]>ultrasonic_distance[4]){
            turnLeft(175);
             delay(10);

        }
      else if (ultrasonic_distance[3]<ultrasonic_distance[4]){
            turnRight(175);
            delay(10);
           

            }
    //   // else if (ultrasonic_distance[5]>ultrasonic_distance[6]){
    //   //     turnLeft(175);
    //   //    delay(900);
    //   // }
    //   // else if(ultrasonic_distance[5]<ultrasonic_distance[6]){
    //   //   turnRight(175);
    //   //    delay(900);

    //   // }
      else{
        // reverse(120,120);
        // delay(300);
        turnRight(175);
        delay(10);
        
        }
  }
      
    // else if ((ultrasonic_distance[3]+ultrasonic_distance[5])<(ultrasonic_distance[4]+ultrasonic_distance[6])){
    //     error3=(REFERENCE_DISTANCE-ultrasonic_distance[3]);
    //     error4=(REFERENCE_DISTANCE-ultrasonic_distance[5]);
    //     errorD2=(error3+error4)/2;
    //     derivative2=errorD2-previous_errorD2;
    //     integral2+=errorD2;
    //     PID_ErrorD2=(USKp*errorD2+USKd*errorD2+USKi*errorD2);
    //     previous_errorD2=errorD2;
       

    //     left_motor_speed=initial_speed+PID_ErrorD2;
    //     right_motor_speed=initial_speed-PID_ErrorD2;
    //     if (left_motor_speed>=MAX_SPEED){
    //         left_motor_speed=MAX_SPEED;
    //     }
    //     if (right_motor_speed>=MAX_SPEED){
    //       right_motor_speed=MAX_SPEED;
    //     }
    //     if (left_motor_speed<=MIN_SPEED){
    //       left_motor_speed=MIN_SPEED;
    //     }
    //     if (right_motor_speed<=MIN_SPEED){
    //       right_motor_speed=MIN_SPEED;
    //     }
    //     analogWrite(enA,left_motor_speed);
    //     analogWrite(enB,right_motor_speed);
    //     digitalWrite(in1,HIGH);
    //     digitalWrite(in2,LOW);
    //     digitalWrite(in3,HIGH);
    //     digitalWrite(in4,LOW);
        


    // }
    // else if ((ultrasonic_distance[3]+ultrasonic_distance[5])>(ultrasonic_distance[4]+ultrasonic_distance[6])){
    //     error1=(REFERENCE_DISTANCE-ultrasonic_distance[4]);
    //     error2=(REFERENCE_DISTANCE-ultrasonic_distance[6]);
    //     errorD1=(error1+error2)/2;
    //     derivative1=errorD1-previous_errorD1;
    //     integral1+=errorD1;
    //     PID_ErrorD1=(USKp*errorD1+USKd*errorD1+USKi*errorD1);
    //     previous_errorD1=errorD1;
    //     // if(ultrasonic_distance[6]==50){
    //     //   turnRight(175);
    //     //   delay(700);
    //     // }

    //     left_motor_speed=initial_speed-PID_ErrorD1;
    //     right_motor_speed=initial_speed+PID_ErrorD1;
    //     if (left_motor_speed>=MAX_SPEED){
    //         left_motor_speed=MAX_SPEED;
    //     }
    //     if (right_motor_speed>=MAX_SPEED){
    //       right_motor_speed=MAX_SPEED;
    //     }
    //     if (left_motor_speed<=MIN_SPEED){
    //       left_motor_speed=MIN_SPEED;
    //     }
    //     if (right_motor_speed<=MIN_SPEED){
    //       right_motor_speed=MIN_SPEED;
    //     }
        
    //     analogWrite(enA,left_motor_speed);
    //     analogWrite(enB,right_motor_speed);
    //     digitalWrite(in1,HIGH);
    //     digitalWrite(in2,LOW);
    //     digitalWrite(in3,HIGH);
    //     digitalWrite(in4,LOW);



    // }

  else if((ultrasonic_distance[1]<5)||(ultrasonic_distance[2]<5)){
        reverse(150,150);
      }
     else if ((ultrasonic_distance[3])<(ultrasonic_distance[4])){
       turnRight(175);
       delay(10);
     }

       else if ((ultrasonic_distance[3])>(ultrasonic_distance[4])){
       turnLeft(175);
       delay(10);
       }

    else if (ultrasonic_distance[1]>ultrasonic_distance[2]){
         turnLeft(175);
         delay(10);

     }
    
        else if (ultrasonic_distance[1]<ultrasonic_distance[2]){
        turnRight(175);
        delay(10);
        
        }

    //   }
    // else if (ultrasonic_distance[3]<ultrasonic_distance[4]){
        
    //       turnLeft(175);
    //       delay(700);
          
          
    //     }
        


    

    // else if (ultrasonic_distance[3]>ultrasonic_distance[4]){
       
    //       turnRight(175);
    //       delay(700);
        


    //  }
    

    else{

      forward(150,140);
      
    }

    
    oled.clearDisplay();
    oled.setTextSize(1);      
    oled.setTextColor(WHITE);
    oled.setCursor(0, 10);   
    oled.println("Team Spectro");
    oled.setCursor(0, 30);    
    oled.print(distance3);
    oled.print("  ");
    oled.print(distance4);
   oled.print("  ");
   oled.print(ultrasonic_distance[0]);
   oled.print("  ");
   oled.print(ultrasonic_distance[1]);
   oled.print("  ");
   oled.print(ultrasonic_distance[2]);
   oled.print("  ");
   oled.print(ultrasonic_distance[3]);
   oled.print("  ");
   oled.print(ultrasonic_distance[4]);
   oled.print("  ");
   oled.print(ultrasonic_distance[5]);
   oled.print("  ");
   oled.print(ultrasonic_distance[6]);
   oled.print("  ");
   oled.print(PID_ErrorD1);
   oled.print("  ");
   oled.print(PID_ErrorD2);
   oled.print("  ");
 
   
 
   
   oled.display();
   
}
int ultrasonic_count=0;
while(1!=0){
  
    if(ultrasonic_part==1){
    readLineSensors();
    oled.clearDisplay();
  oled.setTextSize(1);      
  oled.setTextColor(WHITE);
  oled.setCursor(0, 10);   
  oled.println("Team Spectro");
  oled.setCursor(0, 30); 
     
  oled.print(inputVal[0]);
  oled.print(" ");
  oled.print(inputVal[1]);
  oled.print(" ");
  oled.print(inputVal[2]);
  oled.print(" ");
  oled.print(inputVal[3]);
  oled.print(" ");
  oled.print(inputVal[4]);
  oled.print(" ");
  oled.print(inputVal[5]);
  oled.print(" ");
  oled.print(inputVal[6]);
  oled.print(" ");
  oled.print(inputVal[7]);
  oled.print(" ");
  oled.print(inputVal[8]);
  oled.print(" ");
  oled.print(inputVal[9]);
  oled.print(" ");
  oled.print(inputVal[10]);
  oled.print(" ");
  oled.print(inputVal[11]);
  oled.print(" ");
  
    oled.display();
    
    for (int i=0;i<12;i++){
      ultrasonic_count+=inputVal[i];
    }
    if (ultrasonic_count>=6){
      Stop();
      while(true){
        readLineSensors();
      if((inputVal[0]==1)&&(inputVal[11]==0)){
          turnRight(175);
          delay(40);

      }
      if ((inputVal[11]==1)&&(inputVal[0]==0)){
        turnLeft(175);
        delay(40);
      }
      if((inputVal[11]==1)&&(inputVal[0]==1)){
        Stop();
        delay(2000);
        break;
      }
    }
    
    Stop();
    delay(3000);
    ultrasonic_part=2;
    }
    }
    if (ultrasonic_part==2){
     
    // ultrasonic_count=0;
    // for (int i=0;i<12;i++){
    //   ultrasonic_count+=inputVal[i];
    // }
    // if (ultrasonic_count>=10){
      while(true){
      HCSR04 hc_front_1(trigPin4,echoPin4);
   distance5=hc_front_1.dist();
  HCSR04 hc_front_21(trigPin5,echoPin5);
  distance6=hc_front_21.dist();


    if ((distance5==50)||(distance5==0)){
      distance5==50;
    }
    if ((distance6==50)||(distance6==0)){
      distance6==50;
    }
    oled.clearDisplay();
  oled.setTextSize(1);      
  oled.setTextColor(WHITE);
  oled.setCursor(0, 10);   
  oled.println("Team Spectro");
  oled.setCursor(0, 30);    
  oled.print(distance5);
  oled.print("  ");
  oled.print(distance6);
   oled.print("  ");
    oled.display();


      if(ultrasonic_distance[1]<ultrasonic_distance[2]){
        turnLeft(175);
        delay(20);
    }
    if(ultrasonic_distance[1]>ultrasonic_distance[2]){
        turnLeft(175);
        delay(20);
    }
      // forward(150,130);
      if(distance5<distance6){
        errorD=(distance6-distance5);
        derivative=errorD-previous_errorD;
        integral+=errorD;
        PID_ErrorD=(USKp*errorD+USKd*errorD+USKi*errorD);
        previous_errorD=errorD;
        left_motor_speed=150+PID_ErrorD;
        right_motor_speed=140-PID_ErrorD;
        if (left_motor_speed>=MAX_SPEED){
            left_motor_speed=MAX_SPEED;
        }
        if (right_motor_speed>=MAX_SPEED){
          right_motor_speed=MAX_SPEED;
        }
        if (left_motor_speed<=MIN_SPEED){
          left_motor_speed=MIN_SPEED;
        }
        if (right_motor_speed<=MIN_SPEED){
          right_motor_speed=MIN_SPEED;
        }
        analogWrite(enA,left_motor_speed);
        analogWrite(enB,right_motor_speed);
        digitalWrite(in1,HIGH);
        digitalWrite(in2,LOW);
        digitalWrite(in3,HIGH);
        digitalWrite(in4,LOW);
        

       
      }
       if(distance5>distance6){
        errorD1=(distance6-distance5);
    //     error2=(REFERENCE_DISTANCE-ultrasonic_distance[6]);
    //     errorD1=(error1+error2)/2;
        derivative1=errorD1-previous_errorD1;
        integral1+=errorD1;
        PID_ErrorD1=(USKp*errorD1+USKd*errorD1+USKi*errorD1);
        previous_errorD1=errorD1;
    //     // if(ultrasonic_distance[6]==50){
    //     //   turnRight(175);
    //     //   delay(700);
    //     // }

        left_motor_speed=150+PID_ErrorD1;
        right_motor_speed=140-PID_ErrorD1;
        if (left_motor_speed>=MAX_SPEED){
            left_motor_speed=MAX_SPEED;
        }
        if (right_motor_speed>=MAX_SPEED){
          right_motor_speed=MAX_SPEED;
        }
        if (left_motor_speed<=MIN_SPEED){
          left_motor_speed=MIN_SPEED;
        }
        if (right_motor_speed<=MIN_SPEED){
          right_motor_speed=MIN_SPEED;
        }
        
        analogWrite(enA,left_motor_speed);
        analogWrite(enB,right_motor_speed);
        digitalWrite(in1,HIGH);
        digitalWrite(in2,LOW);
        digitalWrite(in3,HIGH);
        digitalWrite(in4,LOW);

      }
      if((distance5>=50)&&(distance6>=50)){
        ultrasonic_part=3;
      }
    }
    }
    // }
   

 

    
    // if(ultrasonic_part==3){
    //   Stop();
    //   delay(2000);
    //   readLineSensors();
    // ultrasonic_count=0;
    // for (int i=0;i<12;i++){
    //   ultrasonic_count+=inputVal[i];
      
    // }

    
    //  while(true){
      
    //  if((inputVal[0]==0)&&(inputVal[11]==1)){
    //       turnRight(175);
    //       delay(40);
    //  }
    //  if((inputVal[0]==1)&&(inputVal[11]==0)){
    //       turnLeft(175);
    //       delay(40);
    //  }
    //  if((inputVal[0]==0)&&(inputVal[11]==0)){
    //    Stop();
    //    delay(2000);
    //    ultrasonic_part=4;
    //    break;
    //  }
     

    // } 
    // }
   if(ultrasonic_part==3){
     Stop();
     delay(500);
      readLineSensors();
      if((inputVal[0]==1)||(inputVal[1]==1)||(inputVal[2]==1)||(inputVal[3]==1)||(inputVal[4]==1)||(inputVal[5]==1)||(inputVal[6]==1)||(inputVal[7]==1)||(inputVal[8]==1)||(inputVal[9]==1)||(inputVal[10]==1)||(inputVal[11]==1)){
        Stop();
        delay(1000);
        break;
      }

    }
    
    

        
              
           
           
        // if (distance3<distance4){
        //   turnRight(175);
        //   delay(600);
        //   readLineSensors();
        //   while (inputVal[11]!=1){
        //     readLineSensors();
        //     forward(150,150);
        //   }
        //   oled.clearDisplay();
        //   oled.setTextSize(1);      
        //   oled.setTextColor(WHITE);
        //   oled.setCursor(0, 10);   
        //   oled.println("Team Spectro");
        //   oled.setCursor(0, 30); 
        //   oled.print("HI HOW ARE YOU");
        //   Stop();
        //   delay(1000);
        //   if(inputVal[11]==1){
        //     Stop();
        //     turnLeft(175);
        //     delay(600);
                
        //   }
              
        // }
        // if (distance3>distance4){
        //   turnLeft(175);
        //   delay(600);
        //   readLineSensors();
        //   while (inputVal[0]!=0){
        //     readLineSensors();
        //     forward(150,150);
        //   }
        //   if (inputVal[0]==1){
        //     Stop();
        //     delay(10000);
        //     turnRight(175);
        //     delay(600);
                
        //   }
        // }
      
    
           

    oled.clearDisplay();
    oled.setTextSize(1);      
    oled.setTextColor(WHITE);
    oled.setCursor(0, 10);   
    oled.println("Team Spectro");
    oled.setCursor(0, 30);    
    oled.print(distance3);
    oled.print("  ");
    oled.print(distance4);
    oled.print("  ");
    oled.display();
    
}
  HelpingStage==19;
   
}

if(HelpingStage==19){
    RampUpLoop();
    
}

//////////////////////////////////////////  HelpingStage 5 - Ramp /////////////////////////////////////////////////////////
if(HelpingStage==20){
  beepBuz();
  HelpingStage = 21;
  // RAMP 
  //HelpingStage==25;
  // end of the SLRC 
   
}

if(HelpingStage==21){
    ArrowFollowerLoop();
}
if(HelpingStage==22){
   RampDOwnLoop();
}
/////////////////////////////////////////   END OF LOOP  //////////////////////////////////////////////////////////////////////////////
}
}



