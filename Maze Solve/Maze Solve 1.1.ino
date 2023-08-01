#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
//#include <HCSR04.h>//old sonic sensor library
#include <Wire.h>
#include <Adafruit_Sensor.h>
//#include <Adafruit_HMC5883L.h> check wether it is possible to include this(magnatometer library)
#include <NewPing.h>

#define COLLISION_DISTANCE 52
#define MAX_SPEED 180
#define  REFERENCE_DISTANCE 30

#define MAX_DISTANCE 200 // Maximum distance we want to ping for (in centimeters)
#define NUM_READINGS 5 // Number of readings to take for each distance measurement

#define SCREEN_WIDTH 128 // OLED display width,  in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

// declare an SSD1306 display object connected to I2C
Adafruit_SSD1306 oled(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

///////////////////////////////////////////////////////////////////// color sensor variables//////////////////
int MZS0 = 14; //pin names
int MZS1 = 15;
int MZS2 = 16;
int MZS3 = 17;
int MZout = 18;

int MZredFrequency = 0;
int MZgreenFrequency = 0;
int MZblueFrequency = 0;
//////////////////////////////////////////////////////////////////////////////////////////////////////////////
  int error_list[5] = {0,0,0,0,0};  //For both pid and straight line pid functions (Change if necessary)

  //////////   For straight Lines   ///////// //////
  float kpS = 0.7;
  float kdS = 0.3;
  float kiS = 0.0000;

  ////////////  For curved paths   //////////////////
  float kp = 0.6;
  float kd = 0.;
  float ki = 0.0000;

  ////////////// For both curve and straight pids ///////
  int error = 0;
  int d_error = 0;
  int  i_error = 0;
///////////////////////////////////////////////////////////////////////////////////////////////////////////////
char MZcurentDerection = 'r'; //at the green squre
int MZsteps = 100;//used to traverse one edge of squre
int MZextraSteps = 40;// used to check the line (for extra length)
char MZmode;//mode of line
char MZjunctionType='I';
char MZpreviousMode;
int MZX=0;//cordinates at the grean squre
int MZY=0;
int MZpart=1;//starting of maze
int MZpreviousAction=0;// 4 for go left////////////6 for go right//////////////8 for go up////2 for go down///(look for keyboard)
int MZturn180=80 ;// used to turn 180
int MZturn90=40;// used to turn 90
int MZrotatedAngle=0;



/////////////////////////////////////////////////////////////////////////////////////////////////////////////

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

int inputVal[12] = {0,0,0,0,0,0,0,0,0,0,0,0};//stores line sensors values
int LFSensor[12]={0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
float direct;

//float distance1,distance2,distance3,distance4,distance5,distance6,distance7,fronterrorD1,fronterrorD2,backerrorD1,backerrorD2,errorD1,errorD2,derivative1,derivative2,integral1=0.0,integral2=0.0,previousErrorD1=0.0,previousErrorD2=0.0,outputD1,outputD2,MIN_SPEED=40;
//float Kp_d=0.9,Ki_d=0,Kd_d=0.9;
//int speedL,speedR,basespeed=110;



float wallSensorReading[8] = {0.0,0.0,0.0,0.0,0.0,0.0,0.0};
float wallSensors[8] = {0.0,0.0,0.0,0.0,0.0,0.0,0.0};

//sensor 
const int trigPin1 = 4;//distance1
const int echoPin1 = 5;
const int trigPin2 = 26;//distance2
const int echoPin2 = 27;
const int trigPin3 = 14;//distance3
const int echoPin3 = 15;
const int trigPin4 = 16;//distance4
const int echoPin4 = 17;
const int trigPin5 = 22;//distance5
const int echoPin5 = 23;
const int trigPin6 = 22;//distance6/////////////////////////////pins need to change
const int echoPin6 = 23;
const int trigPin7 = 22;//distance7/////////////////////////////pins need to change
const int echoPin7 = 23;

NewPing sonar1(trigPin1, echoPin1, MAX_DISTANCE);
NewPing sonar2(trigPin2, echoPin2, MAX_DISTANCE);
NewPing sonar3(trigPin3, echoPin3, MAX_DISTANCE);
NewPing sonar4(trigPin4, echoPin4, MAX_DISTANCE);
NewPing sonar5(trigPin5, echoPin5, MAX_DISTANCE);
NewPing sonar6(trigPin6, echoPin6, MAX_DISTANCE);
NewPing sonar7(trigPin7, echoPin7, MAX_DISTANCE);

// Create an instance of the HMC5883L sensor object
//Adafruit_HMC5883L mag = Adafruit_HMC5883L();


// Motor A connections
int enA = 2;
int in1 = 39;
int in2 = 37;
// Motor B connections
int enB = 3;
int in3 = 43;
int in4 = 41;

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

void motorStop(){
   //stop the wheels
  //analogWrite(enA, x);
  //analogWrite(enB, y);
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
 //delay(200);//take a decision to use or not delays
}

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


void setup() 
{    
    Serial.begin(9600);           
  //Initialize magnetometer 
  //compass.init();
   // initialize OLED display with address 0x3C for 128x64
  if (!oled.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
          Serial.println(F("SSD1306 allocation failed"));
          while (true);
  }           

  
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

  // color sensor
  pinMode(MZS0, OUTPUT);
  pinMode(MZS1, OUTPUT);
  pinMode(MZS2, OUTPUT);
  pinMode(MZS3, OUTPUT);
  pinMode(MZout, INPUT);
  
  digitalWrite(MZS0, HIGH);
  digitalWrite(MZS1, HIGH);
  //////////////////////////////////
    delay(2000);         // wait for initializing
  oled.clearDisplay(); // clear display
  oled.setTextSize(1);          // text size
  oled.setTextColor(WHITE);     // text color
  oled.setCursor(0, 10);        // position to display
  oled.println("TEAM SPECTRO"); // text to display
  oled.display();               // show on OLED

  int count = 0;


  
}

void loop()
{ 
 MZsolve();  
}

void printToOled(int y, char text){
          oled.clearDisplay(); // clear display
          oled.setTextSize(1);          // text size
          oled.setTextColor(WHITE);     // text color
          oled.setCursor(0, y);        // position to display
          oled.println(text); // text to display
          oled.display();               // show on OLED
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
/////////////////////////////////////////////////////////////////////////////////////////////////
void settleLine(){
  int MazeError=1;
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
        turnLeft(200);
        delay(110);
        motorStop();
  }
    else if (MazeError<-50){
      turnRight(200);
      delay(110);
      motorStop();
      
  }
   else{
     //Stop();
     //printToOled(10,"Settled");
     motorStop();
     break;
         
   }
  }
}
////////////////////////////////////////////////////////////////////////////////////////////////////////

void MZupdateMode(){
  readLineSensors();
  
  // line detected inputValue[i]==0.
  
  if((inputVal[0]==0)&&(inputVal[1]==0)&&(inputVal[2]==0)&&(inputVal[3]==0)&&(inputVal[4]==0)&&(inputVal[5]==0)&&(inputVal[6]==0)&&(inputVal[7]==0)&&(inputVal[8]==0)&&(inputVal[9]==0)&&(inputVal[10]==0)&&(inputVal[11]==0)){
    MZmode = 'N';//No line
  }
  else if((inputVal[0]==1)&&(inputVal[1]==1)&&(inputVal[2]==1)&&(inputVal[3]==1)&&(inputVal[4]==1)&&(inputVal[5]==1)&&(inputVal[6]==1)&&(inputVal[7]==1)&&(inputVal[8]==1)&&(inputVal[9]==1)&&(inputVal[10]==1)&&(inputVal[11]==1)){
    MZmode = 'C';//cross line
  }
  else if((inputVal[0]==0)&&(inputVal[1]==0)&&(inputVal[10]==1)&&(inputVal[11]==1)){
    MZmode = 'L';//Right line Turn (w r t robot)
  }
  else if((inputVal[0]==1)&&(inputVal[1]==1)&&(inputVal[10]==0)&&(inputVal[11]==0)){
    MZmode = 'R';//Left line Turn (w r t robot)
  }
  else{
    MZmode = 'F';//following line
  }

   ///////////////////////////////////////////////////////////////////////////////////////////
  oled.clearDisplay(); // clear display
  oled.setTextSize(1);          // text size
  oled.setTextColor(WHITE);     // text color
  oled.setCursor(0, 10);        // position to display
  oled.println(MZjunctionType); // text to display
  oled.display();               // show on OLED
  //////////////////////////////////////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////////////////////////////////
  oled.clearDisplay(); // clear display
  oled.setTextSize(1);          // text size
  oled.setTextColor(WHITE);     // text color
  oled.setCursor(0, 40);        // position to display
  oled.println(MZmode); // text to display MZcurentDerection
  oled.println(MZmode); // text to display
  oled.display();               // show on OLED
   
  //////////////////////////////////////////////////////////////////////////////////////////
  

}
///////////////////////////////////////////////////////////////////////// solve the maze///////////////////////////////////////////////////
void MZsolve(){


  if (MZpart==1){
  MZgoRight();//maze started
  MZpart=2;
  
  }
  if(MZpart==2){
    //printToOled(40,"o");
    if(MZjunctionType=='I'){
    //MZpart = MZpart +0;
    // straight line w r t robot
        if(MZpreviousAction==6){
          MZgoRight();
        }
        if(MZpreviousAction==8){
          MZgoUp();
        }
        if(MZpreviousAction==4){
          MZgoLeft();
        }
        if(MZpreviousAction==2){
          MZgoDown();
        }
    
    }
    if(MZjunctionType=='W'){
    //MZpart = MZpart +0;
    // left turn w r t robot

       if(MZpreviousAction==6){
          MZgoUp();
          
        }
        if(MZpreviousAction==8){
           MZgoLeft();
        }
        if(MZpreviousAction==4){
                ///////////////////////////////////////////////////////////////////////////////////////////
  oled.clearDisplay(); // clear display
  oled.setTextSize(1);          // text size
  oled.setTextColor(WHITE);     // text color
  oled.setCursor(0, 40);        // position to display
  oled.println(MZmode); // text to display MZcurentDerection
  oled.println(MZpreviousAction); // text to display
  oled.display();               // show on OLED
 
   
  //////////////////////////////////////////////////////////////////////////////////////////
           MZgoDown();
         
        }
        if(MZpreviousAction==2){
          MZgoRight();
        } 
    }
    if(MZjunctionType=='D'){
    //MZpart = MZpart +0;
    // right turn w r t robot
    if(MZpreviousAction==6){
          MZgoDown();
          
        }
        if(MZpreviousAction==8){
           MZgoRight();
        }
        if(MZpreviousAction==4){
          MZgoUp();
         
        }
        if(MZpreviousAction==2){
          MZgoLeft();
        } 
    }

   
    
    if(MZjunctionType=='T'){
    // MZpart = MZpart +0;
    // T junction w r t robot
    }
    
    if( MZjunctionType='Y'){
    // MZpart = MZpart +0;
    // T junction w r t robot from right
    }

    if( MZjunctionType='Z'){
      // MZgoDown();////////////////////////replace this
    // MZpart = MZpart +0;
    // T junction w r t robot from left
       ///////////////////////////////////////////////////////////////////////////////////////////
  oled.clearDisplay(); // clear display
  oled.setTextSize(1);          // text size
  oled.setTextColor(WHITE);     // text color
  oled.setCursor(0, 40);        // position to display
  oled.println("Z Junction detected"); // text to display
  oled.display();               // show on OLED
  //////////////////////////////////////////////////////////////////////////////////////////
    }
    
    if(MZjunctionType=='P'){
    //MZpart = MZpart +0;
    // cross junction w r t robot
    }
    if(MZjunctionType=='N'){//no line
    //MZpart = MZpart +0;
    // no line w r t robot
        if(MZpreviousAction==6){
          MZgoLeft();
          
        }
        if(MZpreviousAction==8){
           MZgoDown();
        }
        if(MZpreviousAction==4){
          MZgoRight();
         
        }
        if(MZpreviousAction==2){
          MZgoUp();
        } 
    
    }
  }
  
}
///////////////////////////////////////////////////////////////////////// solved ///////////////////////////////////////////////////////////////////////////////////
void MZcheckLine(){
  // read line sensor values detect wether cross(P),T or just line
    ///////////////////////////////////////////////////////////////////////////////////////////
  oled.clearDisplay(); // clear display
  oled.setTextSize(1);          // text size
  oled.setTextColor(WHITE);     // text color
  oled.setCursor(0, 50);        // position to display
  oled.println("check line"); // text to display//
  oled.display();               // show on OLED
  //////////////////////////////////////////////////////////////////////////////////////////
  motorStop();
  //delay(10000);
  MZpreviousMode= MZmode;
  int j=0;
  if(MZmode=='N'){
    MZjunctionType='N';//no line junction
  }
  else{
  while(MZextraSteps>j){
    forward(150,150);
    delay(10);
    j++;
  }
  
  MZupdateMode();
  
  if((MZpreviousMode=='C')&&(MZmode=='F')){
    MZjunctionType='P';//cross junction
  }
  else if((MZpreviousMode=='C')&&(MZmode=='N')){
    MZjunctionType='T';//T junction middle
  }
  else if((MZpreviousMode=='R')&&(MZmode=='F')){
    MZjunctionType='Y';//T junction from right
  }
  else if((MZpreviousMode=='L')&&(MZmode=='F')){
    MZjunctionType='Z';//T junction from left
  }
  else if((MZpreviousMode=='R')&&(MZmode=='N')){
    MZjunctionType='D';//just passed right turn immideately take it///////////////////////////////////////////////////
  }
  else if((MZpreviousMode=='L')&&(MZmode=='N')){
    MZjunctionType='W';//just passed left turn immideately take it////////////////////////////////////////////////////
  }
  else if((MZpreviousMode=='F')&&(MZmode=='F')){
    MZjunctionType='I';//straight line
  }  
}
}

/////////////////////////////////////////////////////////////////////////////////////////

void MZgoLeft(){
  //first turn left
  //go untill steps are finished
    ///////////////////////////////////////////////////////////////////////////////////////////
     motorStop();
  oled.clearDisplay(); // clear display
  oled.setTextSize(1);          // text size
  oled.setTextColor(WHITE);     // text color
  oled.setCursor(0, 40);        // position to display
  oled.println("Go Left"); // text to display MZcurentDerection
  oled.display();               // show on OLED
   
  //////////////////////////////////////////////////////////////////////////////////////////
   MZturn('l');
   MZupdateDerection(MZrotatedAngle);
   int i=0;
   while(i<MZsteps){//MZsteps should give
    MZupdateMode();
    pidStraightLineFollower();
    delay(50);
    if ((MZmode=='C')||(MZmode=='R')||(MZmode=='L')||(MZmode=='N')){
      motorStop();
      //MZcheckLine();
      break;
    }
    i++;
   }
   motorStop();
   MZcheckLine();
   MZX--;
   MZpreviousAction=4;
  
}

void MZgoRight(){
     ///////////////////////////////////////////////////////////////////////////////////////////
      motorStop();
  oled.clearDisplay(); // clear display
  oled.setTextSize(1);          // text size
  oled.setTextColor(WHITE);     // text color
  oled.setCursor(0, 40);        // position to display
  oled.println("Go Right"); // text to display MZcurentDerection
  oled.display();               // show on OLED
  
  //////////////////////////////////////////////////////////////////////////////////////////
   MZturn('r');
   MZupdateDerection(MZrotatedAngle);
   int i=0;
   while(i<MZsteps){//MZsteps should give
    MZupdateMode();
    pidStraightLineFollower();
    delay(50);
     if ((MZmode=='C')||(MZmode=='R')||(MZmode=='L')||(MZmode=='N')){
      motorStop();
      //MZcheckLine();
      break;
    }
    i++;
   }
   motorStop();
   MZcheckLine();
   MZX++;
   MZpreviousAction=6;
  
}
void MZgoUp(){
    ///////////////////////////////////////////////////////////////////////////////////////////
     motorStop();
  oled.clearDisplay(); // clear display
  oled.setTextSize(1);          // text size
  oled.setTextColor(WHITE);     // text color
  oled.setCursor(0, 40);        // position to display
  oled.println(" Go up"); // text to display MZcurentDerection
  oled.display();               // show on OLED
   
  //////////////////////////////////////////////////////////////////////////////////////////
   MZturn('u');
   MZupdateDerection(MZrotatedAngle);
   int i=0;
   while(i<MZsteps){//MZsteps should give
    MZupdateMode();
    pidStraightLineFollower();
    delay(50);
    if ((MZmode=='C')||(MZmode=='R')||(MZmode=='L')||(MZmode=='N')){
      motorStop();
      //MZcheckLine();
      break;
    }
    i++;
   }
   motorStop();
   MZcheckLine();
   MZY--;
   MZpreviousAction=8;
  
}
void MZgoDown(){
  
   MZturn('d');
   MZupdateDerection(MZrotatedAngle);
   int i=0;
   
  ///////////////////////////////////////////////////////////////////////////////////////////
   motorStop();
  oled.clearDisplay(); // clear display
  oled.setTextSize(1);          // text size
  oled.setTextColor(WHITE);     // text color
  oled.setCursor(0, 40);        // position to display
  oled.println("go down"); // text to display MZcurentDerection
  oled.display();               // show on OLED
 
  //////////////////////////////////////////////////////////////////////////////////////////
   while(i<MZsteps){//MZsteps should give
     MZupdateMode();
     pidStraightLineFollower();
     delay(50);

     if ((MZmode=='C')||(MZmode=='R')||(MZmode=='L')||(MZmode=='N')){
      motorStop();
      //MZcheckLine();
      break;
    }
    i++;
   }
   motorStop();
   MZcheckLine();
   MZY++;
   MZpreviousAction=2;
  
}

void MZturn(char MZdere){ //'l' for left, 'r' for right, 'u' for up, 'd' for down.
  // derections with respect to the magnatometer on the green squre
  if(!(MZdere == MZcurentDerection)){
    
  
  if (MZcurentDerection == 'r'){
    if (MZdere == 'u'){
      MZturnDegrees(270);
    }
    if (MZdere == 'd'){
      MZturnDegrees(90);
    }
    if (MZdere == 'l'){
      MZturnDegrees(180);
    }
  }

  if (MZcurentDerection == 'l'){
    if (MZdere == 'u'){
      MZturnDegrees(90);
    }
    if (MZdere == 'd'){
      MZturnDegrees(270);
    }
    if (MZdere == 'r'){
      MZturnDegrees(180);
    }
  }

  if (MZcurentDerection == 'u'){
    if (MZdere == 'l'){
      MZturnDegrees(270);
    }
    if (MZdere == 'r'){
      MZturnDegrees(90);
    }
    if (MZdere == 'd'){
      MZturnDegrees(180);
    }
  }

  if (MZcurentDerection == 'd'){
    if (MZdere == 'r'){
      MZturnDegrees(270);
    }
    if (MZdere == 'l'){
      MZturnDegrees(90);
    }
    if (MZdere == 'u'){
      MZturnDegrees(180);
    }
  }

     
  
}
else{
  delay(10);
}
}

void MZupdateDerection(int MZnewderection){
  if(!( MZnewderection==0)){
    
  
  if (MZcurentDerection == 'r'){
    if (MZnewderection == 90){
      MZcurentDerection = 'd';
    }
    if (MZnewderection == 180){
      MZcurentDerection = 'l';
    }
     if (MZnewderection == 270){
      MZcurentDerection = 'u';
    }
    
  }

   if (MZcurentDerection == 'l'){
    if (MZnewderection == 90){
      MZcurentDerection = 'u';
    }
    if (MZnewderection == 180){
      MZcurentDerection = 'r';
    }
     if (MZnewderection == 270){
      MZcurentDerection = 'd';
    }
    
  }

    if (MZcurentDerection == 'u'){
    if (MZnewderection == 90){
      MZcurentDerection = 'r';
    }
    if (MZnewderection == 180){
      MZcurentDerection = 'd';
    }
     if (MZnewderection == 270){
      MZcurentDerection = 'l';
    }
    
  }

    if (MZcurentDerection == 'd'){
    if (MZnewderection == 90){
      MZcurentDerection = 'l';
    }
    if (MZnewderection == 180){
      MZcurentDerection = 'u';
    }
     if (MZnewderection == 270){
      MZcurentDerection = 'r';
    }
    
  }
 
}
}


void MZflorColor(){    // will give the color of the floor (as frequency)

  //use if conditios to detect color
  digitalWrite(MZS2, LOW);
  digitalWrite(MZS3, LOW);
  delay(50);
  
  MZredFrequency = pulseIn(MZout, LOW);
  
  digitalWrite(MZS2, HIGH);
  digitalWrite(MZS3, HIGH);
  delay(50);
  
  MZgreenFrequency = pulseIn(MZout, LOW);
  
  digitalWrite(MZS2, LOW);
  digitalWrite(MZS3, HIGH);
  delay(50);
  
  MZblueFrequency = pulseIn(MZout, LOW);
}

void MZturnDegrees(int MZangle){

  bool MZturned = false;
  ///////////////////////////////////////////////////////////////////////////////////////////
  oled.clearDisplay(); // clear display
  oled.setTextSize(1);          // text size
  oled.setTextColor(WHITE);     // text color
  oled.setCursor(0, 20);        // position to display
  oled.println(MZjunctionType); // text to display
  oled.display();               // show on OLED
  //delay(10000);
  //////////////////////////////////////////////////////////////////////////////////////////
 
  
  if (MZangle == 90){//right

//    while(!(MZturned)){
              ///////////////////////////////////////////////////////////////////////////////////////////
  oled.clearDisplay(); // clear display
  oled.setTextSize(1);          // text size
  oled.setTextColor(WHITE);     // text color
  oled.setCursor(0, 50);        // position to display
  oled.println("Turning Right"); // text to display//
  oled.display();               // show on OLED
  //////////////////////////////////////////////////////////////////////////////////////////
      turnRight(200);
      delay(750);
      motorStop();
      settleLine();
      MZrotatedAngle=90;
  }

    if (MZangle == 270){//left
      

  ///////////////////////////////////////////////////////////////////////////////////////////
  oled.clearDisplay(); // clear display
  oled.setTextSize(1);          // text size
  oled.setTextColor(WHITE);     // text color
  oled.setCursor(0, 50);        // position to display
  oled.println("Turning left"); // text to display//
  oled.display();               // show on OLED
  //////////////////////////////////////////////////////////////////////////////////////////
    turnLeft(200);
    delay(750);
      motorStop();
      settleLine();
      MZrotatedAngle=270;
  }

    if (MZangle == 180){//left can take right as well

  ///////////////////////////////////////////////////////////////////////////////////////////
  oled.clearDisplay(); // clear display
  oled.setTextSize(1);          // text size
  oled.setTextColor(WHITE);     // text color
  oled.setCursor(0, 50);        // position to display
  oled.println("Turning left 180"); // text to display//
  oled.display();               // show on OLED
  //////////////////////////////////////////////////////////////////////////////////////////
    turnLeft(200);
    delay(1500);
      MZturned = true;
      motorStop();
      settleLine();
      MZrotatedAngle=180;
    }
}

void pidStraightLineFollower(){
  //floorPattern();
  readLineSensors();
  
  error = 10*( inputVal[3]*6 +inputVal[4]*3 + inputVal[5]*2 - ( + inputVal[8]*6 +inputVal[7]*3 + inputVal[6]*2) ) ; //(2*(inputVal[0]+inputVal[1]+inputVal[2]+inputVal[3]+inputVal[4]+inputVal[5]+inputVal[6]));

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


  base_speed = 160; 
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
