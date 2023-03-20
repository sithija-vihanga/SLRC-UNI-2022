#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
//#include <HCSR04.h>//old sonic sensor library
#include <Wire.h>
#include <Adafruit_Sensor.h>
//#include <Adafruit_HMC5883L.h> check wether it is possible to include this(magnatometer library)
#include <NewPing.h>

#define SCREEN_WIDTH 128 // OLED display width,  in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

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
/////////////////////////////////////////////////////////////////////////////////

// declare an SSD1306 display object connected to I2C
Adafruit_SSD1306 oled(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
  int error_list[5] = {0,0,0,0,0};  //For both pid and straight line pid functions (Change if necessary)

  //////////   For straight Lines   ///////// //////
  float kpS = 0.6;
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
///////////////////////////////////////////////////////////////////////////////////////////////////////////////
char MZcurentDerection = 'r'; //at the green squre
int MZsteps = 100;//used to traverse one edge of squre
int MZextraSteps = 40;// used to check the line (for extra length)
char MZmode;//mode of line
//char MZjunctionType='I';
char MZpreviousMode;
int MZX=0;//cordinates at the grean squre
int MZY=0;
int MZpart=1;//starting of maze
int MZpreviousAction=0;// 4 for go left////////////6 for go right//////////////8 for go up////2 for go down///(look for keyboard)
int MZturn180=80 ;// used to turn 180
int MZturn90=40;// used to turn 90
int MZrotatedAngle=0;

String MZfloorColor = " ";//color of the floor
String MZjunctionType=" ";//TLR for T mode one junction /// TR - right T junction // TL - left T junction;// CROSS - cross junction 


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
////////////////////////////////////////////////////////////////////////////////////
int strtrted=0;
int stage=0;


// Motor A connections
int enA = 2;
int in1 = 39;
int in2 = 37;
// Motor B connections
int enB = 3;
int in3 = 43;
int in4 = 41;

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int i=0;
int part = 0;// if maze solved this will be 2 and yet to be sloved, this will be 1 (used in the loop)
char path[100] = " ";
char mode = ' ';
int pid;
unsigned char pathLength = 0; // the length of the path
int pathIndex = 0; // used to reach an specific array element.

unsigned int stat = 0; // solving = 0; reach Maze End = 1

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
 delay(80);
}


void runExtraInch(void)
{
  go(140,140);//to move extra length need to change the delay
  go(140,140);//to move extra length need to change the delay
  motorStop();//need to define
  delay(200);
}

void goAndTurn( int Degrees){
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
  delay(1500);
  }
  if( Degrees==90){
  analogWrite(enA, 200);
  analogWrite(enB, 200);
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH) ;
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW) ; 
  delay(750);
  }
}

void mazeTurn (char dir) 
{
       
  if (dir=='L')
  {
      // Turn Left
       goAndTurn (270);
       //lineFollow();      
  }   
    
   else if(dir=='R'){ // Turn Right
       goAndTurn (90);
       //lineFollow();     
   }   
       
   else if(dir== 'B'){ // Turn Back
       goAndTurn (180);
       //lineFollow();     
   }   
       
    else if(dir== 'S'){ // Go Straight
       runExtraInch();
      // lineFollow(); 
    }
  }



void mazeEnd(void)
{
  motorStop();
  delay(1000);

       oled.setTextSize(1);       
       oled.setTextColor(WHITE);
       oled.setCursor(0, 40);    
       oled.print(6);
       oled.print(mode);
       oled.display();
  
  Serial.print("  pathLenght == ");
  Serial.println(pathLength);
  //stat = 1;it will trurn back
  delay(1000);
  //mode = "AE";
}
void simplifyPath()
{
  // only simplify the path if the second-to-last turn was a 'B'
  if(pathLength < 3 || path[pathLength-2] != 'B'){
    delay(10);
  }
  else{
  int totalAngle = 0;
  int i;
  for(i=1;i<=3;i++)
  {
   
     if((path[pathLength-i])=='R'){
      
        totalAngle += 90;
     }
      else if((path[pathLength-i])=='L'){
      
        totalAngle += 270;
     }
       else if((path[pathLength-i])=='B'){
      
        totalAngle += 180;
     }
  }
  
  

  // Get the angle as a number between 0 and 360 degrees.
  totalAngle = totalAngle % 360;

  // Replace all of those turns with a single one.
 if((totalAngle)==0)
  {
    
  path[pathLength - 3] = 'S';
  }
  else if((totalAngle)==90){
  path[pathLength - 3] = 'R';
  }
  else if((totalAngle)==180){
  path[pathLength - 3] = 'B';
  }
  else if((totalAngle)==270){
  path[pathLength - 3] = 'L';
  
  }

  // The path is now two steps shorter.
  pathLength -= 2;
  pathIndex=pathLength-1;//index of last element in path
  

  //change the oder and derections of the path for reverse
  for (int j=0; j<=pathLength-1;j++){
    if (path[j]=='R'){
      path[j]='L';
      j++;
    }
    else if(path[j]=='L'){
      path[j]='R';
      j++;
    }
    else if(path[j]=='S'){
      path[j]='S';
      j++;
    }
  }
  
}
}

void readLFSsensors()
{
  
  if(((inputVal[2]==0)&&(inputVal[3]==0)&&(inputVal[4]==0)&&(inputVal[5]==0)&&(inputVal[6]==0)&&(inputVal[7]==0)&&(inputVal[8]==0)&&(inputVal[9]==0))){
    mode = 'n';//No line
  }
  else if((inputVal[2]==1)&&(inputVal[3]==1)&&(inputVal[4]==1)&&(inputVal[5]==1)&&(inputVal[6]==1)&&(inputVal[7]==1)&&(inputVal[8]==1)&&(inputVal[9]==1)){
    mode = 'c';//cross line
  }
  else if(((inputVal[2]==0)&&(inputVal[1]==0)&&(inputVal[10]==1)&&(inputVal[9]==1))&&((inputVal[3]==1)||(inputVal[4]==1)||(inputVal[5]==1)||(inputVal[6]==1)||(inputVal[7]==1)||(inputVal[8]==1))){
    mode = 'l';//Right line Turn (w r t robot)
  }
  else if(((inputVal[2]==1)&&(inputVal[1]==1)&&(inputVal[10]==0)&&(inputVal[9]==0))&&((inputVal[3]==1)||(inputVal[4]==1)||(inputVal[5]==1)||(inputVal[6]==1)||(inputVal[7]==1)||(inputVal[8]==1))){
    mode = 'r';//Left line Turn (w r t robot)
  }
  else{
    mode = 'f';//following line
  }
  
  /*if     ((LFSensor[0]== 1 )&&(LFSensor[1]== 1 )&&(LFSensor[2]== 1 )&&(LFSensor[3]== 1 )&&(LFSensor[4]== 1 )&&(LFSensor[5]== 1 )&&(LFSensor[6]== 1 ))  {mode = 'n'; }//n= no line
  else if((LFSensor[0]== 0)&&(LFSensor[1]== 0 )&&(LFSensor[5]== 1 )&&(LFSensor[6]== 1 )) {mode = 'l';}//left turn
  else if((LFSensor[0]== 1)&&(LFSensor[1]== 1 )&&(LFSensor[5]== 0 )&&(LFSensor[6]== 0 )) {mode = 'r';}//right turn
  else if((LFSensor[0]== 0 )&&(LFSensor[1]== 0 )&&(LFSensor[2]== 0 )&&(LFSensor[3]== 0 )&&(LFSensor[4]== 0 )&&(LFSensor[5]== 0 )&&(LFSensor[6]== 0 ))  {mode = 'c'; }// cont line
 // else if((LFSensor[0]== 1 )&&(LFSensor[1]== 1 )&&(LFSensor[2]== 0 )&&(LFSensor[3]== 0 )&&(LFSensor[4]== 0 ))  {mode = 'l'; }//after the new 2 sensor used we dont want to use this
 // else if((LFSensor[0]== 0 )&&(LFSensor[1]== 0 )&&(LFSensor[2]== 0 )&&(LFSensor[3]== 1 )&&(LFSensor[4]== 1 ))  {mode = 'r'; }
  else {mode = 'f';}//following line*/
    
}
/////////////////////////////////////////////////////////////////////////////////////from gridMaze///////////////////////
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
   
    if (MazeError>30){
        //turnLeft(200);
        //delay(110);
        go(60,200);
        go(60,200);
        //motorStop();
        //delay(100);
  }
    else if (MazeError<-30){
      //turnRight(200);
      //delay(110);
      go(200,60);
      go(200,60);
      //motorStop();
      //delay(100);
      
  }
   else{
     //Stop();
     //printToOled(10,"Settled");
     motorStop();
     break;
         
   }
  }
}
/////////////////////////////////////////////////////////////////////////////////////from gridMaze end///////////////////////

void mazeOptimization (void)// check the syntax
{
  //while (!stat)
  //{
    //readLFSsensors();
       oled.setTextSize(1);       
       oled.setTextColor(WHITE);
       oled.setCursor(0, 40);    
       oled.print(4);
       oled.print(mode);
       oled.display();  
      
      //go(95,100);
      if (mode== 'f'){
        linefollow();
      }  
      else if(mode== 'c'){
        if (pathIndex <= 0) {mazeEnd (); }
        else {mazeTurn (path[pathIndex]); pathIndex=pathIndex-1;}
      }  
      else if(mode== 'l'){
        if (pathIndex <= 0) {mazeEnd ();} 
        else {mazeTurn (path[pathIndex]); pathIndex=pathIndex-1;}
      }  
      else if(mode=='r'){
        if (pathIndex <= 0) {mazeEnd ();} 
        else {mazeTurn (path[pathIndex]); pathIndex=pathIndex-1;}
      }  
    }    
   


void recIntersection(char Direction)
{
  path[pathLength] = Direction; // Store the intersection in the path variable.
  pathLength ++;
  simplifyPath(); // Simplify the learned path.
             oled.setTextSize(1);       
            oled.setTextColor(WHITE);
            oled.setCursor(0, 50);    
            oled.print(5);
            oled.print(mode);
            oled.display();
}

void linefollow(){ 

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


  base_speed = 140; 
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

void mazeSolve(void)
{           oled.setTextSize(1);       
            oled.setTextColor(WHITE);
            oled.setCursor(0, 40);    
            oled.print(1);
            oled.print("maze solve");
            oled.display();
             
//       go(140,140);
              
       if (mode=='n')
        {   

            motorStop();
            //delay(500);
            goAndTurn (180);
            motorStop();
            delay(500);
            settleLine();
            //lineFollow();

            recIntersection('B');
        }
            
          
         else if(mode== 'c'){
                       
           
            motorStop();
            //delay(500);
            //runExtraInch();
            //go(140,140);
            //go(140,140);
            //motorStop();
            //delay(250);
           readLineSensors();
           readLFSsensors();
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

           if(MZfloorColor=="Colour Blue"){
            goAndTurn (180);
            motorStop();
            delay(200);
            settleLine();
            recIntersection('B');
            
           }

           else if(MZfloorColor=="Colour Red"){//end of the Maze
            //goAndTurn (180);
            motorStop();
            delay(10000);
            //settleLine();
            //recIntersection('B');
            
           }
           else if(MZfloorColor=="Colour Green"){
            goAndTurn (180);
            motorStop();
            delay(200);
            settleLine();
            recIntersection('B');
           }
           
        else{
          if(mode=='n'){
            MZjunctionType="TLR";
          }
          else if(mode=='f'){
            MZjunctionType="CROSS";
          }
          
            //if ((strtrted==0)){
//              go(145,150);
//              //go(145,150);
//              //go(145,150);
//              strtrted++;
              
            //}
            //else
            //{goAndTurn (270);motorStop();delay(200);settleLine(); recIntersection('L');} // or it is a "T" or "Cross"). In both cases, goes to LEFT
          }
            //else mazeEnd();
          } 
         
         
            
         else if(mode== 'r'){

             motorStop();
             //delay(500);
             //go(140,140);
             //go(140,140);
             //runExtraInch();
             //motorStop();
             readLineSensors();
             readLFSsensors();
           
            if (mode == 'n') {
              goAndTurn (90);
              motorStop();
              delay(500);
              settleLine();
              
              recIntersection('R');
              }
            else {
              goAndTurn (90);
              motorStop();
              delay(500);
              settleLine();
            recIntersection('R');
            MZjunctionType="TR";
         } 
         } 
            
           else if(mode== 'l'){

             motorStop();
             //delay(500);
             //go(140,140);
             //go(140,140);
             //runExtraInch();
             //motorStop();
             readLineSensors();
             readLFSsensors();
           
            if (mode == 'n') {
              goAndTurn (270);
              motorStop();
              delay(500);
              settleLine();
              
              recIntersection('L');
              }
            else {
              goAndTurn (270);
              motorStop();
              delay(500);
              settleLine();
            recIntersection('L');
            MZjunctionType="TL";
         } 
         }   
         
         else if(mode== 'f'){
            linefollow();
                
        
         }
}

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
  if (b < 50)
  {
    Serial.println("B Colour Blue");
    color = "Colour Blue";
  }
  else if (g < 50)
  {
    Serial.println("B Colour Green");
    color = "Colour Green";
  }
  else if (r < 50)
  {
    Serial.println("B Colour Red");
    color = "Colour Red";
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
//    oled.clearDisplay();
//  oled.setTextSize(1);      
//  oled.setTextColor(WHITE); 
//  //oled.println("Team Spectro");
//  oled.setCursor(0, 30);    
//  oled.print(r);
//  oled.print(" ");
//   oled.print(g);
//    oled.print(" ");
//    oled.print(b);
//     oled.println(" "); 
//  oled.display();
  ////////////////////////////////////////////
  if((r<600)&&(g<600)&&(b<600)){
    color = "Colour white";
  }
  else{
  if ((b < 700)&&(r>1000))
  {
    Serial.println("F Colour Blue");
    color = "Colour Blue";
  }

  else if (r < 750)
  {
    Serial.println("F Colour Red");
    color = "Colour Red";
  }
    else if ((b < 1000)&&(g <1000))
  {
    Serial.println("F Colour Green");
    color = "Colour Green";
  }
  return(color);
}
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

//}




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


}



void loop(){ 
   readLineSensors();
  
  oled.clearDisplay();
  oled.setTextSize(1);      
  oled.setTextColor(WHITE);
  oled.setCursor(0, 10);   
  //oled.println("Team Spectro");
  oled.setCursor(0, 30);    
  oled.print(mode);  

  
  oled.display();
///////////////////////////////////////////////////////////////////////////////////  
///////////////////////////////////////////////////////////////////////////////////
  readLFSsensors(); 
  if (mode=='c'||mode=='r'||mode=='l'||mode=='n'){
    //motorStop();
    go(150,150);
    //go(155,150);
    //go(95,100);
    go(100,100);
    i=i+1;

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
    //if (i>=2){
       
      if((part==0)&& (mode=='n')){
        motorStop();
        delay(500);
        i=0;
      }
      if ((part==0)&&(MZfloorColor == "Colour Green")){//maze started
            go(145,150);
            //go(145,150);
            //go(95,100);
            //go(100,100);
            
            motorStop();
            delay(500);
           readLineSensors();
           readLFSsensors();
            //if(mode=='c'){
              go(145,150);
              go(145,150);
              //go(95,100);
              
              part=1;//maze started
              //}
             i=0;
             }
       if ((part==1)&&(stat==0)){
        mazeSolve();
       oled.setTextSize(1);       
       oled.setTextColor(WHITE);
       oled.setCursor(0, 40);    
       oled.print(2);
       oled.print(path);
       oled.display();
     //     i=0;// First pass to solve the maze
      }

       if ((stat==1)&&(part==1)){
         
         /*goAndTurn(180);
         go(95,100);
         go(95,100);
         go(95,100);
         go(95,100);
         //pathIndex = 0;
         stat = 0;
         part=2;
         i=0;*/
       }
      if (part==2){
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
 linefollow();

  }
}



/////////////////////////////////////////////////////////////////////////////////////////////////
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

void linefollowlast(){
  /*inputVal[0]  = digitalRead(ProxSensor_left);
  inputVal[1]  = digitalRead(ProxSensor_1);
  inputVal[2]  = digitalRead(ProxSensor_2);
  inputVal[3]  = digitalRead(ProxSensor_3);
  inputVal[4]  = digitalRead(ProxSensor_4);
  inputVal[5]  = digitalRead(ProxSensor_5);
  inputVal[6]  = digitalRead(ProxSensor_right);
  
  error = (-inputVal[0]*3000 -inputVal[1]*1000 - inputVal[2]*200 + inputVal[4]*200 +inputVal[5]*1000 + inputVal[6]*3000)/(2*(inputVal[0]+inputVal[1]+inputVal[2]+inputVal[3]+inputVal[4]+inputVal[5]+inputVal[6]));

  for (int i = 0; i<4 ; i++){
    error_list[i] = error_list[i+1];
  }

  error_list[4] = error;
  d_error = error_list[4] - error_list[3];

  i_error = i_error + error;
  int pid = kp*error + kd*d_error + ki*i_error;  

  oled.clearDisplay();
  oled.setTextSize(1);       
  oled.setTextColor(WHITE);
  oled.setCursor(0, 40);    
  oled.print(inputVal[0]);
  oled.print(inputVal[1]);
  oled.print(inputVal[2]);
  oled.print(inputVal[3]);
  oled.print(inputVal[4]);
  oled.print(inputVal[5]);  
  oled.println(inputVal[6]);
  
  oled.display();

int base_speed = 80;  
 int plus_speed = 80;
 int min_speed = 80;


  base_speed = 80;  
  plus_speed = base_speed + pid;
  min_speed = base_speed - pid;



  if(pid>100){
    turnLeft();
    delay(100);
  }
  else if(pid<-100){
    turnRight();
    delay(100);
  }
  else{
  base_speed = 120;  
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
  forward(plus_speed,min_speed);*/

}
