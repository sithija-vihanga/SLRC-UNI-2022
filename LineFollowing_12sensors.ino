  #include <Wire.h>
  #include <Adafruit_GFX.h>
  #include <Adafruit_SSD1306.h>
  #include <QMC5883LCompass.h>
  QMC5883LCompass compass;

  #define SCREEN_WIDTH 128 // OLED display width,  in pixels
  #define SCREEN_HEIGHT 64 // OLED display height, in pixels
  #define NUM_SENSORS 12   //For floor pattern
  #define NUM_VALUES 6   //for floor pattern


  // declare an SSD1306 display object connected to I2C
  Adafruit_SSD1306 oled(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

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
  int thUserInput01 = 1; //Change these
  int thUserInput02 = 1; // Change thses

  int thExploredBoxes[3] = {48, 32, 16}; // 0: largest box 1: middle box 2: Smallest box
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


  int inputVal[12] = {0,0,0,0,0,0,0,0,0,0,0,0};

  // Motor A connections
  int enA = 2;
  int in1 = 5;
  int in2 = 4;
  // Motor B connections
  int enB = 3;
  int in3 = 7;
  int in4 = 6;



void setup() 
{    
  Serial.begin(9600);           
  //Initialize magnetometer 
  compass.init();
   // initialize OLED display with address 0x3C for 128x64
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

  delay(2000);         // wait for initializing
  oled.clearDisplay(); // clear display

  oled.setTextSize(1);          // text size
  oled.setTextColor(WHITE);     // text color
  oled.setCursor(0, 10);        // position to display
  oled.println("TEAM SPECTRO"); // text to display
  oled.display();               // show on OLED

  //Remove this
  thMainDirection = readMagAngle();
}
  int count = 0;
  

int error_list[5] = {0,0,0,0,0};  //For both pid and straight line pid functions (Change if necessary)

//////////   For straight Lines   ///////// //////
float kpS = 0.5;
float kdS = 0.2;
float kiS = 0.0000;

////////////  For curved paths   //////////////////
float kp = 0.6;
float kd = 0.;
float ki = 0.0000;

////////////// For both curve and straight pids ///////
int error = 0;
int d_error = 0;
int  i_error = 0;



void loop(){ 
  /////////////////////////////////////////////////////////////////////////
    //thUpdateJunction();

    ///////////////////////////////////////////////////////////////////////////////////
    
    thPathFinder();
    printToOled(10,String(thStage));


    //floorPattern();
    //readLineSensors();
    //pidLineFollower();
    
    /*delay(1000);
    rightTurn90();
    delay(2000);
    leftTurn90(); */
    
    //settleLine();
    //delay(5000);

    /////////////////////////////////////////////////////////////  
    /*if(thStarted){
      thStarted = false;
      pidLineFollower();
      delay(1000);
      pidStraightLineFollower();
      delay(500);      
      


    }*/
   /*
    if(not thDestinationReached){
           thGoTo(32);
    }
   else{   //Change this
        //pidStraightLineFollower();
        thNodeAnalysis();
        
        //forward(150,150);
        //delay(2000);
        //Stop();
        //delay(100);
        //leftTurn180();
        //delay(100);
        //pidStraightLineFollower();
        //forward(150,150);
        //delay(2000); 
   }   */
   ////////////////////////////////////////////////// 
   /*
   delay(2000);
   turnedAngle(90,'L'); */

  
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
  turnRight(150);
  delay(600);
  settleLine();   //#
  Stop();
}

void leftTurn90(){
  turnLeft(150);
  delay(600);
  settleLine();  //#
  Stop();
}

void leftTurn180(){
  turnLeft(150);
  delay(1500);
  Stop();
}

void forward(int lSpeed,int rSpeed){
  analogWrite(enA, lSpeed);
  analogWrite(enB, rSpeed);
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW) ;
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW) ;
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

  int base_speed = 80;  
  int plus_speed = 80;
  int min_speed = 80;

  //base_speed = 80;  
  plus_speed = base_speed + pid;
  min_speed = base_speed - pid;

  //Check for Higher pid values to get to the line
  if(pid>100){
    turnRight(150);
    delay(250);
  }
  else if(pid<-150){
    turnLeft(150);
    delay(250);
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
  plus_speed = base_speed + pid;
  min_speed = base_speed - pid;


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
}

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
                      else if((lineSensorCount[0] >2 ) and (lineSensorCount[1] >=1) and (lineSensorCount[2] < 2 )){
                              linePathPattern = "HardRightTurn";
                              //forward(50,50);
                              pathCrossing = true;
                      }
                      else if((lineSensorCount[0] <2 ) and (lineSensorCount[1] >=1) and (lineSensorCount[2] >2 )){
                      //else{
                              linePathPattern = "HardLeftTurn";
                              //forward(50,50);
                              pathCrossing = true;
                      }

                      else if((lineSensorCount[0] <2 ) and (lineSensorCount[1] >=1) and (lineSensorCount[2] < 2 )){
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
                    }
                    else{
                      //turnRight
                      Serial.println("Turn Right");
                      printToOled(10,"Right");
                      rightTurn90();
                      settleLine();  //#
                      
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
      turnLeft(100); 
  }
    else if (MazeError<-10){
      turnRight(100); 
  }
   else{
     Stop();
     printToOled(10,"Settled");
     break;
         
   }
  }
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
         delay(5000);

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
          turnLeft(150);
          Serial.println("turning left");
        }  
        else{
          turnRight(150);
          Serial.println("turning right");
        }
      }
      Stop();
      Serial.println("Stopped");
      delay(5000);

}

int readMagAngle(){
  compass.read();
	int MAngle = compass.getAzimuth();
	//delay(100);
  return MAngle;

}

void thNodeAnalysis(){
  //floorPattern();
  int nodeCounter = 0;
  while(nodeCounter<23){
      pidStraightLineFollower();
      nodeCounter++;
  }
  //delay(2000);
  nodeCounter = 0;
  Stop();
  delay(1000);
  leftTurn180();
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
            delay(5000);
            thLocationIndex++;
            if(thLocationsToMove[thLocationIndex]==5){
              printToOled(10,"TH Finished");
              thStage++; // Going to next stage
              delay(1000);
              printToOled(20,String(thStage));
              thDestinationReached = false;  //Update later
              thLocationIndex = 0;  //go to begining of new list
              delay(1000);
              //break;
            }
            thDestinationReached = false;
      }
  }
}

int thCheckOrientation(){
    int thCurrentAngle =  readMagAngle();
    int thAngleLimits[4];
    //Initialize limits
    thAngleLimits[0] = thMainDirection + 45;
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
        int thSector = thCheckOrientation();
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
    int thAllignment = thCheckOrientation();
    if(thAllignment == 0){
        thCurrentJuncIndex++;
    }
    else if(thAllignment == 2){
        thCurrentJuncIndex--;
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

/*void settleLine(){
  settleLine();
  pidStraightLineFollower();
  delay(500);
  Stop();
  settleLine();  
} */



    

