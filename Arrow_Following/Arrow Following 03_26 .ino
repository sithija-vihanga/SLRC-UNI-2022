# include <Wire.h>
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


int sensorValues[NUM_SENSORS][NUM_VALUES];  // For 12 line sensors to store 10 previous values
int lineSensorCount[3] = {0, 0, 0} ;

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
const int ProxSensor_10=A9;      // 4 left
const int ProxSensor_11=A10;     // 5 left
const int ProxSensor_12=A11;     // 6 left

///////////////////////////////////////////////////////////////////////////////////////////


int inputVal[12] = {0,0,0,0,0,0,0,0,0,0,0,0};

int count = 0;
  

int error_list[5] = {0,0,0,0,0};  //For both pid and straight line pid functions (Change if necessary)

////////////  For curved paths   //////////////////
  float kp = 0.6;
  float kd = 0.3;
  float ki = 0.0000;
  
////////////// For both curve and straight pids ///////
int error = 0;
int d_error = 0;
int  i_error = 0;

// Motor A connections
int enA = 2;
int in1 = 39;    //5   
int in2 = 37;   //4 
// Motor B connections
int enB = 3;
int in3 = 43;     //7
int in4 = 41;    //6

int HelpingStage = 21;


#define S0_PIN_box 9
#define S1_PIN_box 10
#define S2_PIN_box 11
#define S3_PIN_box 12
#define OUT_PIN_box 13

#define S0_PIN_floor 5
#define S1_PIN_floor 4
#define S2_PIN_floor 7
#define S3_PIN_floor 6
#define OUT_PIN_floor 8


int red = 0;  
int green = 0;  
int blue = 0; 



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
bool AF_ramp = true;


int AF_leftMostIndex[11];
int AF_rightMostIndex[11];
int AF_leftDiff[10];
int AF_righttDiff[10];



void setup(){    
  Serial.begin(9600);           
  //Initialize magnetometer 
  compass.init();
   // initialize OLED display with address 0x3C for 128x64
  if (!oled.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
          Serial.println(F("SSD1306 allocation failed"));
          while (true);
  } 


   
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

  
}

void loop(){ 

  if(HelpingStage == 19){
        
  RampUpLoop();
  
  } 

  if(HelpingStage == 21){
        
  ArrowFollowerLoop();
  }
  
  if(HelpingStage == 22){
        
  RampDOwnLoop();
  }
}

void ArrowFollowerLoop(){
    
  AF_pidArrowFollower();
  printToOled(10,"Arrow follow");  
  AF_arrowReadLineSensors();  
  if (AF_start == true){  
    AF_arrowReadLineSensors();        
    forward(150,135);
    delay(1000);
    AF_start = false;    
  }


  

  
  String colour= FindColorFloor();
  // if (colour == "Colour Blue"){
  //     Stop() ;
  //     delay(1000);      
  //     pidLineFollower();
  //     delay(400); 
  //     HelpingStage =  22;   
  //     forward(120,110);    
  //     delay(400);
  // }
  AF_currentAllIR = 0;
  for (int i=0;i<12;i++){
    if (inputVal[i] == 1)
          AF_currentAllIR += 1;
  }
     if (AF_currentAllIR >=9){
       Stop() ;
      delay(1000); 
      // forward(120,110);
      // delay(1500);   
      HelpingStage =  22;   

     }
  
  
  if (AF_whiteDetected == true){
      AF_arrowReadLineSensors();    

      AF_pidArrowFollower();

      //////////////  Checking Direction //////////////

      if((AF_blackAgain == true)&& AF_currentAllIR >= 3){


      Stop();
      delay(1000);
      AF_blackAgain = false ;
      printToOled(10, "black");
      for (int i = 0; i<11; i++){
        forward(120,110);

        AF_arrowReadLineSensors();
        int AF_min = 0;
        int AF_max = 0;       
        for (int j = 0 ; j<12 ; j++){
          if (inputVal[j] == 1){
            AF_min = j;
            break;        
          }  
        }

        for (int k = 11 ; k>=0 ; k--){
          if (inputVal[k] == 1){
            AF_max = k;
            break;      
          }  
        }

        AF_leftMostIndex[i] = AF_min;
        AF_rightMostIndex[i] = AF_max;
      }
    
        int err_arrowLeft = 0;
        int err_arrowRight = 0;

        

        for (int l = 0; l<10;l++){
          AF_leftDiff[l] = AF_leftMostIndex[l] - AF_leftMostIndex[l+1];
          err_arrowLeft += (AF_leftDiff[l]);
          
          AF_righttDiff[l] = AF_rightMostIndex[l] -AF_rightMostIndex[l+1];  
          err_arrowRight += AF_righttDiff[l];    
        } 

        for (int i = 0; i<11; i++){
          reverse(120,110);
          delay(25);
        
        } 

        if (err_arrowLeft + err_arrowRight > 0){
          turnRight(180);
          delay((err_arrowLeft + err_arrowRight)*2.0);  
          AF_pidArrowFollower();
          delay(200);
        }

        else{
              turnLeft(180);
              delay((-1)*(err_arrowLeft + err_arrowRight)*2.0);  
              AF_pidArrowFollower();
              delay(200);           
        }       

           
    AF_pidArrowFollower() ;   

  
    
    
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
  
  if (AF_ramp == true){
    readLineSensors();
    
    for (int i = 0; i<12;i++){
      if (inputVal[i] == 1){
          AF_currentAllIR += 1;
      }
      
    }
    
    if (AF_currentAllIR > 2){
      AF_ramp = false;   
      forward(150,135);   
      delay(200); 
      AF_currentAllIR = 0;
                  
    }
  } 
  
  else{
    AF_currentAllIR = 0;
    rampPidLineFollower();
    for (int i = 0; i<12;i++){
      if (inputVal[i] == 1){
          AF_currentAllIR += 1;
      }
      
    }
    
    if (AF_currentAllIR > 8){
      Stop(); 
      delay(3000); 
      HelpingStage = 20;

      AF_currentAllIR = 0;        
    }    
  } 
  
}

void RampDOwnLoop(){AF_currentAllIR = 0;
  
  if (AF_end == true){
    readLineSensors();
    
    for (int i = 0; i<12;i++){
      if (inputVal[i] == 1){
          AF_currentAllIR += 1;
      }
      
    }
    
    if (AF_currentAllIR > 2){
      AF_end = false;   
      forward(150,135);   
      delay(200); 
      AF_currentAllIR = 0;
                  
    }
  } 
  
  else{
    AF_currentAllIR = 0;
    pidLineFollower();
        
    delay(1000);
    Stop();
    delay(1000);
    HelpingStage = 25;    
    for (int i = 0; i<12;i++){
      if (inputVal[i] == 1){
          AF_currentAllIR += 1;
      }
      
    }
    
    if (AF_currentAllIR > 8){
      Stop(); 
      delay(3000); 
      HelpingStage = 20;

      AF_currentAllIR = 0;        
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
  error = 10*(inputVal[0]*6 +inputVal[1]*5 +inputVal[2]*4 + inputVal[3]*3 +inputVal[4]*2 + inputVal[5] -(inputVal[11]*6 +inputVal[10]*5 +inputVal[9]*4 + inputVal[8]*3 +inputVal[7]*2 + inputVal[6]) ) ; //(2*(inputVal[0]+inputVal[1]+inputVal[2]+inputVal[3]+inputVal[4]+inputVal[5]+inputVal[6]));

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

void forward(int lSpeed, int rSpeed){
  analogWrite(enA, lSpeed);
  analogWrite(enB, rSpeed);
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW) ;
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW) ;

}

void reverse(int lSpeed, int rSpeed){
  analogWrite(enA, lSpeed);
  analogWrite(enB, rSpeed);
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH) ;
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH) ;
}

void Stop(){
  analogWrite(enA, 100);
  analogWrite(enB, 100);
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW) ;
  digitalWrite(in3, LOW);
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

void printToOled(int y, String text){
  oled.clearDisplay();           // clear display
  oled.setTextSize(1);           // text size
  oled.setTextColor(WHITE);      // text color
  oled.setCursor(0, y);          // position to display
  oled.println(text);            // text to display
  oled.display();                // show on OLED
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

  int base_speed = 120;  
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


void rampPidLineFollower(){
  //floorPattern();
  readLineSensors();
  //Error function
  error = 1*(inputVal[0]*6 +inputVal[1]*5 +inputVal[2]*4 + inputVal[3]*3 +inputVal[4]*2 + inputVal[5] -(inputVal[11]*6 +inputVal[10]*5 +inputVal[9]*4 + inputVal[8]*3 +inputVal[7]*2 + inputVal[6]) ) ; //(2*(inputVal[0]+inputVal[1]+inputVal[2]+inputVal[3]+inputVal[4]+inputVal[5]+inputVal[6]));

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

  int base_speed = 220;  
  int plus_speed = 80;
  int min_speed = 80;

  //base_speed = 80;  
  plus_speed = base_speed + pid;
  min_speed = base_speed - pid;

  //Check for Higher pid values to get to the line
  if(pid>100){
    turnRight(220);   //200
    delay(100);      //250
  }
  else if(pid<-100){
    turnLeft(220);    //200
    delay(100);      //250
  } 
  else{
  base_speed = 220;  
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

