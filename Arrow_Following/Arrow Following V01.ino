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

// Motor A connections
int enA = 2;
int in1 = 39;    //5   
int in2 = 37;   //4 
// Motor B connections
int enB = 3;
int in3 = 43;     //7
int in4 = 41;    //6


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


int count = 0;
  

int error_list[5] = {0,0,0,0,0};  //For both pid and straight line pid functions (Change if necessary)



////////////  For curved paths   //////////////////
float kp = 1.5;
float kd = 0.7;

float ki = 0.0000;

////////////// For both curve and straight pids ///////
int error = 0;
int d_error = 0;
int  i_error = 0;

bool whiteDetected = false;
bool blackAgain = true;

int leftSensorVal = 0;
int rightSensorVal = 0;

int currentAllIR = 0;
int currentLeftIR = 0;
int currentRightIR = 0;


void loop(){ 
  
  
  arrowReadLineSensors();
  
  if ((whiteDetected == true) && (blackAgain == true)) {
    stop();
    delay(1000);
    blackAgain = false ;
    printToOled(10, "start");    
  }

  
  if (whiteDetected == true){

    if (currentAllIR >= 5){
      int diff_IR[12];
      int leftDiff = 0;
      int rightDiff = 0;
      
      for (int i = 0; i<12; i++){
        diff_IR[i] = inputVal[i];
      }

      forward(120,110);
      delay(100);
      stop();
      delay(400);
      arrowReadLineSensors();

      for (int i = 0; i<12; i++){
        diff_IR[i] -= inputVal[i]; 

        if (i%6 == 0){
            leftDiff += diff_IR[i];
        }    
        else{
            rightDiff += diff_IR[i];
        }         
        if (leftDiff > rightDiff){
          turnLeft(110);
          delay(400);     
        }
        else{
          turnRight(110);
          delay(400); 
          
        }
        printToOled(10, "one");   
      }
    }
    
    if (leftSensorVal > rightSensorVal){
        turnLeft(140);
        delay(250);
        leftSensorVal = 0;
        rightSensorVal = 0;  
        stop() ; 
        delay(200)    ;
        pidArrowFollower();
        printToOled(10, "two");
    }

    else if (leftSensorVal > rightSensorVal){
        turnRight(140);
        delay(250);
        leftSensorVal = 0;
        rightSensorVal = 0; 
        stop() ; 
        delay(200)    ;
        pidArrowFollower();   
        printToOled(10, "three");           
    }    
    else{
      forward(140,130);
      delay(400);
      pidArrowFollower();
    }
    pidArrowFollower();
    
  }


  if (whiteDetected == false){
    forward(150,135);    
    blackAgain = true ;
    leftSensorVal = 0;
    rightSensorVal = 0;   
    printToOled(10, "end")  ;   
  }  
}







void arrowReadLineSensors(){            
                          
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

  whiteDetected = false;
    
    currentAllIR = 0;
    currentLeftIR = 0;
    currentRightIR = 0;
 

  for (int i = 0 ; i<12 ; i++){
    
      if ((inputVal[i] == 1) && (i<8) && (i>3)){
      whiteDetected = true;
      
      if (i%6 == 0){
        leftSensorVal += 1;  
        currentLeftIR += 1;       
      }     
      
      else{
        rightSensorVal += 1;
        currentRightIR += 1;
      }    
      
    }
                  
    oled.print(inputVal[i]); // text to display   
    oled.print("");    
         
    
  }                 
      
  oled.println(""); 
  oled.println(whiteDetected); // text to display
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

void pidArrowFollower(){
  //floorPattern();
  arrowReadLineSensors();
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
  int plus_speed = 130;
  int min_speed = 130;

  //base_speed = 80;  
  plus_speed = base_speed + pid;
  min_speed = base_speed - pid-10;

  

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

void stop(){
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



