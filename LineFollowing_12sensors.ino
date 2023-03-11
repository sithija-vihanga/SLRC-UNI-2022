#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128 // OLED display width,  in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

// declare an SSD1306 display object connected to I2C
Adafruit_SSD1306 oled(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);


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

  delay(2000);         // wait for initializing
  oled.clearDisplay(); // clear display

  oled.setTextSize(1);          // text size
  oled.setTextColor(WHITE);     // text color
  oled.setCursor(0, 10);        // position to display
  oled.println("TEAM SPECTRO"); // text to display
  oled.display();               // show on OLED

  int count = 0;
  }

int error_list[5] = {0,0,0,0,0};
float kp = 0.6;
float kd = 0.;
float ki = 0.0000;

int error = 0;
int d_error = 0;
int  i_error = 0;

void loop(){ 

  
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

  for (int i = 0; i<12; i++){
          if(inputVal[i]<200){
                inputVal[i] = 1;            
          }
          else{
                inputVal[i] = 0;
          }
          Serial.print(inputVal[i]);
          Serial.print(" ");

  }
  Serial.println();
    
 

  
  error = 10*(inputVal[0]*6 +inputVal[1]*5 +inputVal[2]*4 + inputVal[3]*3 +inputVal[4]*2 + inputVal[5] -(inputVal[11]*6 +inputVal[10]*5 +inputVal[9]*4 + inputVal[8]*3 +inputVal[7]*2 + inputVal[6]) ) ; //(2*(inputVal[0]+inputVal[1]+inputVal[2]+inputVal[3]+inputVal[4]+inputVal[5]+inputVal[6]));

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
  for(int j=0; j<12; j++){
        oled.print(inputVal[j]);
  }    
  
  
  oled.display();

int base_speed = 80;  
 int plus_speed = 80;
 int min_speed = 80;





  base_speed = 80;  
  plus_speed = base_speed + pid;
  min_speed = base_speed - pid;


  if(pid>100){
    turnRight();
    delay(250);
  }
  else if(pid<-100){
    turnLeft();
    delay(250);
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
  forward(plus_speed,min_speed);
  
      


}

void forward(int lSpeed,int rSpeed){
  analogWrite(enA, lSpeed);
  analogWrite(enB, rSpeed);
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW) ;
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW) ;
}

void turnLeft(){
  analogWrite(enA, 150);
  analogWrite(enB, 150);
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW) ;
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH) ;  
}

void turnRight(){
  analogWrite(enA, 150);
  analogWrite(enB, 150);
   digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH) ;
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW) ;  
}