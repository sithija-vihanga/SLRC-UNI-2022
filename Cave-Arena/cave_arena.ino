#include <HCSR04.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
 
 #define SCREEN_WIDTH 128 // OLED display width,  in pixels
  #define SCREEN_HEIGHT 64 // OLED display height, in pixels
  #define NUM_SENSORS 12   //For floor pattern
  #define NUM_VALUES 6   //for floor pattern

    // declare an SSD1306 display object connected to I2C
  Adafruit_SSD1306 oled(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);
  int REFERENCE_DISTANCE=10;
  float distance1,distance3,distance4,distance5,distance6,error1,error2,errorD1,errorD,previous_errorD1,previous_errorD,integral1,integral,derivative1,derivative,error3,error4,PID_ErrorD1,PID_ErrorD2,PID_ErrorD;
  float ultrasonic_distance[7];
  int left_motor_speed,right_motor_speed,initial_speed=150,MAX_SPEED=225,MIN_SPEED=80;
  float ultrasonic_list1[5];
  int ultrasonic_count=0;
  int ultrasonic_part=0;
  
  // Motor A connections
  int enA = 2;
  int in1 = 39;    //5   
  int in2 = 37;   //4 
  // Motor B connections
  int enB = 3;
  int in3 = 43;     //7
  int in4 = 41;    //6

  float Kp = 1.6;
  float Kd = 0.3;
  float Ki = 0.0000;
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


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
//////////////////////////////////////////////////////////////////////////////////////////////
  //ultrasonic sensors
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
const int trigPin7 = 35;//distance7
const int echoPin7 = 34;
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
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
//  sort(tempInputValue_0,3);
//  sort(tempInputValue_1,3);
//  sort(tempInputValue_2,3);
//  sort(tempInputValue_3,3);
//  sort(tempInputValue_4,3);
//  sort(tempInputValue_5,3);
//  sort(tempInputValue_6,3);
//  sort(tempInputValue_7,3);
//  sort(tempInputValue_8,3);
//  sort(tempInputValue_9,3);
//  sort(tempInputValue_10,3);
//  sort(tempInputValue_11,3);

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
/////////////////mac angle////////////////////////////////////////////////
// int readMagAngle(){
//   compass.read();
// 	int MAngle = compass.getAzimuth();
// 	//delay(100);
//   return MAngle;

// }
///////////////////////Check Orientation///////////////////////////////////
// int thCheckOrientation(int mainDirection){
//     int thCurrentAngle =  readMagAngle();
//     int thAngleLimits[4];
//     //Initialize limits
//     thAngleLimits[0] = mainDirection + 45;
//     for(int i = 1; i<4; i++){
//       thAngleLimits[i] = thAngleLimits[i-1] + 90;
//     }
//     for(int k =0; k<4; k++){   //Keeps values betweeen 
//       if(thAngleLimits[k]>360){
//         thAngleLimits[k] = thAngleLimits[k] - 360;
//       }
//     }
//     //Check for the required sector
//     for(int j=1; j<4; j++){
//         if(thAngleLimits[j-1] < thAngleLimits[j]){   //No 360 jump
//             if((thAngleLimits[j-1]<=thCurrentAngle) and (thCurrentAngle<thAngleLimits[j])){
//                 return  j;
//             }
//         }
//         else{
          
//             if(((thAngleLimits[j-1]<=thCurrentAngle) and (thCurrentAngle<360) ) or ((thCurrentAngle<thAngleLimits[j]) and (thCurrentAngle>0))){
//                 return  j;
//             }

//         }
//     }
//     return 0; // if not in any sectors

// }
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

/////////////////////////////////////////////////////
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
   if ((ultrasonic_distance[3]>=8)||(ultrasonic_distance[3]==0)){
    ultrasonic_distance[3]=8;
    
    }
    /*
  Serial.print("distance 4:");
  Serial.print(ultrasonic_distance[3]);
  Serial.println();
  */
  ultrasonic_distance[4]=get_distance(trigPin5,echoPin5);
   if ((ultrasonic_distance[4]>=8)||(ultrasonic_distance[4]==0)){
    ultrasonic_distance[4]=8;
    
    }
    /*
  Serial.print("distance 5:");
  Serial.print(ultrasonic_distance[4]);
  Serial.println();
  */
  
  ultrasonic_distance[5]=get_distance(trigPin6,echoPin6);
   if ((ultrasonic_distance[5]>=100)||(ultrasonic_distance[5]==0)){
    ultrasonic_distance[5]=100;
    
    }
    /*
  Serial.print("distance 6:");
  Serial.print(ultrasonic_distance[5]);
  Serial.println();
  */
  
  ultrasonic_distance[6]=get_distance(trigPin7,echoPin7);
   if ((ultrasonic_distance[6]>=100)||(ultrasonic_distance[6]==0)){
    ultrasonic_distance[6]=100;
    
    }
    /*
  Serial.print("distance 7:");
  Serial.print(ultrasonic_distance[6]);
  Serial.println();
  */
 
  
  
}

/////////////////////////////////////////////////////

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  if (!oled.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
          Serial.println(F("SSD1306 allocation failed"));
          while (true);
  }
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

           // wait for initializing
  oled.clearDisplay(); // clear display

  oled.setTextSize(1);          // text size
  oled.setTextColor(WHITE);     // text color
  oled.setCursor(0, 10);        // position to display
  oled.println("TEAM SPECTRO"); // text to display
  oled.display();               // show on OLED

}

void loop() {
  // put your main code here, to run repeatedly:
 
  Stop();
  delay(2000);
  ultrasonic_array();
  while(ultrasonic_distance[0]>=24){
    ultrasonic_array();
    forward(150,150);
  }
  
  
  HCSR04 hc_left_2(trigPin6,echoPin6);
  distance5=hc_left_2.dist();
  HCSR04 hc_right_2(trigPin7,echoPin7);
  distance6=hc_right_2.dist();
  
  if((distance5==100)||(distance5==0)){
    distance5=100;
  }
  if((distance6==100)||(distance6==0)){
    distance6=100;
  }
   
  
  if (distance5<distance6){
    turnRight(175);
    delay(500);
  }
  if (distance5>distance6){
    turnLeft(175);
    delay(500);
  }
  while (1!=0){
    readLineSensors();
    if((inputVal[0]==1)||(inputVal[11]==1)){
      ultrasonic_part=1;
      break;

    }
    
  ultrasonic_array();
  
 
  
  
   if (ultrasonic_distance[0]<=25){
        Stop();
        delay(600);
      if((ultrasonic_distance[3]==8)&&(ultrasonic_distance[4]==8)){
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
        forward(150,150);
        
        }
}
      
    
else if((ultrasonic_distance[1]<5)||(ultrasonic_distance[2]<5)){
        reverse(150,150);
      }
     else if ((ultrasonic_distance[3])<(ultrasonic_distance[4])){
       turnRight(150);
       delay(10);
     }

       else if ((ultrasonic_distance[3])>(ultrasonic_distance[4])){
       turnLeft(150);
       delay(10);
       }

    else if (ultrasonic_distance[1]>ultrasonic_distance[2]){
         turnLeft(150);
         delay(10);
         
         

        

        
        




     }
    
        else if (ultrasonic_distance[1]<ultrasonic_distance[2]){
        turnRight(150);
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

      forward(150,150);
      
    }

//    
//    oled.clearDisplay();
//  oled.setTextSize(1);      
//  oled.setTextColor(WHITE);
//  oled.setCursor(0, 10);   
//  oled.println("Team Spectro");
//  oled.setCursor(0, 30);    
//  oled.print(distance3);
//  oled.print("  ");
//  oled.print(distance4);
//   oled.print("  ");
//   oled.print(ultrasonic_distance[0]);
//   oled.print("  ");
//   oled.print(ultrasonic_distance[1]);
//   oled.print("  ");
//   oled.print(ultrasonic_distance[2]);
//   oled.print("  ");
//   oled.print(ultrasonic_distance[3]);
//   oled.print("  ");
//   oled.print(ultrasonic_distance[4]);
//   oled.print("  ");
//   oled.print(ultrasonic_distance[5]);
//   oled.print("  ");
//   oled.print(ultrasonic_distance[6]);
//   oled.print("  ");
// 
// 
//   
// 
//   
//   oled.display();
   
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
    
    // Stop();
    // delay(3000);
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
        delay(10);
    }
    if(ultrasonic_distance[1]>ultrasonic_distance[2]){
        turnLeft(175);
        delay(10);
    }
      // forward(150,130);
      if(distance5<distance6){
        errorD=(distance6-distance5);
        derivative=errorD-previous_errorD;
        integral+=errorD;
        PID_ErrorD=(Kp*errorD+Kd*errorD+Ki*errorD);
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
        PID_ErrorD1=(Kp*errorD1+Kd*errorD1+Ki*errorD1);
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
      if((distance5>=50)||(distance6>=50)){
        Stop();
        delay(500);
        ultrasonic_part=3;
        break;
       
        
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
      readLineSensors();
      Stop();
      delay(1000);
      if((inputVal[0]==1)||(inputVal[1]==1)||(inputVal[2]==1)||(inputVal[3]==1)||(inputVal[4]==1)||(inputVal[5]==1)||(inputVal[6]==1)||(inputVal[7]==1)||(inputVal[8]==1)||(inputVal[9]==1)||(inputVal[10]==1)||(inputVal[11]==1)){
        Stop();
        delay(1000);
        break;
      }

    }
    delay(10000);
    
    

        
              
           
           
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
}



void reverse(int lspeed,int rspeed){
  analogWrite(enA,lspeed);
  analogWrite(enB,rspeed);
  digitalWrite(in1,LOW);
  digitalWrite(in2,HIGH);
  digitalWrite(in3,LOW);
  digitalWrite(in4,HIGH);
  
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



void forward(int lSpeed,int rSpeed){
  analogWrite(enA, lSpeed);
  analogWrite(enB, rSpeed);
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW) ;
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW) ;
}






