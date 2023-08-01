#define NUM_SENSORS 12
#define NUM_VALUES 6


// Functions of Tower of Hanoi  |  th
void thRightShift_90();
void thLeftShift_90();

////////////////////////////// Tower of hanoi ///////////////////////////////////////////
  //Junction mapping
  int thJunc[3][5] = {{ 48,  8,  3,  100 },
                      { 32,  4,  2,  12  },
                      { 16,  0,  1,  8   }};     // 100 means finished  and 0 means No path

  int thCurrentJuncIndex = 0;   //Current junction   0: 12, 1: 32, 2: 16
  int thCurrentJunc[4] = {0, 0, 0, 0}; //Get a copy of current junction and do the shifting

  int thDirection = 0;
  //String thJuncType;
/////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////// Main controller variables ///////////////////////////////
  String junctionType = "None";
  bool junctionDetected = false; 
  String linePathPattern = "None"; 
  bool pathCrossing = false;
  int sensorValues[NUM_SENSORS][NUM_VALUES];  // For 12 line sensors to store 10 previous values
  int lineSensorCount[3] = {0, 0, 0} ;
   
////////////////////////////////////////////////////////////////////////////////////////////
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





////////////////////////////////////////////////////////////////////////////////////////
int inputVal[12] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  

void setup() {
  // put your setup code here, to run once:
      
      // initialize sensor values to 0
      for (int i = 0; i < NUM_SENSORS; i++) {
        for (int j = 0; j < NUM_VALUES; j++) {
          sensorValues[i][j] = 0;
        }
      }




}

void loop() {
  // put your main code here, to run repeatedly:


}


void thRightShift_90(){   //Right shift Junction mapping
          //copy original list to currrent junction list
          for(int j =0; j<4; j++){
            thCurrentJunc[j] = thJunc[thCurrentJuncIndex][j];
          }

          int temp = thCurrentJunc[-1];  //last element of current junction
          for (int i =1; i<4 ; i++){
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
          thCurrentJunc[-1]= temp;
}



void floorPattern(){

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
                  for(int j = 0; j<NUM_VALUES;  j++){
                      lineSensorCount[i] += sensorValues[4*i][j] + sensorValues[4*i+1][j] + sensorValues[4*i+2][j] + sensorValues[4*i+3][j];
                  }
            }
            //Detect patterns
            if(not pathCrossing){
                      if((lineSensorCount[0] >18) and (lineSensorCount[1] >18) and (lineSensorCount[2] >18)){
                              linePathPattern = "WhiteLine";
                              pathCrossing = true;                              
                      }
                      else if((lineSensorCount[0] <6 ) and (lineSensorCount[1] >6) and (lineSensorCount[2] < 6 )){
                              linePathPattern = "Line";
                              //pathCrossing = true;
                      }
                      else if((lineSensorCount[0] >18 ) and (lineSensorCount[1] >6) and (lineSensorCount[2] < 6 )){
                              linePathPattern = "HardLeft";
                              pathCrossing = true;
                      }
                      else if((lineSensorCount[0] <6 ) and (lineSensorCount[1] >6) and (lineSensorCount[2] >18 )){
                              linePathPattern = "HardRight";
                              pathCrossing = true;
                      }

            }
            else{
                      pathCrossing = false;
                      String prePathPattern = linePathPattern;
                      delay(100); // Change this

                      if((lineSensorCount[0] >18) and (lineSensorCount[1] >18) and (lineSensorCount[2] >18)){
                              linePathPattern = "WhiteLine";
                              //pathCrossing = true;                              
                      }
                      else if((lineSensorCount[0] <6 ) and (lineSensorCount[1] >6) and (lineSensorCount[2] < 6 )){
                              linePathPattern = "Line";
                              //junctionDetected = true;
                              //pathCrossing = true;
                      }
                      else if((lineSensorCount[0] <6 ) and (lineSensorCount[1] <6) and (lineSensorCount[2] < 6 )){
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

                      

            }

            


  }

/*
void floorPattern(){  //Follow the line until a junction or a white box
          int LeftLineSensors = inputVal[0] + inputVal[1] + inputVal[2] + inputVal[3];
          int CenterLineSensors = inputVal[4] + inputVal[5] + inputVal[6] + inputVal[7];
          int RightLineSensors = inputVal[8] + inputVal[9] + inputVal[10] + inputVal[11];
          bool pathCrossing = false;  //Cross path detected


          if(not pathCrossing){
                  if ((CenterLineSensors >= 3) and (LeftLineSensors <=1) and (RightLineSensors <=1)){    //Center line detected
                            linePathPattern = "WhiteLine";
                  }
                  else if ((CenterLineSensors <= 1) and (LeftLineSensors <=1) and (RightLineSensors <=1)){   //Black space detected
                            linePathPattern = "BlackLine"; 
                       
                  }
                  else if ((CenterLineSensors >= 3) and (LeftLineSensors >=3) and (RightLineSensors <=1)){   //Left hard turn detected
                            linePathPattern = "HardLeft"; 
                            pathCrossing = true; 
                  }
                  else if ((CenterLineSensors >= 3) and (LeftLineSensors <=1) and (RightLineSensors >=3)){   //Right hard turn detected
                            linePathPattern = "HardRight"; 
                            pathCrossing = true; 
                  }
                  else if ((CenterLineSensors >= 3) and (LeftLineSensors >=3) and (RightLineSensors >=3)){   //white space detected
                            linePathPattern = "HardLeftRight"; 
                            pathCrossing = true; 
                  }
          }
          else{
                 delay(500);  //Change this // Wait some time and get another reading
                 String preLinePattern = linePathPattern;
                 pathCrossing = false;
                  if ((CenterLineSensors >= 3) and (LeftLineSensors <=1) and (RightLineSensors <=1)){    //Center line detected
                            linePathPattern = "WhiteLine";
                  }
                  else if ((CenterLineSensors <= 1) and (LeftLineSensors <=1) and (RightLineSensors <=1)){   //Black space detected
                            linePathPattern = "BlackLine"; 
                       
                  }
                  else if ((CenterLineSensors >= 3) and (LeftLineSensors >=3) and (RightLineSensors >=1)){   //Left hard turn detected
                            linePathPattern = "HardLeftRight"; 
                            
                  }

                  //Identify Junction type

                  if((preLinePattern == "HardLeftRight") and (linePathPattern == "BlackLine")){                       //For tower of hanoi
                            thJuncType = "T_Junc";    
                  }
                  else if((preLinePattern == "HardLeftRight") and (linePathPattern == "WhiteLineLine")){             //For tower of hanoi
                            thJuncType = "WhiteBox";
                  }
                  else{
                            thJuncType = "None";                                                                     //For tower of hanoi
                  }

                 
                             
                  }                  
          }

          */
/*
void StraightLineFollower(){
          forward(150,150);     //Change this
}

void thAutomaticRouting(int num){      //Take the decision at junctions
          //Update floor mappings with respect to directions
          // num is the location to reach
          // Write speeds to motors to rotate
          //delay
          //thPathFinder()
          

}          

void thPathFinder(int num){    
          floorPattern();    //Update Floor pattern
          if(not junctionDetected){
                StraightLineFollower();
          }
          else{
                Stop();     //Stop the robot
                thAutomaticRouting(num);


          }
        

}

*/








