////////////////////////////////pins//////////////////////////////
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

void setup()
{
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
  Serial.begin(9600);
}
void loop()
{
  String colorBox = FindColorBox();////////////////////////////check color//////////////////////
  //String colorFloor = FindColorFloor();////////////////////////////check color//////////////////////
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
//////////////////////////////////////////////////////////////////////////////////////
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
  if ((b < 1700))
  {
    Serial.println("F Colour Blue");
    color = "Colour Blue";
  }
  else if ((b > 2000)&&(g <2200))
  {
    Serial.println("F Colour Green");
    color = "Colour Green";
  }
  else if (r < 2800)
  {
    Serial.println("F Colour Red");
    color = "Colour Red";
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
