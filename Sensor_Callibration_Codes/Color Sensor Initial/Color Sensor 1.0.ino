// TCS3200 color sensor pins
const int S0 = 2;
const int S1 = 3;
const int S2 = 4;
const int S3 = 5;
const int OUT = 6;

// Set frequency scaling factor for color sensor
// (2 = 2%, 20 = 20%, 100 = 100%, and 1000 = 1000%)
int frequencyScalingFactor = 2;

void setup() {
  // Initialize color sensor pins as outputs
  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);
  pinMode(OUT, INPUT);

  // Set frequency scaling factor for color sensor
  switch (frequencyScalingFactor) {
    case 2:
      digitalWrite(S0, LOW);
      digitalWrite(S1, HIGH);
      break;
    case 20:
      digitalWrite(S0, HIGH);
      digitalWrite(S1, LOW);
      break;
    case 100:
      digitalWrite(S0, LOW);
      digitalWrite(S1, LOW);
      break;
    case 1000:
      digitalWrite(S0, HIGH);
      digitalWrite(S1, HIGH);
      break;
    default:
      // Invalid scaling factor, default to 2%
      digitalWrite(S0, LOW);
      digitalWrite(S1, HIGH);
  }
  
  // Enable color sensor
  digitalWrite(S2, LOW);
  digitalWrite(S3, LOW);
}

void loop() {
  // Read raw color values from sensor
  int redValue = pulseIn(OUT, digitalRead(OUT) == HIGH ? LOW : HIGH);
  int greenValue = pulseIn(OUT, digitalRead(OUT) == HIGH ? LOW : HIGH);
  int blueValue = pulseIn(OUT, digitalRead(OUT) == HIGH ? LOW : HIGH);

  // Convert raw color values to RGB values
  float red = (255.0 / redValue);
  float green = (255.0 / greenValue);
  float blue = (255.0 / blueValue);

  // Print RGB values to serial monitor
  Serial.print("Red: ");
  Serial.print(red);
  Serial.print(", Green: ");
  Serial.print(green);
  Serial.print(", Blue: ");
  Serial.println(blue);
  
  // Wait for a short time before reading again
  delay(100);
}
