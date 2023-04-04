// Declare pin functions on Arduino
#define stp4 9
#define dir4 8
#define stp3 7
#define dir3 6
#define stp2 5
#define dir2 4
#define stp1 3
#define dir1 2

// Declare variables for functions
int motor_state;
int user_input;
int x;
int stepsPerRot = 4200;                 // how far to turn for each toggle
const int pulseWidthMicros = 30;        // 30 (us)
const int betweenStepsMillis = 2 ;      // pause time

void setup() {
  // pins 2-9 should be output
  for(x=2; x<=9; x++)
  {
    pinMode(x, OUTPUT);
  }
  resetEDPins(); // Set step, direction, microstep and enable pins to default states
  Serial.begin(9600); // Open Serial connection for debugging
  Serial.println("Begin motor control");
  Serial.println();
}

// Main loop
void loop() {
  while(Serial.available()){
      user_input = Serial.read(); // Read user input and trigger appropriate function
      user_input = user_input-48; // Convert '1','2','3' characters to int format
      if (user_input==5) // toggle all
      {
        toggle_all();
      }
      else if (user_input==1)
      {
        toggle1();
      }
      else if (user_input==2)
      {
        toggle2();
      }
      else if (user_input==3)
      {
        toggle3();
      }
      else if (user_input==4)
      {
        toggle4();
      }
  }
}

// Reset Easy Driver pins to default states
void resetEDPins()
{
  digitalWrite(stp1, LOW); // default motor direction
  digitalWrite(dir1, LOW);
  digitalWrite(stp2, LOW);
  digitalWrite(dir2, LOW);
  digitalWrite(stp3, LOW); // prepare step input to accept next rising edge
  digitalWrite(dir3, LOW);
  digitalWrite(stp4, LOW);
  digitalWrite(dir4, LOW);
}

// Toggle feeders
void toggle_all()
{
  Serial.println("Toggle all");
  digitalWrite(dir1, !digitalRead(dir1));
  digitalWrite(dir2, !digitalRead(dir1));
  digitalWrite(dir3, !digitalRead(dir1));
  digitalWrite(dir4, !digitalRead(dir1));
  delay(1);
  for(x=0; x<stepsPerRot; x++)  // Step until toggle is complete
  {
    digitalWrite(stp1,HIGH); //Trigger one step
    digitalWrite(stp2,HIGH);
    digitalWrite(stp3,HIGH);
    digitalWrite(stp4,HIGH);
    delay(betweenStepsMillis);
    digitalWrite(stp1,LOW); //Pull step pin low so it can be triggered again
    digitalWrite(stp2,LOW);
    digitalWrite(stp3,LOW);
    digitalWrite(stp4,LOW);
    delay(betweenStepsMillis);
  }
}

void toggle1()
{
  Serial.println("Toggle 1");
  digitalWrite(dir1, !digitalRead(dir1));
  for(x=0; x<stepsPerRot; x++)  // Loop the forward stepping enough times for motion to be visible
  {
    digitalWrite(stp1,HIGH); // Trigger one step forward
    delayMicroseconds(pulseWidthMicros);
    digitalWrite(stp1,LOW); // Pull step pin low so it can be triggered again
    delay(betweenStepsMillis);
  }
}

void toggle2()
{
  Serial.println("Toggle 2");
  digitalWrite(dir2, !digitalRead(dir2));
  for(x= 0; x<stepsPerRot; x++)  // Loop the forward stepping enough times for motion to be visible
  {
    digitalWrite(stp2,HIGH); // Trigger one step forward
    delayMicroseconds(pulseWidthMicros);
    digitalWrite(stp2,LOW); // Pull step pin low so it can be triggered again
    delay(betweenStepsMillis);
  }
}

void toggle3()
{
  Serial.println("Toggle 3");
  digitalWrite(dir3, !digitalRead(dir3));
  for(x= 0; x<stepsPerRot; x++)  // Loop the forward stepping enough times for motion to be visible
  {
    digitalWrite(stp3,HIGH); // Trigger one step forward
    delayMicroseconds(pulseWidthMicros);
    digitalWrite(stp3,LOW); // Pull step pin low so it can be triggered again
    delay(betweenStepsMillis);
  }
}

void toggle4()
{
  Serial.println("Toggle 4");
  digitalWrite(dir4, !digitalRead(dir4));
  for(x= 0; x<stepsPerRot; x++)  // Loop the forward stepping enough times for motion to be visible
  {
    digitalWrite(stp4,HIGH); // Trigger one step forward
    delayMicroseconds(pulseWidthMicros);
    digitalWrite(stp4,LOW); // Pull step pin low so it can be triggered again
    delay(betweenStepsMillis);
  }
}
