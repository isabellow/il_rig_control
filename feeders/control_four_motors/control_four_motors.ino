// From ELM https://github.com/emackev/paridae
// CircularTrack_fourmotors

// Declare pin functions on Redboard
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
const int betweenStepsMillis = 2 ;      // 12 (us) was 30 going to 50

void setup() {
  pinMode(stp1, OUTPUT);
  pinMode(dir1, OUTPUT);
  pinMode(stp2, OUTPUT);
  pinMode(dir2, OUTPUT);
  pinMode(stp3, OUTPUT);
  pinMode(dir3, OUTPUT);
  pinMode(stp4, OUTPUT);
  pinMode(dir4, OUTPUT);
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
      {Serial.println("Toggle all");
        toggle1();
        toggle2();
        toggle3();
        toggle4();
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

      else if (user_input==0) // close all
      {Serial.println("Close all");
        if (digitalRead(dir1)==LOW)
        {toggle1();}
        if (digitalRead(dir2)==LOW)
        {toggle2();}
        if (digitalRead(dir3)==LOW)
        {toggle3();}
        if (digitalRead(dir4)==LOW)
        {toggle4();}
      }
      
  }
}

// Reset Easy Driver pins to default states
void resetEDPins()
{
  digitalWrite(stp1, LOW);
  digitalWrite(dir1, LOW);
  digitalWrite(stp2, LOW);
  digitalWrite(dir2, LOW);
  digitalWrite(stp3, LOW);
  digitalWrite(dir3, LOW);
  digitalWrite(stp4, LOW);
  digitalWrite(dir4, LOW);
}

void toggle1()
{
  Serial.println("Toggle 1");
  digitalWrite(dir1, !digitalRead(dir1));
  for(x= 0; x<stepsPerRot; x++)  // Loop the forward stepping enough times for motion to be visible
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
