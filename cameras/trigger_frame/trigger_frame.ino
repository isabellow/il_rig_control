// from SC: https://github.com/Selmaan/RigControl
// videoFrameTrigger

// set-up variables
const int topTrigPin = 2; // to send frame triggers to the side and top cameras
const int botTrigPin = 3; // to send frame triggers to the bottom camera
const unsigned int topTrigDelay = 10000; //half-delay in microseconds btw frames (10000 is 50hz)

// to run bottom camera at half speed
const int botTrigRateMod = 2; 
int botTrigModCnt = 0;

// initialize states and time
int topTrigState = LOW;
int botTrigState = LOW;
unsigned long previousMicros;

void setup() {
  // put your setup code here, to run once:
  pinMode(topTrigPin, OUTPUT);
  pinMode(botTrigPin, OUTPUT);
  digitalWrite(topTrigPin, topTrigState);
  digitalWrite(botTrigPin, botTrigState);
  Serial.begin(9600);
  Serial.println("Waiting to Begin...");
  boolean pulseOn = false;
  while (!pulseOn){
    pulseOn = Serial.read()>0;
    delay(1);
  }
  Serial.println("Sending Pulses");
  previousMicros = micros();
}

void loop() {
  // put your main code here, to run repeatedly:
  unsigned long currentMicros = micros(); // note rollover ~70m, should be fine though?

  if (currentMicros - previousMicros >= topTrigDelay) {
    previousMicros = currentMicros;
    topTrigState = !topTrigState; //invert top state
    digitalWrite(topTrigPin, topTrigState);

    botTrigModCnt = botTrigModCnt + 1; //advance bot frame subselector counter
    if (botTrigModCnt >= botTrigRateMod) {
      botTrigModCnt = 0; //reset subselector counter
      botTrigState = !botTrigState; //invert bot state
      digitalWrite(botTrigPin, botTrigState);
    }
  }
}
