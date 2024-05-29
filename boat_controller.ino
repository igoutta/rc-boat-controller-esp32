#include <Arduino.h>

const unsigned int SerialRate = 115200;

const unsigned int PWM_FREQ = 50;     // 50Hz
const unsigned char PWM_RESOLUTION = 16; // ESP32 can go up to 16 bits 
const unsigned long PWM_CYCLE_TICKS = (unsigned long)pow(2, PWM_RESOLUTION); //Total of ticks 
const unsigned int MAX_DUTY_CYCLE = PWM_CYCLE_TICKS - 1;  //Max tick to write
const unsigned long PWM_CYCLE_USEC = (unsigned long)(1E6/PWM_FREQ); // Total time to complete a cycle in microseconds
const unsigned int DELAY_MS = (unsigned int)(2E3/PWM_FREQ); //40 ms

struct RCScaler {
  const uint8_t PIN;
  const unsigned short MinUS;
  const unsigned short MaxUS;
  const unsigned short TrimUS;
  const unsigned short MinScaledUS;
  const unsigned short MaxScaledUS;
  const unsigned short TrimScaledUS;
  const uint8_t Deadband; //Calculated value of deadzone percentaje to filter inputs vallues and output safe PWM values  
  volatile unsigned long StartTime;
  volatile unsigned long CurrentTime;
  unsigned int ScaledPulse;
};

struct Servo {
  const uint8_t PIN;
  const uint8_t PWM_CHANNEL;    // ESP32 has 16 channels which can generate 16 independent waveforms
  unsigned int usToTicks;
};

RCScaler RC_CCW = {32, 1015, 2000, 1500, 1100, 1900, 1500, 25, 0, 0, 0};
RCScaler RC_CW = {33, 1015, 2000, 1500, 1900, 1100, 1500, 25, 0, 0, 0};
RCScaler THREE_SETS = {25, 1000, 2000, 1500, 1100, 1900, 1500, 5, 0, 0, 0};
Servo CCW = {12, 1, 0};
Servo CW = {14, 0, 0};

void setup() {
  Serial.begin(SerialRate);

  pinMode(RC_CCW.PIN, INPUT_PULLUP);
  attachInterruptArg(RC_CCW.PIN, readPulse, &RC_CCW, CHANGE);
  pinMode(RC_CW.PIN, INPUT_PULLUP);
  attachInterruptArg(RC_CW.PIN, readPulse, &RC_CW, CHANGE);
  pinMode(THREE_SETS.PIN, INPUT_PULLUP);
  attachInterruptArg(THREE_SETS.PIN, readPulse, &THREE_SETS, CHANGE);

  ledcSetup(CW.PWM_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(CW.PIN,CW.PWM_CHANNEL);
  ledcSetup(CCW.PWM_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(CCW.PIN,CCW.PWM_CHANNEL);
}

void loop() {
  scalePulse(&RC_CCW);
  scalePulse(&RC_CW);
  scalePulse(&THREE_SETS);
  Serial.print(RC_CCW.CurrentTime);
  Serial.print(",");
  Serial.print(RC_CW.CurrentTime);
  Serial.print(",");
  Serial.print(THREE_SETS.CurrentTime);
  Serial.print(",");
  if(THREE_SETS.ScaledPulse != THREE_SETS.TrimScaledUS) {
    updateTicks(&CCW, THREE_SETS.ScaledPulse);
    updateTicksReversed(&CW, THREE_SETS.ScaledPulse);
  }
  else {
    updateTicks(&CCW, RC_CCW.ScaledPulse);
    updateTicks(&CW, RC_CW.ScaledPulse);
  }
  Serial.print((CCW.usToTicks*float(PWM_CYCLE_USEC))/float(PWM_CYCLE_TICKS));
  Serial.print(",");
  Serial.println((CW.usToTicks*float(PWM_CYCLE_USEC))/float(PWM_CYCLE_TICKS));
  writeToMotor(&CCW);
  writeToMotor(&CW);
}

void ARDUINO_ISR_ATTR readPulse(void* arg) {
  RCScaler* rc = static_cast<RCScaler*>(arg);
  if (digitalRead(rc->PIN) == HIGH) {
    rc->StartTime = micros();
  } else {
    rc->CurrentTime = micros() - rc->StartTime;
  }
}

void scalePulse(RCScaler* rc) {
  noInterrupts();
  if (rc->CurrentTime < (rc->MinUS - rc->Deadband) || rc->CurrentTime > (rc->MaxUS + rc->Deadband)) {
    rc->ScaledPulse = rc->TrimScaledUS;
  } else {
    if (abs(rc->TrimUS- (int)rc->CurrentTime) < rc->Deadband) {
      rc->ScaledPulse = rc->TrimScaledUS;
    } else {
      if (rc->CurrentTime < rc->TrimUS) {
        rc->ScaledPulse = map(rc->CurrentTime, rc->MinUS, rc->TrimUS, rc->MinScaledUS + rc->Deadband, rc->TrimScaledUS);
      } else {
        rc->ScaledPulse = map(rc->CurrentTime, rc->TrimUS, rc->MaxUS, rc->TrimScaledUS, rc->MaxScaledUS - rc->Deadband);
      }
    }
  }
  interrupts();
}

void updateTicks(Servo* servo, int value) {
  servo->usToTicks = (unsigned int)((float(value)/float(PWM_CYCLE_USEC))*float(PWM_CYCLE_TICKS));
}

void updateTicksReversed(Servo* servo, int value) {
  int reversed_value = 1500 - (value - 1500); //Trim-range
  servo->usToTicks = (unsigned int)((float(reversed_value)/float(PWM_CYCLE_USEC))*float(PWM_CYCLE_TICKS));
}


void writeToMotor(Servo* servo) {
  ledcWrite(servo->PWM_CHANNEL, servo->usToTicks);
  delay(DELAY_MS);
}