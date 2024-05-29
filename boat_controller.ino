#include <sbus.h>

const unsigned int SERIAL_RATE = 115200;
const unsigned char RX_S2 = 16;
const unsigned char TX_S2 = 17;

const unsigned int PWM_FREQ = 50;     // 50Hz
const unsigned char PWM_RESOLUTION = 16; // ESP32 can go up to 16 bits 
const unsigned long PWM_CYCLE_TICKS = (unsigned long)pow(2, PWM_RESOLUTION); //Total of ticks 
const unsigned int MAX_DUTY_CYCLE = PWM_CYCLE_TICKS - 1;  //Max tick to write
const unsigned long PWM_CYCLE_USEC = (unsigned long)(1E6/PWM_FREQ); // Total time to complete a cycle in microseconds
const unsigned int DELAY_MS = (unsigned int)(2E3/PWM_FREQ); //40 ms

struct rcScaler {
  const unsigned short minUS;
  const unsigned short maxUS;
  const unsigned short trimUS;
  const unsigned short minScaledUS;
  const unsigned short maxScaledUS;
  const unsigned short trimScaledUS;
  const unsigned char deadzone; //Calculated value of deadzone percentaje to filter inputs values of Trim.
  unsigned short pulseUS;
  unsigned short scaledPulseUS;
  unsigned short reversedPulseUS;
};

struct escWriter {
  const unsigned char pin;
  unsigned int usToTicks;
  bool reversed;
};

bfs::SbusRx RC_RX(&Serial2, RX_S2, TX_S2, true);
bfs::SbusData RCData;

rcScaler Throttle = {200,1800,1000, 1100, 1900, 1500, 40, 0, 0, 0};
rcScaler Steering = {200,1800,1000, 1100, 1900, 1500, 40, 0, 0, 0};
rcScaler AutoThrottle = {200,1800,1000, 1100, 1900, 1500, 40, 0, 0, 0};

escWriter CCW = {12,  0, false};
escWriter CW = {14, 0, true};

void setup() {
  Serial.begin(SERIAL_RATE);
  while (!Serial) {}
  RC_RX.Begin();

  ledcAttach(CCW.pin, PWM_FREQ, PWM_RESOLUTION);
  ledcAttach(CW.pin, PWM_FREQ, PWM_RESOLUTION);
}

void loop() {
  if (RC_RX.Read()) {
    RCData = RC_RX.data();

    Throttle.pulseUS = RCData.ch[2];
    Steering.pulseUS = RCData.ch[0];
    AutoThrottle.pulseUS = RCData.ch[6];
  }
  scalePulse(&AutoThrottle);
  scalePulse(&Throttle);
  scalePulse(&Steering);

  if(AutoThrottle.scaledPulseUS != AutoThrottle.trimScaledUS) {
    if (CCW.reversed) { 
      updateTicks(&CCW, AutoThrottle.scaledPulseUS);
    } else { //CCW is reversed by default in the steering control
      updateTicks(&CCW, AutoThrottle.reversedPulseUS);
    }
    if (CW.reversed) {
      updateTicks(&CW, AutoThrottle.reversedPulseUS);
    } else {  //CW is by default in the positive control
      updateTicks(&CW, AutoThrottle.scaledPulseUS);
    }
  }
  else {
    if (CCW.reversed) { 
      updateTicks(&CCW, (Throttle.scaledPulseUS + Steering.scaledPulseUS)/2);
    } else {  //CCW is reversed by default in the steering control
      updateTicks(&CCW, (Throttle.reversedPulseUS + Steering.reversedPulseUS)/2);
    }
    if (CW.reversed) {
      updateTicks(&CW, (Throttle.reversedPulseUS + Steering.reversedPulseUS)/2);
    } else {  //CW is by default in the positive control
      updateTicks(&CW, (Throttle.scaledPulseUS + Steering.scaledPulseUS)/2);
    }
  }
  writeToMotor(&CCW);
  writeToMotor(&CW);

  Serial.print(AutoThrottle.pulseUS);
  Serial.print(",");
  Serial.print(Throttle.pulseUS);
  Serial.print(",");
  Serial.print(Steering.pulseUS);
  Serial.print(",");
  Serial.print((CCW.usToTicks*float(PWM_CYCLE_USEC))/float(PWM_CYCLE_TICKS));
  Serial.print(",");
  Serial.println((CW.usToTicks*float(PWM_CYCLE_USEC))/float(PWM_CYCLE_TICKS));
}

void scalePulse(rcScaler* rc) {
  if (rc->pulseUS < rc->minUS || rc->pulseUS > rc->maxUS || abs(rc->trimUS - rc->pulseUS) < rc->deadzone) {
    rc->scaledPulseUS = rc->trimScaledUS;
    rc->reversedPulseUS = rc->trimScaledUS;
  } else {
    if (rc->pulseUS < rc->trimUS) {
      rc->scaledPulseUS = map(rc->pulseUS, rc->minUS, rc->trimUS, rc->minScaledUS, rc->trimScaledUS);
      rc->reversedPulseUS = map(rc->pulseUS, rc->minUS, rc->trimUS, rc->maxScaledUS, rc->trimScaledUS);
    } else {
      rc->scaledPulseUS = map(rc->pulseUS, rc->trimUS, rc->maxUS, rc->trimScaledUS, rc->maxScaledUS);
      rc->reversedPulseUS = map(rc->pulseUS, rc->trimUS, rc->maxUS, rc->trimScaledUS, rc->minScaledUS);
    }
  }
}

void updateTicks(escWriter* motor, int value) {
  motor->usToTicks = (unsigned int)((float(value)/float(PWM_CYCLE_USEC))*float(PWM_CYCLE_TICKS));
}

void writeToMotor(escWriter* motor) {
  ledcWrite(motor->pin, motor->usToTicks);
  delay(DELAY_MS);
}
