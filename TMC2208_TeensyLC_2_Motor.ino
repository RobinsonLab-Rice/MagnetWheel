

// Define pins
#define EN_PIN_A    3  // LOW: Driver enabled. HIGH: Driver disabled
#define STEP_PIN_A  2  // Step on rising edge
#define DIR_PIN_A   4  // Pin state switches direction

#define EN_PIN_B    12  // LOW: Driver enabled. HIGH: Driver disabled
#define STEP_PIN_B  5  // Step on rising edge
#define DIR_PIN_B   6  // Pin state switches direction

#define TRIGGER_PIN 14 // Triggers the same functionality as sending the "spin" commend when voltage goes HIGH
#define FRAME_PIN	12

// The TMC2208 has an internal clock of ~12 MHz.
// Max Full Steps per Second: Clk / 512 ~= 23,437.5
#define MAX_STEP_RATE 23437.5

#include <AccelStepper.h>
#include <MultiStepper.h>

#include <ADC.h>

const int SPIN_DLEAY_FROM_FRAME_TRIGGER = 30;  // The number of seconds to wait after recieving a frame trigger
	//before starting the wheel spinning
unsigned long frameDelayMillis;
unsigned long firstFrameTrigger;
bool waitForFrameTrigger = false;
bool frameTriggerDelay = false;

const int readPin = A9;
const int ZERO_FIELD = 2047; // ADC values will be 12-bit, null field is the middle value (4096 / 2) - 1
ADC *adc = new ADC();

volatile int lastHallRead;
volatile int hallMax;
volatile int hallMin;

volatile bool frameTrigger = false;

#include <TMC2208Stepper.h>                       // Include library
TMC2208Stepper driverA = TMC2208Stepper(&Serial1);  // Create driver and use
TMC2208Stepper driverB = TMC2208Stepper(&Serial3);  // Create driver and use

AccelStepper accelA = AccelStepper(AccelStepper::DRIVER, STEP_PIN_A, DIR_PIN_A);
AccelStepper accelB = AccelStepper(AccelStepper::DRIVER, STEP_PIN_B, DIR_PIN_B);

class HybridStepper
{
  uint8_t microstep;
  unsigned int fullStepsPerRotation;
  unsigned int totalStepsPerRotation;
  double HertzFactor;
  double HertzSpeed;
  double HertzAccel;
  uint8_t stepPin;
  uint8_t dirPin;
  uint8_t enPin;

  public:
  TMC2208Stepper * driver;
  AccelStepper * accel;
  bool accelerating;
  bool stopping;
  bool stepping;
  double stepSpeed;
  double acceleration;
  uint16_t accelCurrent;
  uint16_t runCurrent;
  bool reduceCurrent;
  
  HybridStepper(int step_pin, int dir_pin, int en_pin, TMC2208Stepper *d, AccelStepper *a)    {
    microstep = 4;
    fullStepsPerRotation = 200;
    totalStepsPerRotation = 800;
    HertzFactor = 0.00125;
    HertzSpeed = 0.5;
    HertzAccel = 1.0;
    stepPin = step_pin;
    dirPin = dir_pin;
    enPin = en_pin;
    driver = d;
    accel = a;
    stepSpeed = 400.0;
    acceleration = 800.0;
    stepping = false;
    accelerating = false;
    stopping = false;
    accelCurrent = 1400;
    runCurrent = 700;
    reduceCurrent = false;
  }

  void setup()    {
    driver->push();
    driver->pdn_disable(true);     // Use PDN/UART pin for communication
    driver->I_scale_analog(false); // Use internal voltage reference
    driver->rms_current(accelCurrent);      // Set driver current 500mA
    driver->mstep_reg_select(true); // User MSTEP register for microstep setting
    driver->microsteps(microstep);
    driver->toff(2);               // Enable driver in software

    accel->setMaxSpeed(stepSpeed);
    accel->setSpeed(stepSpeed);
    accel->setAcceleration(acceleration);
  }

  void start()  {
    driver->rms_current(accelCurrent);
    accel->move(acceleration*2);
    stepping = true;
    accelerating = true;
    stopping = false;
  }

  void stop() {
    if (reduceCurrent)  {
      driver->rms_current(accelCurrent);
    }
    accel->stop();
    stopping = true;
    accelerating = false;
    Serial.println("Stopping");
  }

  void halt() {
    stepping = false;
    enable(0);
  }

  bool run()    {
    if (stepping)  {
      if (!accel->isRunning())  {
        Serial.println("Stopped");
        stepping = false;
        stopping = false;
        accelerating = false;
        driver->rms_current(accelCurrent);
      }
      if (accelerating) {
        if (accel->distanceToGo() <= acceleration)  {
          accelerating = false; 
          // THE FOLLOWING LINES ARE FOR TESTING REDUCED MOTOR CURRENT AFTER ACCELERATION
          if (reduceCurrent)  {
            driver->rms_current(runCurrent);
          }
        }
        return accel->run();
      }
      else if (stopping)  {
        return accel->run();
      }
      else {
        return accel->runSpeed();
      }
    }
    return false;
  }

  void enable(byte val)   {
    if (val == 0) {
      digitalWriteFast(enPin, HIGH);
      Serial.println("Disabled");
    }
    else  {
      digitalWriteFast(enPin, LOW);
      Serial.println("Enabled");
    }
  }

  void direction(byte val)    {
    if (val == 0) {
      driver->shaft(false);
      Serial.println("Direction 0");
    }
    else  {
      driver->shaft(true);
      Serial.println("Direction 1");
    }
  }

  void setMicros(byte val)    {
    microstep = val;
    totalStepsPerRotation = fullStepsPerRotation * microstep;
    HertzFactor = 1.0 / totalStepsPerRotation;
    stepSpeed = HertzSpeed * totalStepsPerRotation;
    if (stepSpeed > MAX_STEP_RATE)  {
      stepSpeed = MAX_STEP_RATE;
      HertzSpeed = MAX_STEP_RATE / totalStepsPerRotation;
    }
    acceleration = HertzAccel * totalStepsPerRotation;
    driver->microsteps(val);
    accel->setSpeed(stepSpeed);
    accel->setMaxSpeed(stepSpeed);
    accel->setAcceleration(acceleration);
  }

  void setAccelCurrent(int val) {
    accelCurrent = val;
    if (!stepping || accelerating)  {
      driver->rms_current(val);
    }
  }

  void setRunCurrent(int val) {
    runCurrent = val;
    if (!accelerating || !stopping)  {
      driver->rms_current(val);
    }
  }

  void setSpeed(double val)    {
    HertzSpeed = val;
    stepSpeed = HertzSpeed * totalStepsPerRotation;
    if (stepSpeed > MAX_STEP_RATE)  {
      stepSpeed = MAX_STEP_RATE;
      HertzSpeed = MAX_STEP_RATE / totalStepsPerRotation;
    }
    accel->setSpeed(stepSpeed);
    accel->setMaxSpeed(stepSpeed);
    Serial.println(stepSpeed/totalStepsPerRotation);
  }

  void setAcceleration(double val)   {
    HertzAccel = val;
    acceleration = HertzAccel * totalStepsPerRotation;
    accel->setAcceleration(acceleration);
    Serial.println(acceleration);
  }

  void setStepperSteps(unsigned int val) {
    fullStepsPerRotation = val;
    totalStepsPerRotation = fullStepsPerRotation * microstep;
    HertzFactor = 1.0 / totalStepsPerRotation;
    stepSpeed = HertzSpeed * totalStepsPerRotation;
    if (stepSpeed > MAX_STEP_RATE)  {
      stepSpeed = MAX_STEP_RATE;
      HertzSpeed = MAX_STEP_RATE / totalStepsPerRotation;
    }
    acceleration = HertzAccel * totalStepsPerRotation;
    accel->setSpeed(stepSpeed);
    accel->setMaxSpeed(stepSpeed);
    accel->setAcceleration(acceleration);
    Serial.println(val);
  }

  void toggleReduceCurrent()  {
    if (reduceCurrent)  {
      reduceCurrent = false;
    }
    else  {
      reduceCurrent = true;
    }
    Serial.println(reduceCurrent);
  }

  void reset()  {
    driver->toff(0);
    driver->push();
    driver->pdn_disable(true);     // Use PDN/UART pin for communication
    driver->I_scale_analog(false); // Use internal voltage reference
    driver->rms_current(accelCurrent);      // Set driver current 500mA
    driver->mstep_reg_select(true); // User MSTEP register for microstep setting
    driver->microsteps(microstep);
    driver->toff(2);               // Enable driver in software
  }
};

HybridStepper drivers[] = {
  HybridStepper(STEP_PIN_A, DIR_PIN_A, EN_PIN_A, &driverA, &accelA),
  HybridStepper(STEP_PIN_B, DIR_PIN_B, EN_PIN_B, &driverB, &accelB)
};

uint8_t selectedDriver = 0;

bool reduceCurrent = false;

uint16_t counter = 0;
uint32_t data = 0;

unsigned long loopMillis = 0ul;
unsigned long blinkTime = 0ul;

String message = "";

void setup() {
  pinMode(13,OUTPUT);

  // Prepare pins
  pinMode(EN_PIN_A, OUTPUT);
  pinMode(STEP_PIN_A, OUTPUT);
  pinMode(DIR_PIN_A, OUTPUT);
  digitalWriteFast(EN_PIN_A, HIGH);   // Disable driver in hardware

  pinMode(EN_PIN_B, OUTPUT);
  pinMode(STEP_PIN_B, OUTPUT);
  pinMode(DIR_PIN_B, OUTPUT);
  digitalWriteFast(EN_PIN_B, HIGH);   // Disable driver in hardware
  
  frameDelayMillis = (unsigned long)SPIN_DLEAY_FROM_FRAME_TRIGGER*1000ul;
  
  message.reserve(64);
  Serial.begin(115200);               // Reset registers
  while (!Serial);
  Serial.println("Setting Up...");
  
  pinMode(readPin, INPUT);
  adc->setResolution(12);
  adc->setAveraging(16);
  adc->setConversionSpeed(ADC_CONVERSION_SPEED::HIGH_SPEED_16BITS);
  adc->setSamplingSpeed(ADC_SAMPLING_SPEED::VERY_LOW_SPEED);
  adc->enableInterrupts(ADC_0);
  adc->startContinuous(readPin, ADC_0);
  
  Serial.println("Connecting to driver A...");
  Serial1.begin(115200);        // Start hardware serial 1
  drivers[0].setup();
  //driverA.push();
   
//  driverA.pdn_disable(true);     // Use PDN/UART pin for communication
//  driverA.I_scale_analog(false); // Use internal voltage reference
//  driverA.rms_current(500);      // Set driver current 500mA
//  driverA.mstep_reg_select(true); // User MSTEP register for microstep setting
//  driverA.microsteps(microstep);
//  
//  driverA.toff(2);               // Enable driver in software

  digitalWriteFast(EN_PIN_A, LOW);    // Enable driver in hardware

  Serial.print("A:DRV_STATUS = 0x");
  drivers[0].driver->DRV_STATUS(&data);
  Serial.println(data, HEX);
  Serial.println(drivers[0].driver->version());

  Serial.println("Connecting to driver B...");
  Serial3.begin(115200);        // Start hardware serial 3
  drivers[1].setup();
  //driverB.push(); 

  digitalWrite(EN_PIN_B, LOW);    // Enable driver in hardware

  Serial.print("B:DRV_STATUS = 0x");
  drivers[1].driver->DRV_STATUS(&data);
  Serial.println(data, HEX);
  Serial.println(drivers[1].driver->version());

  Serial.println("Ready");

  pinMode(FRAME_PIN, INPUT);
  attachInterrupt(FRAME_PIN, interruptFrame, RISING);
//  attachInterrupt(TRIGGER_PIN, interruptSpin, RISING);

  delay(500);
}

void loop() {
  loopMillis = millis();
  if (loopMillis - blinkTime > 500ul) {
    blinkTime = loopMillis;
    digitalWriteFast(13,!digitalReadFast(13));
  }
  drivers[0].run();
  drivers[1].run();
  if (frameTrigger)	{ // Return a value from the hall sensor, specifically the greatest magnitude measurement since the last trigger
    frameTrigger = false;
    if (ZERO_FIELD - hallMin > hallMax - ZERO_FIELD)  {
      Serial.println(hallMin);
    }
    else  {
      Serial.println(hallMax);
    }
	  //Serial.println(lastHallRead);
    hallMin = hallMax = ZERO_FIELD;
  	if (waitForFrameTrigger)	{
      waitForFrameTrigger = false;
      frameTriggerDelay = true;
      firstFrameTrigger = loopMillis;
      Serial.println("Triggered");
  	}
  }
  if (frameTriggerDelay && loopMillis - firstFrameTrigger >= frameDelayMillis)	{
	  frameTriggerDelay = false;
  	if (!drivers[selectedDriver].stepping) {
  	  drivers[selectedDriver].start();
      Serial.println("Delay finished - starting spin");
  	}
  }
  if (Serial.available()) {
    bool moreMessage = true;
    while (message.length() < 64 && moreMessage)  {
      char c = Serial.read();
      if (c != '\n')  {
        message += c;
      }
      else  {
        moreMessage = false;
      }
    }
    if (moreMessage)  {
      Serial.println("Message Error - too long");
    }
    else {
      parseMessage();
      message = "";
    }
  }
  adc->printError();
  // if (counter >= 10000) {
    // counter = 0;
    // digitalWriteFast(13, !digitalReadFast(13));
    // Serial.println(millis());
  // }
}

inline void parseMessage() {
  if (message.startsWith("run"))  {
    int ind = message.indexOf(':',3);
    if (ind > 0) {
      int val = message.substring(ind+1).toInt();
      if (val >= -1000000 && val <= 1000000) {
        drivers[selectedDriver].driver->VACTUAL(val);
        drivers[selectedDriver].driver->DRV_STATUS(&data);
//          driverA.VACTUAL(val);
//          driverA.DRV_STATUS(&data);
        Serial.print("DRV_STATUS = 0x");
        Serial.println(data, HEX);
      }
    }
  }
  else if (message.startsWith("en"))  {
    int ind = message.indexOf(':',2);
    if (ind > 0) {
      int val = message.substring(ind+1).toInt();
      drivers[selectedDriver].enable(val);
      drivers[selectedDriver].driver->DRV_STATUS(&data);
      Serial.print("A:DRV_STATUS = 0x");
      Serial.println(data, HEX);
    }
  }
  else if (message.startsWith("spin"))  {
    if (drivers[selectedDriver].stepping) {
      Serial.println("Already spinning");
      return;
    }
//    drivers[selectedDriver].accel->move(drivers[selectedDriver].acceleration*2);
//    drivers[selectedDriver].stepping = true;
//    drivers[selectedDriver].accelerating = true;
    drivers[selectedDriver].start();
  }
  else if (message.startsWith("stop"))  {
    if (drivers[selectedDriver].stepping) {
      drivers[selectedDriver].stop();
    }
  }
  else if (message.startsWith("halt"))  {
    if (drivers[selectedDriver].stepping) {
      drivers[selectedDriver].halt();
    }
  }
  else if (message.startsWith("wait"))  {
    waitForFrameTrigger = true;
    Serial.println("Waiting for trigger...");
  }
  else if (message.startsWith("speed"))  {
    int ind = message.indexOf(':',5);
    if (ind > 0) {
      float val = message.substring(ind+1).toFloat();
      if (val > 0 && val <= 100)  {
        drivers[selectedDriver].setSpeed(val);
      }
    }
  }
  else if (message.startsWith("accel"))  {
    int ind = message.indexOf(':',5);
    if (ind > 0) {
      float val = message.substring(ind+1).toFloat();
      if (val > 0 && val <= 100)  {
        drivers[selectedDriver].setAcceleration(val);
      }
    }
  }
  else if (message.startsWith("dir"))  {
    int ind = message.indexOf(':',3);
    if (ind > 0) {
      int val = message.substring(ind+1).toInt();
      drivers[selectedDriver].direction(val);
      drivers[selectedDriver].driver->DRV_STATUS(&data);
      Serial.print("A:DRV_STATUS = 0x");
      Serial.println(data, HEX);
    }
  }
  else if (message.startsWith("cur"))  {
    int ind = message.indexOf(':',3);
    if (ind > 0) {
      int val = message.substring(ind+1).toInt();
      if (val >= 0 && val <= 1400) {
        //drivers[selectedDriver].driver->rms_current(val);
        drivers[selectedDriver].setAccelCurrent(val);
        Serial.println(drivers[selectedDriver].driver->rms_current());
        if (!drivers[selectedDriver].stepping)  {
          drivers[selectedDriver].driver->DRV_STATUS(&data);
          Serial.print("A:DRV_STATUS = 0x");
          Serial.println(data, HEX);
        }
      }
    }
  }
  else if (message.startsWith("hcur"))  {
    int ind = message.indexOf(':',4);
    if (ind > 0) {
      int val = message.substring(ind+1).toInt();
      if (val >= 0 && val <= 1400) {
        //drivers[selectedDriver].driver->rms_current(val);
        drivers[selectedDriver].setRunCurrent(val);
        Serial.println(val);
        if (!drivers[selectedDriver].stepping)  {
          drivers[selectedDriver].driver->DRV_STATUS(&data);
          Serial.print("A:DRV_STATUS = 0x");
          Serial.println(data, HEX);
        }
      }
    }
  }
  else if (message.startsWith("micro"))  {
    int ind = message.indexOf(':',5);
    if (ind > 0) {
      int val = message.substring(ind+1).toInt();
      if (val >= 0 && val <= 256) {
        drivers[selectedDriver].setMicros(val);
        drivers[selectedDriver].driver->DRV_STATUS(&data);
        Serial.print("A:DRV_STATUS = 0x");
        Serial.println(data, HEX);
      }
    }
  }
  else if (message.startsWith("stepper"))  {
    int ind = message.indexOf(':',7);
    if (ind > 0) {
      int val = message.substring(ind+1).toInt();
      if (val == 0) {
        selectedDriver = 0;
      }
      else  {
        selectedDriver = 1;
      }
    }
  }
  else if (message.startsWith("spr"))  {
    int ind = message.indexOf(':',3);
    if (ind > 0) {
      int val = message.substring(ind+1).toInt();
      if (val >= 0 && val <= 16000) {
        drivers[selectedDriver].setStepperSteps(val);
      }
    }
  }
  else if (message.startsWith("rcur"))  {
    drivers[selectedDriver].toggleReduceCurrent();
  }
  else if (message.startsWith("reset"))  {
    drivers[selectedDriver].reset();
  }
  else if (message.startsWith(""))  {
    
  }
}

void interruptSpin()  {
  if (!drivers[selectedDriver].stepping) {
      drivers[selectedDriver].start();
    }
}

void interruptFrame()	{
	frameTrigger = true;
}

void adc0_isr(void) {
    lastHallRead = adc->analogReadContinuous(ADC_0);
    if (lastHallRead < hallMin) {
      hallMin = lastHallRead;
    }
    else if (lastHallRead > hallMax)  {
      hallMax = lastHallRead;
    }
    //counter += 1;
    //digitalWriteFast(LED_BUILTIN, !digitalReadFast(LED_BUILTIN)); // Toggle the led
}
