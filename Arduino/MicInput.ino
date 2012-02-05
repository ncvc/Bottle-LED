#include "TimerOne.h"

const int SER_PIN = 5;
const int RCLK_PIN = 6;
const int SRCLK_PIN = 7;
const int INPUT_PIN_1 = 2;
const int INPUT_PIN_2 = 3;
const int BAUD = 19200;

const int NUM_LEDS = 10;
const int NUM_COLORS = 3;    // number of colors per LED
const int NUM_SHIFT_REGISTERS = 4;
const int NUM_REGISTER_PINS = NUM_SHIFT_REGISTERS * 8;

const int WEIGHTED_DATA_LEN = 16; // Number of weighted averages to remember
//const int RAW_DATA_LEN = 64;  // Number of previous samples to remember

const int TIMER_WINDOW = 256;
const int TIMER_BIT_RESOLUTION = 4;
const int TIMER_RESOLUTION = pow(2, TIMER_BIT_RESOLUTION);

const float RETENTION_RATIO = .95;    // The ratio of how much previous signal is kept to how much of the new signal is incorporated per timestep

const int STAT_RECALC_THRESHOLD = 2048;    // Indicates the number of steps until the stats are recalculated
const int COLOR_CHANGE_THRESHOLD = 4096;   // Indicates the number of steps until the colors switch

const int NUM_MIXED_COLORS = 7;    // Indicates the number of colors by mixing RGB values

const byte MIXED_COLORS[NUM_MIXED_COLORS][NUM_COLORS] = {{1,0,0},    // Red
                                                         {0,1,0},    // Green
                                                         {0,0,1},    // Blue
                                                         {0,1,1},    // Cyan
                                                         {1,0,1},    // Magenta
                                                         {1,1,0},    // Yellow
                                                         {1,1,1}};   // White
                                                         // An array of the different combinations of LEDs (r,g,b) that make different colors

const byte COLOR_RED = 0;
const byte COLOR_GREEN = 1;
const byte COLOR_BLUE = 2;
const byte COLOR_CYAN = 3;
const byte COLOR_MAGENTA = 4;
const byte COLOR_YELLOW = 5;
const byte COLOR_WHITE = 6;

boolean registers[NUM_REGISTER_PINS];

byte leds[2][NUM_LEDS][NUM_COLORS];    // Array of (led number) x (r, g, b)
                                       // Note: There are two copies of the data - one to write to, and one to read from to eliminate race conditions

//float rData[RAW_DATA_LEN] = {0};
//float lData[RAW_DATA_LEN] = {0};

float wAvg1[WEIGHTED_DATA_LEN] = {0};    // Contains previous weighted averages
float wAvg2[WEIGHTED_DATA_LEN] = {0};

int timerCurrentFrame = TIMER_BIT_RESOLUTION;
int timerStep = -1;

int maxSignal = 512;      // Maximum value of the input signal

unsigned long loopStep = 0;

int currentColor = 0;    // If cycling through colors, indicates the current color
/*
float lMean = 0;
float lVariance = 0;
float rMean = 0;
float rVariance = 0;*/

byte fade = 0;


// - Arduino -

void setup() {
  pinMode(INPUT_PIN_1, INPUT);
  pinMode(INPUT_PIN_2, INPUT);
  pinMode(SER_PIN, OUTPUT);
  pinMode(RCLK_PIN, OUTPUT);
  pinMode(SRCLK_PIN, OUTPUT);
  
  analogReference(INTERNAL);

  clearLEDs(true);
  
  // Reset all register pins
  clearRegisters();
  writeRegisters();
  
  Serial.begin(BAUD);
  
  Timer1.initialize(TIMER_WINDOW);
  Timer1.attachInterrupt(modifiedBAM);
  
  //calculateStats();
}

void loop() {
  int inp1 = analogRead(INPUT_PIN_1);
  int inp2 = analogRead(INPUT_PIN_2);
  
  //pushFloat(lData, leftInput, RAW_DATA_LEN);
  //pushFloat(rData, rightInput, RAW_DATA_LEN);
  pushFloat(wAvg1, RETENTION_RATIO * wAvg1[0] + (1 - RETENTION_RATIO) * inp1, WEIGHTED_DATA_LEN);
  pushFloat(wAvg2, RETENTION_RATIO * wAvg2[0] + (1 - RETENTION_RATIO) * inp2, WEIGHTED_DATA_LEN);
  /*
  if (loopStep % STAT_RECALC_THRESHOLD == 0) {
    Serial.println("recalc");
    //calculateStats();
    Serial.println(lMean);
    Serial.println(lVariance);
  }*/
  
  if (loopStep % COLOR_CHANGE_THRESHOLD == 0) {
    currentColor = (currentColor + 1) % NUM_MIXED_COLORS;
  }
  
  processData();
 
  loopStep++;
}


// - Data Analysis -

void processData() {
  //sendDataOverSerial();
  
  // Set the leds array to the correct values to reflect the data array
  clearLEDs();
  
  modeBrightness(wAvg1[0], 0, NUM_LEDS/2, false);
  modeBrightness(wAvg2[0], NUM_LEDS-1, NUM_LEDS/2, true);
  //modeCycle();
  //useThresholds();
  //useDeltas();
}

// Sets all LEDs to the same color and brightness, altering the brightness
void modeBrightness(float inp, int startPos, int numLEDs, boolean reverse) {
  int increment = reverse ? -1: 1;
  int ledIndex = startPos;
  
  for(int ledNum=0; ledNum < startPos+numLEDs; ledNum++) {
    for (int i = 0; i < NUM_COLORS; i++) {
      if (MIXED_COLORS[currentColor][i]) {
        leds[0][ledIndex][i] = (byte) (inp / maxSignal * TIMER_RESOLUTION);
      } else {
        leds[0][ledIndex][i] = 0;
      }
    }
    ledIndex += increment;
  }
}

void useThresholds() {
  float threshold=0;
  float thresholdSize = 8;
  
  for(int ledNum = 0; ledNum < NUM_LEDS; ledNum++) {
    threshold += thresholdSize;
    for (int i = 0; i < 2; i++) {
      if(wAvg1[0] > threshold) {
        leds[0][ledNum][i] = HIGH;
      }
    }
  }
}

// Randomly cycles through colors on a per-LED basis
void modeCycle() {
  fade++;
  if (fade >= TIMER_RESOLUTION) {
    fade = 0;
  }
  
  for (int i=0; i < NUM_LEDS; i++) {
    for (int j=0; j < NUM_COLORS; j++) {
      leds[0][i][j] = random(16);
    }
  }
  
  Serial.print("fade: ");
  Serial.println(fade);
  delay(1000);
}
/*
// Calculates mean and variance of the available samples
void calculateStats() {
  lMean = getMean(lData, RAW_DATA_LEN);
  rMean = getMean(rData, RAW_DATA_LEN);
  
  lVariance = getVariance(lData, lMean, RAW_DATA_LEN);
  rVariance = getVariance(rData, rMean, RAW_DATA_LEN);
}

float getVariance(float array[], float mean, int arrayLen) {
  float var = 0;
  for (int i=0; i < arrayLen; i++) {
    var += pow(array[i] - mean, 2);
  }
  return var / arrayLen;
}

float getMean(float array[], int arrayLen) {
  float mean = 0;
  for (int i=0; i < arrayLen; i++) {
    mean += array[i];
  }
  return mean / arrayLen;
}*/

// Gets the maximum element in the array
float getMax(float array[], int arrayLen) {
  float maxNum = array[0];
  
  for (int i=1; i < arrayLen; i++) {
    if (array[i] > maxNum) {
      maxNum = array[i];
    }
  }
  
  return maxNum;
}


// - LED Control -

// Sets all registers to the appropriate value using the leds array
void setLEDs() {
  byte shade;
  int registerNum;
  int ledOffset;
  int val;
  
  for(int ledNum=0; ledNum < NUM_LEDS; ledNum++) {
    ledOffset = ledNum * NUM_COLORS;
    for(int i=0; i < NUM_COLORS; i++) {
      shade = leds[0][ledNum][i];
      
      // if the current frame's bit is one in shade, turn on the LED, otherwise turn it off
      if ((shade >> timerCurrentFrame-1) & 1) {
        val = HIGH;
      } else {
        val = LOW;
      }
      
      registerNum = ledOffset + i;
      setRegister(registerNum, val);
    }
  }
  
  writeRegisters();
}

// Uses a modified version of Bit Angle Modulation (essentially double buffering) that is not susceptable to the race conditions that standard BAM is.
// TODO: Fix the blink when transitioning from pow(2, TIMER_BIT_RESOLUTION-1) to pow(2, TIMER_BIT_RESOLUTION-1)+1
void modifiedBAM() {
  // Update the current frame, and update the LED buffer if necessary
  timerStep++;
  
  if (timerStep >= pow(2, timerCurrentFrame-1)) {
    timerCurrentFrame--;
    timerStep = -1;
    if (timerCurrentFrame == 0) {
      timerCurrentFrame = TIMER_BIT_RESOLUTION;
      //updateLEDBuffer();
    }
    
    // Turn LEDs on and off accordingly
    setLEDs();
  }
}

// Copies contents of the writable portion of leds to the read-only portion
void updateLEDBuffer() {
  for(int i=0; i < NUM_LEDS; i++) {
    for(int j=0; j < NUM_COLORS; j++) {
      leds[1][i][j] = leds[0][i][j];
    }
  }
}

// Set all LEDs to off
// all - indicates whether to reset the entire array or just the writable portion
void clearLEDs(boolean all) {
  for(int i=0; i < NUM_LEDS; i++) {
    for(int j=0; j < NUM_COLORS; j++) {
      leds[0][i][j] = 0;
    }
  }
  
  if (all) {
    for(int i=0; i < NUM_LEDS; i++) {
      for(int j=0; j < NUM_COLORS; j++) {
        leds[1][i][j] = 0;
      }
    }
  }
}

void clearLEDs() {
  clearLEDs(false);
}


// - Register Control -

// Set all register pins to LOW
void clearRegisters(){
  for(int i = 0; i < NUM_REGISTER_PINS; i++) {
     registers[i] = LOW;
  }
} 

// Set and display registers
// Only call AFTER all values are set how you would like (slow otherwise)
void writeRegisters(){
  digitalWrite(RCLK_PIN, LOW);

  for(int i = NUM_REGISTER_PINS - 1; i >= 0; i--) {
    digitalWrite(SRCLK_PIN, LOW);

    int val = registers[i];

    digitalWrite(SER_PIN, val);
    digitalWrite(SRCLK_PIN, HIGH);
  }
  
  digitalWrite(RCLK_PIN, HIGH);
}

// Set an individual pin HIGH or LOW
void setRegister(int index, int value){
  registers[index] = value;
}


// - Utility -

// Push item (int) onto the front of the array
void pushInt(int array[], int item, int arrayLen) {
  for (int i=arrayLen-1; i > 0; i--) {
    array[i] = array[i-1];
  }
  array[0] = item;
}

// Push item (float) onto the front ofthe array
void pushFloat(float array[], float item, int arrayLen) {
  for (int i=arrayLen-1; i > 0; i--) {
    array[i] = array[i-1];
  }
  array[0] = item;
}
/*
void sendDataOverSerial() {
  for (int i = 0; i < RAW_DATA_LEN; i++) {
    Serial.print((int) lData[i]);
    Serial.print(',');
  }
  Serial.println();
}*/
