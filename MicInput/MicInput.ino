#include "TimerOne.h"

#define SER_PIN 5
#define RCLK_PIN 6
#define SRCLK_PIN 7
#define R_INPUT_PIN 3
#define L_INPUT_PIN 2
#define BAUD 19200

#define NUM_LEDS 10
#define NUM_COLORS 3    // number of colors per LED
#define NUM_SHIFT_REGISTERS 4
#define NUM_REGISTER_PINS NUM_SHIFT_REGISTERS * 8

#define DATA_LEN NUM_LEDS // Number of mic samples to be collected
#define RAW_DATA_LEN 64  // Number of previous samples to remember

#define MAX_PAST_SAMPLES 10    // Number of past samples to find the max over

#define TIMER_WINDOW 64
#define TIMER_BIT_RESOLUTION 8

boolean registers[NUM_REGISTER_PINS];

byte leds[2][NUM_LEDS][NUM_COLORS];    // Array of (led number) x (r, g, b)
                                       // Note: There are two copies of the data - one to write to, and one to read from to eliminate race conditions

float rData[DATA_LEN] = {0};
float lData[DATA_LEN] = {0};

int rawData[RAW_DATA_LEN] = {0};

int timerCurrentFrame = TIMER_BIT_RESOLUTION;

byte fade = 0;

// - Arduino -

void setup() {
  pinMode(L_INPUT_PIN, INPUT);
  pinMode(R_INPUT_PIN, INPUT);
  pinMode(SER_PIN, OUTPUT);
  pinMode(RCLK_PIN, OUTPUT);
  pinMode(SRCLK_PIN, OUTPUT);
  
  analogReference(INTERNAL);

  clearLEDs(true);
  
  // Reset all register pins
  clearRegisters();
  writeRegisters();
  
  Serial.begin(BAUD);
  
  Timer1.initialize();
  Timer1.attachInterrupt(modifiedBAM);
}

void loop() {
  int leftInput = analogRead(L_INPUT_PIN);
  int rightInput = analogRead(R_INPUT_PIN);
  
  pushFloat(lData, .95 * lData[0] + .05 * leftInput, DATA_LEN);
  pushFloat(rData, .95 * rData[0] + .05 * rightInput, DATA_LEN);
  
  pushInt(rawData, leftInput, RAW_DATA_LEN);
  //Serial.println(leftInput);
  
  processData();
}


// - Data Analysis -

void processData() {
  //sendDataOverSerial();
  
  // Set the leds array to the correct values to reflect the data array
  clearLEDs();
  
  fade++;
  if (fade > pow(2, TIMER_BIT_RESOLUTION)) {
    fade = 0;
  }
  
  for (int i=0; i < NUM_LEDS; i++) {
    leds[0][i][2] = fade;
  }
  delay(1000);
  Serial.println(fade);
  
  //useThresholds();
  //useDeltas();
}

void useDeltas() {
  int delta = lData[0] - lData[1];
  int count = -512;
  int bucketSize = 512 / (NUM_LEDS * 3);
  
  for(int ledNum = 0; ledNum < NUM_LEDS; ledNum++) {
    count += bucketSize;
  }
}

void useThresholds() {
  int threshold=0;
  int thresholdSize = getMax(rawData, MAX_PAST_SAMPLES) / NUM_LEDS;
  
  for(int ledNum = 0; ledNum < NUM_LEDS; ledNum++) {
    threshold += thresholdSize;
    for (int i = 0; i < 2; i++) {
      if(lData[0] > threshold) {
        leds[0][ledNum][i] = HIGH;
      }
    }
  }
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
      shade = leds[1][ledNum][i];
      
      // if the current frame's bit is one in shade, turn on the LED, otherwise turn it off
      if ((shade >> timerCurrentFrame) & 1) {
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

// Uses a modified version of Bit Angle Modulation that is not susceptable to the race conditions that standard BAM is.
void modifiedBAM() {
  // Stop the timer to wait for the LED updates to finish
  Timer1.stop();
  
  // Update the current frame, and update the LED buffer if necessary
  timerCurrentFrame--;
  if (timerCurrentFrame == -1) {
    timerCurrentFrame = TIMER_BIT_RESOLUTION-1;
    updateLEDBuffer();
  }
  Serial.println('lol');
  // Turn LEDs on and off accordingly
  setLEDs();
  
  // Restart the timer
  Timer1.attachInterrupt(modifiedBAM, pow(2, timerCurrentFrame) * TIMER_WINDOW);
  Timer1.restart();
  Timer1.start();
}

// Copies contents of the writable portion of leds to the read-only portion
void updateLEDBuffer() {
  for(int i=0; i < NUM_LEDS; i++) {
    for(int j=0; j < NUM_COLORS; j++) {
      leds[1][i][j] = leds[0][i][j];
    }
  }
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

// Gets the maximum element in the array
int getMax(int array[], int arrayLen) {
  int maxNum = array[0];
  
  for (int i=1; i < arrayLen; i++) {
    if (array[i] > maxNum) {
      maxNum = array[i];
    }
  }
  
  return maxNum;
}

void sendDataOverSerial() {
  for (int i = 0; i < RAW_DATA_LEN; i++) {
    Serial.print((int) rawData[i]);
    Serial.print(',');
  }
  Serial.println();
}
