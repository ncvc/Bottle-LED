#define SER_PIN 5
#define RCLK_PIN 6
#define SRCLK_PIN 7
#define R_INPUT_PIN 3
#define L_INPUT_PIN 2
#define BAUD 19200

#define NUM_LEDS 4
#define NUM_COLORS 3    // number of colors per LED
#define NUM_SHIFT_REGISTERS 4
#define NUM_REGISTER_PINS NUM_SHIFT_REGISTERS * 8

#define DATA_LEN NUM_LEDS // Number of mic samples to be collected
#define RAW_DATA_LEN 64  // Number of previous samples to remember

#define MAX_PAST_SAMPLES 10    // Number of past samples to find the max over

boolean registers[NUM_REGISTER_PINS];

boolean leds[NUM_LEDS][NUM_COLORS];    // Array of (led number) x (r, g, b)

float rData[DATA_LEN] = {0};
float lData[DATA_LEN] = {0};

int rawData[RAW_DATA_LEN] = {0};

void setup() {
  pinMode(L_INPUT_PIN, INPUT);
  pinMode(R_INPUT_PIN, INPUT);
  pinMode(SER_PIN, OUTPUT);
  pinMode(RCLK_PIN, OUTPUT);
  pinMode(SRCLK_PIN, OUTPUT);
  
  analogReference(INTERNAL);

  clearLEDs();
  
  // Reset all register pins
  clearRegisters();
  writeRegisters();
  
  Serial.begin(BAUD);
}

void loop() {
  int leftInput = analogRead(L_INPUT_PIN);
  int rightInput = analogRead(R_INPUT_PIN);
  
  pushFloat(lData, .95 * lData[0] + .05 * leftInput, DATA_LEN);
  pushFloat(rData, .95 * rData[0] + .05 * rightInput, DATA_LEN);
  
  pushInt(rawData, leftInput, RAW_DATA_LEN);
  Serial.println(leftInput);
  
  processData();
}

void processData() {
  //sendDataOverSerial();
  
  // Set the leds array to the correct values to reflect the data array
  clearLEDs();
  
  int threshold=0;
  int thresholdSize = getMax(rawData, MAX_PAST_SAMPLES) / NUM_LEDS;
  
  for(int ledNum = 0; ledNum < NUM_LEDS; ledNum++) {
    threshold += thresholdSize;
    for (int i = 0; i < 1; i++) {
      if(lData[0] > threshold) {
        leds[ledNum][i] = HIGH;
      }
    }
  }
  
  setLEDs();
}

// Sets all registers to the appropriate value using the leds array
void setLEDs() {
  int val;
  int registerNum;
  
  for(int ledNum=0; ledNum < NUM_LEDS; ledNum++) {
    for(int i=0; i < NUM_COLORS; i++) {
      registerNum = ledNum * 3 + i;
      val = leds[ledNum][i];
      setRegister(registerNum, val);
    }
  }
  
  writeRegisters();
}

void sendDataOverSerial() {
  for (int i = 0; i < RAW_DATA_LEN; i++) {
    Serial.print((int) rawData[i]);
    Serial.print(',');
  }
  Serial.println();
}

// Set all LEDs to off
void clearLEDs() {  
  for(int i=0; i < NUM_LEDS; i++) {
    for(int j=0; j < 3; j++) {
      leds[i][j] = LOW;
    }
  }
}

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
