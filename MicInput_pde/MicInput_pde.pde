#include <fix_fft.h>

#define SER_PIN 5
#define RCLK_PIN 6
#define SRCLK_PIN 7
#define R_INPUT_PIN 3
#define L_INPUT_PIN 2
#define BAUD 9600

#define EXP 8           // 2^EXP = DATA_LEN (necessary for FFT)
#define DATA_LEN 4      // Number of mic samples to be collected

#define SIGNAL_SCALAR .25    // Scalar for the raw mic input
#define SIGNAL_OFFSET 128    // Offset for the raw mic input

#define NUM_LEDS 4
#define NUM_COLORS 3    // number of colors per LED
#define NUM_SHIFT_REGISTERS 4
#define NUM_REGISTER_PINS NUM_SHIFT_REGISTERS * 8

#define NUM_BANDS 64                          // Number of bands in the FFT output
#define BANDS_PER_LED NUM_BANDS / NUM_LEDS    // Number of bands to which each LED is assigned

boolean registers[NUM_REGISTER_PINS];

boolean leds[NUM_LEDS][NUM_COLORS];    // Array of (led number) x (r, g, b)

char im[DATA_LEN];
char rData[DATA_LEN];
char lData[DATA_LEN];

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
  unsigned long time = 0;
  unsigned long t;
  for (int i=0; i < DATA_LEN;) {
    t = micros();
    if (t > time) {
      lData[i] = .05 * lData[i] + .95 * analogRead(L_INPUT_PIN);// * SIGNAL_SCALAR - SIGNAL_OFFSET;
      rData[i] = .05 * rData[i] + .95 * analogRead(R_INPUT_PIN);// * SIGNAL_SCALAR - SIGNAL_OFFSET;
      im[i] = 0;
      
      time = t;
      i++;
    }
  }
  
  //int scale = fix_fft(data, im, EXP, 0);

  // I am only interested in the absolute value of the transformation
  for (int i=0; i < DATA_LEN; i++) {
     //data[i] = pow(2, scale) * (data[i] * data[i] + im[i] * im[i]);
  }
  
  // process the magnitudes
  processData();
}

void processData() {
  sendDataOverSerial();
  
  // Set the leds array to the correct values to reflect the data array
  clearLEDs();
  
  int threshold;
  
  for(int ledNum = 0; ledNum < NUM_LEDS; ledNum++) {
    threshold = 0;
    for (int i = 0; i < 3; i++) {
      threshold += 40;
      if(lData[ledNum] > threshold) {
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
  for (int i = 0; i < DATA_LEN; i++) {
    Serial.print((int) lData[i]);
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
