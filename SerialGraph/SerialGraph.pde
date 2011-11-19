import processing.serial.*;

static final int DATA_LEN = 256;
static final int BAUD = 9600;

static final int LOWER_DATA_RANGE = -128;
static final int UPPER_DATA_RANGE = 128;

static final int WINDOW_WIDTH = 800;
static final int WINDOW_HEIGHT = 400;
static final int BAR_WIDTH = max(1, WINDOW_WIDTH / DATA_LEN);

Serial myPort;
float[] data = new float[DATA_LEN];
 
void setup () {
  size(WINDOW_WIDTH, WINDOW_HEIGHT);
  background(0);
  
  myPort = new Serial(this, Serial.list()[0], BAUD);
  myPort.bufferUntil('\n');
}

void draw () {
  int xPos=0;
  float inByte;
  
  background(0);
  
  for (int i=0; i < DATA_LEN; i++) {
    inByte = data[i];
    
    // draw the line:
    stroke(127,34,255);
    rect(xPos, height-inByte, BAR_WIDTH, inByte);
   
    xPos += BAR_WIDTH;
  }
}
 
void serialEvent (Serial myPort) {
  println();
  String inString = myPort.readStringUntil(',');
  
  for (int index = 0; inString != null; index++) {
    if (index < DATA_LEN) {
      inString = trim(inString);
      
      // remove trailing comma
      inString = inString.substring(0, inString.length()-1);
      
      print(inString + ',');
     
      // convert to a float and map to the screen height:
      float inByte = float(inString);
      data[index] = map(inByte, LOWER_DATA_RANGE, UPPER_DATA_RANGE, 0, height);
    }
    inString = myPort.readStringUntil(',');
  }
}
