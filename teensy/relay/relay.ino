#define HWSERIAL Serial1
#define MAX_BYTES 36
//
//char input[MAX_BYTES];
//int i = 0;
//bool writeFlag = false;

void setup() {
  Serial.begin(9600);
  HWSERIAL.begin(9600);
}

void loop() {
  int incomingByte = 0;
  if (HWSERIAL.available() > 0) {
//    if(incomingByte == 36){
//      Serial.println();
//    }
//    incomingByte = HWSERIAL.read();
//    input[i] = incomingByte;
//    i++;
    Serial.write(HWSERIAL.read());
    
  }
}

