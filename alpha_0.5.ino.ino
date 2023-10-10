
// Motor parameters

// left motor
#define inA 3
#define inB 4


// enable pin
#define enA 5
#define enB 6

// right motor
#define inC 7
#define inD 8

// PID
int IMS = 180; // Initial motor speed
int LMS = 180;
int RMS = 180;

int error = 0;
float P = 0, I = 0, D = 0, PID = 0;
float prevErr = 0; // Previous error
float prevI = 0;   // Previous integral

// if your bot oscillate too much then we need to decrease kp value
// after that if it response slowly then we need to increase it until i get better result
float kp = 22, kd = 38, ki = 1.7; // Tune it

//  If your robot overshoots the desired path, you may need to increase kd. If it responds too slowly to changes, you might need to decrease it

int flag = 0, tlag = 0;
 
int middleValue = 300;
int sensorVal[8]; // Change the array size to match the number of sensors
int sum = 0, searchFlag = 0, sumTemp = 12;

void readSensor();
void Forward();
void Back();
void Stop();
void Search();
void softRight();
void softLeft();
void hardRight();
void hardLeft();
void logic();
void pidFun();    // PID function
void pidExecute(); // Analog write via PID update

void setup() {
  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  pinMode(A2, INPUT);
  pinMode(A3, INPUT);
  pinMode(A4, INPUT);
  pinMode(A5, INPUT);
  pinMode(A6, INPUT);
  pinMode(A7, INPUT); // Set pins 6 and 7 as digital inputs

  Serial.begin(9600);
}

void loop() {
  readSensor();
  pidExecute();

  logic();
  Serial.print("LMS ");
  Serial.print(LMS);
  Serial.print("    ");
  Serial.print("RMS");
  Serial.print(RMS);

  Serial.println();
  Serial.println();
  Serial.println();
}

void readSensor() {
  for (int i = 0; i < 8; i++) { // Change the loop limit to match the number of analog sensors
    if (analogRead(i) > 500)
      sensorVal[i] = 1;
    else
      sensorVal[i] = 0;

    Serial.print(sensorVal[i]);
    Serial.print(' ');
  }


  Serial.println('\n');

  sum = (sensorVal[0]*128)+(sensorVal[1]*64)+(sensorVal[2]*32)+(sensorVal[3]*16)+(sensorVal[4]*8)+(sensorVal[5]*4) + (sensorVal[6]*2) +(sensorVal[7]*1); 

  pidFun();
}

void logic() {
  if (sum == 24 || sum == 48 || sum == 12 || sum == 96 || sum == 192 || sum == 6 || sum == 3 || sum == 14 || sum == 28 || sum == 112 || sum == 7|| sum==224) {
    Serial.println("Condition Forward");
    Forward();
  } else if (sum == 255) { // Example condition for full black with 8 sensors
    Forward();
    delay(100);
    Serial.println("Condition Stop");
    Stop();
    readSensor();
    if (sum == 255) {
      Stop();
      delay(1000);
    } else if (sum == 0) {
      Search();
    } else {
      readSensor();
      Forward();
    }
  } else {
    Search();
  }
}

void pidExecute() {
  P = error;
  I = I + prevI;
  D = error - prevErr;
  PID = (kp * P) + (kd * D) + (ki * I);
  prevErr = error;
  prevI = I;

  LMS = IMS + PID;
  RMS = IMS - PID;
  LMS = constrain(LMS, 0, 255);
  RMS = constrain(RMS, 0, 255);

  analogWrite(enA, RMS);
  analogWrite(enB, LMS);
}

void pidFun() {
  switch (sum) {
    case 1:  // Binary: 00000001
      error = -7;
      break;
    case 2:  // Binary: 00000010
      error = -6;
      break;
    case 4:  // Binary: 00000100
      error = -5;
      break;
    case 8:  // Binary: 00001000
      error = -4;
      break;
    case 16:  // Binary: 00010000
      error = -3;
      break;
    case 32:  // Binary: 00100000
      error = -2;
      break;
    case 64:  // Binary: 01000000
      error = -1;
      break;
    case 128:  // Binary: 10000000
      error = 0;
      break;
    case 192:  // Binary: 11000000
      error = 1;
      break;
    case 160:  // Binary: 10100000
      error = 2;
      break;
    case 96:  // Binary: 01100000
      error = 3;
      break;
    case 80:  // Binary: 01010000
      error = 4;
      break;
    case 144:  // Binary: 10010000
      error = 5;
      break;
    case 240:  // Binary: 11110000
      error = 6;
      break;
    case 224:  // Binary: 11100000
      error = 7;
      break;
    default:
      error = 0;
      P = 0;
      I = 0;
      D = 0;
      PID = 0;
      prevErr = 0;
      prevI = 0;
  }


}

void Forward()
{
  digitalWrite(inA,LOW);
  digitalWrite(inB,HIGH);
  digitalWrite(inC,LOW);
  digitalWrite(inD,HIGH);

  Serial.println("Forward"); 
}

void Back()
{
  analogWrite(enA, IMS);
  analogWrite(enB, IMS);
  
  digitalWrite(inA,HIGH);
  digitalWrite(inB,LOW);
  digitalWrite(inC,HIGH);
  digitalWrite(inD,LOW);

  Serial.println("Backward"); 
}

//INA,INB LEFT
//enA right
void softRight()
{

  analogWrite(enA, 0);
  analogWrite(enB, IMS);
   
  digitalWrite(inA,LOW);
  digitalWrite(inB,LOW);
  digitalWrite(inC,LOW);
  digitalWrite(inD,HIGH);
  flag=1;

  Serial.println("Soft Left");
}

void softLeft()
{
  analogWrite(enA, IMS);
  analogWrite(enB, 0);
  
  digitalWrite(inA,LOW);
  digitalWrite(inB,LOW);
  digitalWrite(inC,LOW);
  digitalWrite(inD,HIGH);
  tlag=1;

  Serial.println("Soft Left");
}
void hardRight()
{
  analogWrite(enA, IMS);
  analogWrite(enB, IMS);
  
  digitalWrite(inA,LOW);
  digitalWrite(inB,HIGH);
  digitalWrite(inC,HIGH);
  digitalWrite(inD,LOW);
  flag=1;

  Serial.println("Hard Right");
}
void hardLeft()
{
  analogWrite(enA, IMS);
  analogWrite(enB, IMS);
  
  digitalWrite(inA,HIGH);
  digitalWrite(inB,LOW);
  digitalWrite(inC,LOW);
  digitalWrite(inD,HIGH);

  Serial.println("Hard Left");  
  tlag=1;
}
void Stop()
{
  analogWrite(enA, 0);
  analogWrite(enB, 0);
  
  digitalWrite(inA,LOW);
  digitalWrite(inB,LOW);
  digitalWrite(inC,LOW);
  digitalWrite(inD,LOW);

  Serial.println("Stop");
}

void Search()
{
  readSensor();
//  Stop();
  if(sum>24)
  {
    softLeft();
      Stabilizer();
  }
  else if(sum<16){
    softRight(); 
      Stabilizer(); 
  }
  else if(sum == 24)
  {
    Forward();
  }
  else
  {
    Stop();
    delay(500);
  }
  Serial.println("Searching");
}

void Stabilizer()
{
    readSensor();
    
    if(sum == 24 || sum == 16 || sum == 8 || sum == 6|| sum == 48)
    {
      if(flag==1 && tlag==0)
      {
        hardLeft();
        delay(10);
        flag=0;
        tlag=0;
        readSensor();
      }
      else if(flag==0 && tlag==1)
      {
        hardRight();
        delay(10);
        flag=0;
        tlag=0;
        readSensor();
      }
      Stop();
      delay(100);
      break;
    }
  
  readSensor();
  pidExecute();
  
  Forward();
}
