int discardByte[22];
int n=1;
#define MagF            51                      // electromagnet pin

#define InL1            13                      // left motor pin1
#define PWML            10                      // PWM for left motor pin  
#define InL2            9                       // left motor pin2  

#define InR1            7                       // right motor pin1
#define PWMR            6                       // PWM for right motor pin
#define InR2            4                       // right motor pin2 

void setup() {
Serial.begin(9600);
Serial3.begin(9600);
pinMode(MagF, OUTPUT);
    
pinMode(InL1, OUTPUT);
pinMode(InL2, OUTPUT);
pinMode(PWML, OUTPUT);
    
pinMode(InR1, OUTPUT);
pinMode(InR2, OUTPUT);
pinMode(PWMR, OUTPUT);

pinMode(MagF, OUTPUT);
digitalWrite(MagF, LOW);
    
}

void loop() {
int a,b; //variables containing analog readings
if (Serial3.read() == 0x7E) { // 7E is the start byte
for (int i = 1; i<=20; i++) { // Skip ahead to the analog data
  discardByte[i] = Serial3.read();
  //Serial.print(discardByte[i]);
  //Serial.print(" ");
}
int analogMSB = discardByte[13]; // Read the first analog byte data
int analogLSB = discardByte[14]; // Read the second byte
int analogReading = analogLSB + (analogMSB * 256);
int analog1MSB = discardByte[15]; // Read the first analog byte data
int analog1LSB = discardByte[16]; // Read the second byte
int analog1Reading = analog1LSB + (analog1MSB * 256);
int magnet = discardByte[11];
//int buzzer = discardByte[12];//extra pin on remote used for buzzer
if (analogReading>=0 && analogReading<1024 && analog1Reading>=0 && analog1Reading<1024) //filter for garbage value
{
   a = analogReading;
   b = analog1Reading;  
}
Serial.print("A0= ");
Serial.print(a);
Serial.print(" ");
Serial.print(" A1= ");
Serial.print(b);
Serial.print(" ");
Serial.print("magnet=");
Serial.print(magnet);
Serial.println(" ");
delay(10);
if (a==1023 || b==1023)  // if and else if logic for motor movement
{
 forward(0,0);
 backward(0,0);
 magnet_off();
}
 if (a>511 && a<1020)
{
 forward(1,0);
 backward(1,0);
}
else if (a<541)
{
 forward(0,1);
 backward(0,1);
}
else if( b>540 && b<1020)
{
  left(0,1);
}
else if( b<240)
{
  right(0,1);
}
else if(magnet ==0)
{
  magnet_on();
}
else if(magnet ==1)
{
  magnet_off();
} 
delay(25);
}
}
void forward(int x1,int x2) //runs motor in forward direction
{
if (x1==0 && x2==0)
{digitalWrite(InL1,LOW);
digitalWrite(InL2,LOW);
analogWrite(PWML,0);}
else if (x1==1 && x2==0)
{digitalWrite(InL1,HIGH);
digitalWrite(InL2,LOW);
analogWrite(PWML,255);}
else if (x1==0 && x2==1)
{digitalWrite(InL1,LOW);
digitalWrite(InL2,HIGH);
analogWrite(PWML,255);}
}
void backward(int y1,int y2) //runs motor in backward direction
{
if (y1==0 && y2==0)
{digitalWrite(InR1,LOW);
digitalWrite(InR2,LOW);
analogWrite(PWMR,0);
}
else if (y1==1 && y2==0)
{digitalWrite(InR1,LOW);
digitalWrite(InR2,HIGH);
analogWrite(PWMR,255);
}
else if (y1==0 && y2==1)
{digitalWrite(InR1,HIGH);
digitalWrite(InR2,LOW);
analogWrite(PWMR,255);
}
}
void left(int a1,int a2) //one motor moves forward and other one moves backwards
{
{
digitalWrite(InR1,HIGH);
digitalWrite(InR2,LOW);
analogWrite(PWMR,255);
digitalWrite(InL1,HIGH);
digitalWrite(InL2,LOW);
analogWrite(PWML,255);
}
}
void right(int b1,int b2) //similar to left function but reciprocal in motor action
{
{
digitalWrite(InR1,LOW);
digitalWrite(InR2,HIGH);
analogWrite(PWMR,255);
digitalWrite(InL1,LOW);
digitalWrite(InL2,HIGH);
analogWrite(PWML,255);
}
}
void magnet_on(void) //for turning on the magnet
{
  digitalWrite(MagF, HIGH);
}
void magnet_off(void) //for turning off the magnet
{
  digitalWrite(MagF, LOW);
}
