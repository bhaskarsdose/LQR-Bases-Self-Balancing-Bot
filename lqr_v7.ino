 /*
 * Team Id: 1456
 * Author List: Bhaskar,Deepanshu
 * Filename: lqr_v7
 * Theme: Biped-Patrol
 * Functions: setup(),loop(),task(),attachinterrupt(),left_movement(),righ_movement(),motorinit(),motorForwardL(),motorForwardR(),motorBackwardR(),motorBackwardL(),
 * stop_motorR(),stop_motorL(),BUZZ_init(),MAG_init(),MagPick(),MagDrop(),timer_init(),start_timer(),ISR(),process_sensor(),low_pass_filter(),high_pass_filter(),
 * comp_pitch(),balance_using_motor(),process_encoder(),lqr(),handle_flags(),joystick_read(),left_motor(),right_motor(),left(),right().
 * Global Variables: slope_offset,move_offset,max_angle_vel,max_angle_enc,accel_angle,gyro_angle,rotation_left,rotation_right,
 * left_RPM,right_RPM,left_prev_count,right_prev_count,last_task_time_PID,y_setpoint.
 */
#include "I2Cdev.h"
#include "MPU6050.h"
#include "math.h"
//#include <digitalWriteFast.h>
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif
#define MagF            51                      // electromagnet pin
#define buzz_pin        31                      // electromagnet pin
#define RED_pin         43                      //LED Pin
#define Common_pin      45                      //LED Pin
#define GREEN_pin       47                      //LED Pin
#define BLUE_pin        49                      //LED Pin
#define InL1            11                      // motor pin
#define PWML            46                     // PWM motor pin
#define InL2            12                       // motor pin
#define InR1            7                       // motor pin
#define PWMR            45                       // PWM motor pin
#define InR2            8                       // motor pin
#define enc1pinA 2 
#define enc1pinB 5
#define enc2pinA 3  
#define enc2pinB 4
    int f_cut = 5.50;
float alpha1=0.01;
int flagright=0, flagleft=0,flagforward=0, flagbackward=0, flagstop=0;
int discardByte[22];
int n=1;
int A[10],B[10];
int a,b;

// Global Variables
float slope_offset=0, move_offset=0, max_angle_vel=4, max_angle_enc=2;
volatile float accel_angle=0, gyro_angle=0;
volatile float rotation_left=0, rotation_right=0;
volatile float left_RPM=0, right_RPM=0, left_prev_count=0, right_prev_count=0;
unsigned long last_task_time_PID=0;
volatile float y_setpoint=0;

//timer 4
volatile unsigned long int time_ms = 0;
volatile unsigned long int time_sec = 0;
unsigned long elapsed_time;
volatile unsigned long int sensor_ms = 0;

//encoder variables
volatile int right_count=0;
volatile int left_count=0;
volatile int newposition_1=0,newposition_2=0;
volatile int oldposition_1=0,oldposition_2=0;
unsigned long timenew,timeold=0;
float vel = 0, velb=0,vela = 0,ul=0, ur=0,v0=0,v1=0;

MPU6050 accelgyro;
int flagr=0,flagl=0;
float t,t1,oldt;
float x; //state variables position using encoder
float x_dot;//state variable velocity
float theta;// angle using MPU6050
float theta_dot;// angular velocity
float pitch_angle=0;
//sensor values
int16_t ax1, ay1, az1;
int16_t gx1, gy1, gz1;
float old1=0, old2=0 ,old3=0;
float AX1=0,AY1=0,AZ1=0;
float old_gx=0, new_gx =0,old_gy=0 ,new_gy=0 ,old_gz=0 ,new_gz=0;
float GX_N=0 ,GY_N=0, GZ_N=0;
float GX_O=0, GY_O=0, GZ_O=0;
float old_pitch_angle=0, new_pitch_angle=0;
float pitch_acc=0,  pitch_gyro=0;
float old_roll_angle =0,new_roll_angle=0;
float roll_acc=0, roll_gyro=0;

float u=0; //input for the system
// Structure to handle the various inputs from the controller
typedef struct JoystickController
{
    int x_position; // Joystick X-Axis ADC Value
    int y_position; // Joystick Y-Axis ADC Value

    bool button_1; // Push Button 1 Value
    bool button_2; // Push Button 2 Value
    bool button_3; // Push Button 3 Value
    bool button_4; // Push Button 4 Value

    // De-bounce time for the push buttons
    unsigned long b1_time;
    unsigned long b2_time;
    unsigned long b3_time;
    unsigned long b4_time;
};

extern JoystickController joystick;
JoystickController joystick = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
unsigned long int last_lqr_task =0;
unsigned long int last_sensor_task =0;
// External Variables
extern JoystickController joystick;

/**********************************
Function name	:	setup
Functionality	:	To setup all parameters once in main()
Arguments		:	None
Return Value	:	None
Example Call	:	setup() - Called internally by main()
***********************************/
void setup()

{   
    Serial3.begin(9600);
    timer3_init();    // for sensor reading gy87
    timer4_init();    // for calculating overall program time epoch()
    timer1_init();  // for encoder reading
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
    // TWBR = 24;
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
#endif
    attach_interrupt();
    motor_init();
    LED_init();
    BUZZ_init();
    MAG_init();
    accelgyro.initialize();
    start_timer3(); // for sampling sensor data @0.005 sec
    start_timer4(); // for controlling motors  @0.005 sec
    start_timer1();
    
}
/**********************************
Function name	:	loop
Functionality	:	To loop the program 
Arguments		:	None
Return Value	:	None
Example Call	:	loop() - Called internally 
***********************************/
void loop()
{
   task(); //for every task
}
/**********************************
Function name	:	task
Functionality	:	To schedule various tasks
Arguments		:	None
Return Value	:	None
Example Call	:	task_scheduler()
***********************************/
void task()
{
  accelgyro.getMotion6(&ax1, &ay1, &az1, &gx1, &gy1, &gz1);
  joystick_read();
  handle_flags();
      if ((sensor_time() - last_sensor_task) > 1)
      {
        last_sensor_task = sensor_time();
        process_sensor();
      }
      if ((epoch() - last_lqr_task) > 1)
      {
        last_lqr_task = epoch();
        lqr();
      }
}
/**********************************
Function name	:	attach_interrupt()
Functionality	:	To configure the quadrature encoder pins
Arguments		:	None
Return Value	:	None
Example Call	:	attach_interrupt
***********************************/
void attach_interrupt() //attaching interrups to digital pins 2,3 and also initialising the motor pins
{  
    pinMode(enc1pinA,INPUT);
    pinMode(enc1pinB,INPUT);
    pinMode(enc2pinA,INPUT);
    pinMode(enc2pinB,INPUT);
    digitalWrite(enc1pinA,HIGH);
    digitalWrite(enc1pinB,HIGH);
    digitalWrite(enc2pinA,HIGH);
    digitalWrite(enc2pinB,HIGH);
    attachInterrupt(1, left_movement,FALLING);
    attachInterrupt(0, right_movement, FALLING);
    right_count = 0;
    left_count = 0;
}
/**********************************
Function name	:	left_movement()
Functionality	:	To handle interrupt for left encoder channel A
Arguments		:	None
Return Value	:	None
Example Call	:	Called automatically
***********************************/
void left_movement()
{
    int state = digitalRead(enc1pinA);
    if(digitalRead(enc1pinB))
    state ? left_count-- : left_count++;
    else
    state ? left_count++ : left_count--;
}
/**********************************
Function name	:	right_movement()
Functionality	:	To handle interrupt for right encoder channel A
Arguments		:	None
Return Value	:	None
Example Call	:	Called automatically
***********************************/
void right_movement()
{
    int state = digitalRead(enc2pinA);
    if(digitalRead(enc2pinB))
    state ? right_count++ : right_count--;
    else
    state ? right_count-- : right_count++;
}
/**********************************
Function name	:	motor_init
Functionality	:	To configure the motor pins
Arguments		:	None
Return Value	:	None
Example Call	:	motor_init()
***********************************/
void motor_init()
{
    pinMode(MagF, OUTPUT);
    pinMode(InL1, OUTPUT);
    pinMode(InL2, OUTPUT);
    pinMode(PWML, OUTPUT);
    pinMode(InR1, OUTPUT);
    pinMode(InR2, OUTPUT);
    pinMode(PWMR, OUTPUT);
}
/**********************************
Function name	:	motorForwardL
Functionality	:	To move left motor forward
Arguments		:	None
Return Value	:	None
Example Call	:	motorForwardL()
***********************************/
void motorForwardL(int PWM_val)  
{
    analogWrite(PWML, PWM_val);
    digitalWrite(InL1, LOW);
    digitalWrite(InL2, HIGH);
    flagl=1;
}
/**********************************
Function name	:	motorBackwardL
Functionality	:	To move left motor back
Arguments		:	None
Return Value	:	None
Example Call	:	motorBackwardL()
***********************************/
void motorBackwardL(int PWM_val)  
{
    analogWrite(PWML, PWM_val);
    digitalWrite(InL2, LOW);
    digitalWrite(InL1, HIGH);
    flagl=0;
}
/**********************************
Function name	:	motorBackwardR
Functionality	:	To move right motor backward
Arguments		:	None
Return Value	:	None
Example Call	:	motorBackwardR()
***********************************/
void motorBackwardR(int PWM_val)  
{
    analogWrite(PWMR, PWM_val);
    digitalWrite(InR2, LOW);
    digitalWrite(InR1, HIGH);
}
/**********************************
Function name	:	motorForwardR
Functionality	:	To move right motor forward
Arguments		:	None
Return Value	:	None
Example Call	:	motorForwardR()
***********************************/
void motorForwardR(int PWM_val)  
{
    analogWrite(PWMR, PWM_val);
    digitalWrite(InR1, LOW);
    digitalWrite(InR2, HIGH);
}
/**********************************
Function name	:	stop_motorR
Functionality	:	To stop right motor
Arguments		:	None
Return Value	:	None
Example Call	:	stop_motorR()
***********************************/
void stop_motorR()
{
    digitalWrite(InR1, LOW);
    digitalWrite(InR2, LOW);
}
/**********************************
Function name	:	stop_motorL
Functionality	:	To stop the left motor
Arguments		:	None
Return Value	:	None
Example Call	:	stop_motorL()
***********************************/
void stop_motorL()
{
    digitalWrite(InL2, LOW);
    digitalWrite(InL1, LOW);  
}
/**********************************
Function name	:	LED_init
Functionality	:	To configure LED pins
Arguments		:	None
Return Value	:	None
Example Call	:	LED_init()
***********************************/
void LED_init(){
    pinMode(RED_pin, OUTPUT);
    pinMode(Common_pin, OUTPUT);
    pinMode(GREEN_pin, OUTPUT);
    pinMode(BLUE_pin, OUTPUT);
    digitalWrite(RED_pin, HIGH);
    digitalWrite(Common_pin, HIGH);
    digitalWrite(GREEN_pin, HIGH);
    digitalWrite(BLUE_pin, HIGH);
}
/**********************************
Function name	:	BUZZ_init
Functionality	:	To configure the Buzzer pin
Arguments		:	None
Return Value	:	None
Example Call	:	BUZZ_init()
***********************************/
void BUZZ_init()
{
    pinMode(buzz_pin, OUTPUT);
    digitalWrite(buzz_pin, HIGH);
}
/**********************************
Function name	:	MAG_init
Functionality	:	To configure the magnet pins
Arguments		:	None
Return Value	:	None
Example Call	:	MAG_init()
***********************************/
void MAG_init(){
    pinMode(MagF, OUTPUT);
    digitalWrite(MagF, LOW);
}
/**********************************
Function name	:	MagPick
Functionality	:	To turn magnet pin high
Arguments		:	None
Return Value	:	None
Example Call	:	MagPick()
***********************************/
void MagPick(void)  {
    digitalWrite(MagF, HIGH);
}
/**********************************
Function name	:	MagDrop
Functionality	:	To turn the magnet pin low
Arguments		:	None
Return Value	:	None
Example Call	:	MagDrop()
***********************************/
void MagDrop(void)  {
    digitalWrite(MagF, LOW);
}
/**********************************
Function name	:	timer3_init
Functionality	:	TIMER3 Initialize - Prescaler: 1024
Arguments		:	None
Return Value	:	None
Example Call	:	timer3_init()
***********************************/
//timer 3 FOR SENSOR sampling
void timer3_init()
{
    TCCR3B = 0x00;  // Stop Timer
    TCNT3 = 0xFEC7; //0.005 sec
    OCR3A = 0x0000; // Output Compare Register (OCR) - Not used
    OCR3B = 0x0000; // Output Compare Register (OCR) - Not used
    OCR3C = 0x0000; // Output Compare Register (OCR) - Not used
    ICR3 = 0x0000;  // Input Capture Register (ICR)  - Not used
    TCCR3A = 0x00;
    TCCR3C = 0x00;
}
/**********************************
Function name	:	start_timer3
Functionality	:	Start timer 3
Arguments		:	None
Return Value	:	None
Example Call	:	start_timer3()
***********************************/
void start_timer3()
{
    TCCR3B = 0x04; // Prescaler 256
    TIMSK3 = 0x01; // Enable Timer Overflow Interrupt
}
/**********************************
Function name	:	ISR(TIMER3_OVF_vect)
Functionality	:	ISR for measuring the tilt angle at 100Hz
Arguments		:	Timer 3 overflow vector
Return Value	:	None
Example Call	:	Called automatically
***********************************/
ISR(TIMER3_OVF_vect) // overflow flag is raised when TCNT reaches 65536
{
     TIMSK3 = 0x00;
     sensor_ms++;      // Increment ms value
     TCNT3 = 0xFEC7;   //0.005 ,256 as pre scalar
     TIMSK3 = 0x01;
}
/**********************************
Function name	:	sensor_time
Functionality	:	return the last sensor reading time
Arguments		:	None
Return Value	:	time in ms
Example Call	:	sensor_time()
***********************************/
unsigned int sensor_time()
{
  return sensor_ms;
  }
/**********************************
Function name	:	process_sensor
Functionality	:	to turn raw value into useful sensor data
Arguments		:	None
Return Value	:	None
Example Call	:	process_sensor()
***********************************/
void process_sensor()
  {
    low_pass_filter(ax1, ay1, az1);              // accelerometer values to low pass filter
    high_pass_filter(gx1, gy1, gz1);             // gyroscope values to high pass filter
    comp_pitch(AX1, AY1, AZ1, GX_N, GY_N, GZ_N); // comp filter for calculating pitch
    theta = new_pitch_angle;
    theta_dot = ((new_pitch_angle - pitch_angle) / 0.01);
    pitch_angle = new_pitch_angle;
     }
/**********************************
Function name	:	timer4_init
Functionality	:	TIMER4 Initialize - Prescaler: None
Arguments		:	None
Return Value	:	None
Example Call	:	timer4_init()
***********************************/
//timer 4 FOR FULL PROGRAM TIME
void timer4_init()
     {
       TCCR4B = 0x00;  // Stop Timer
       TCNT4 = 0x63C0; // 0.0009999593097s (~0.001s)
       OCR4A = 0x0000; // Output Compare Register (OCR) - Not used
       OCR4B = 0x0000; // Output Compare Register (OCR) - Not used
       OCR4C = 0x0000; // Output Compare Register (OCR) - Not used
       ICR4 = 0x0000;  // Input Capture Register (ICR)  - Not used
       TCCR4A = 0x00;
       TCCR4C = 0x00;
}
/**********************************
Function name	:	start_timer4
Functionality	:	Start timer 4
Arguments		:	None
Return Value	:	None
Example Call	:	start_timer4()
***********************************/
void start_timer4()
{
  TCCR4B = 0x02;    // Prescaler 8
  TIMSK4 = 0x01;    // Enable Timer Overflow Interrupt
}
/**********************************
Function name	:	ISR(TIMER4_OVF_vect)
Functionality	:	ISR for program clock
Arguments		:	Timer 4 overflow vector
Return Value	:	None
Example Call	:	Called automatically
***********************************/
ISR(TIMER4_OVF_vect)
{
  TIMSK4 = 0x00;  // Reload counter value
  time_ms++;      // Increment ms value
  TCNT4=0xD8F0;//DELAY 0F 0.005 SEC 8 as pre scalar
  TIMSK4= 0x01;
}
/**********************************
Function name	:	epoch
Functionality	:	To keep program time from the start in ms
Arguments		:	None
Return Value	:	Current program time
Example Call	:	epoch()
***********************************/
unsigned long epoch()
{
  elapsed_time = time_ms;
  return elapsed_time;
}
/**********************************
Function name	:	timer1_init
Functionality	:	TIMER1 Initialize - Prescaler: 256
Arguments		:	None
Return Value	:	None
Example Call	:	timer1_init()
***********************************/
//timer 1    FOR ENCODER  Sampling
void timer1_init()
{
    TCCR1B = 0x00;  // Stop Timer
    TCNT1 = 0xFB80; // 0.02s
    OCR1A = 0x0000; // Out-put Compare Register (OCR) - Not used
    OCR1B = 0x0000; // Output Compare Register (OCR) - Not used
    OCR1C = 0x0000; // Output Compare Register (OCR) - Not used
    ICR1 = 0x0000;  // Input Capture Register (ICR)  - Not used
    TCCR1A = 0x00;
    TCCR1C = 0x00;
}
/**********************************
Function name	:	start_timer1
Functionality	:	Start timer 1
Arguments		:	None
Return Value	:	None
Example Call	:	start_timer1()
***********************************/
void start_timer1()
{
    TCCR1B = 0x04; // Prescaler 256 1-0-0
    TIMSK1 = 0x01; // Enable Timer Overflow Interrupt
}
/**********************************
Function name	:	ISR(TIMER1_OVF_vect)
Functionality	:	ISR for program clock
Arguments		:	Timer 1 overflow vector
Return Value	:	None
Example Call	:	Called automatically
***********************************/
ISR(TIMER1_OVF_vect) 
{
  TIMSK1 = 0x00;
  newposition_1 = left_count;
  newposition_2 = right_count;
  process_encoder();
  oldposition_1=newposition_1;
  oldposition_2=newposition_2;
  TCNT1 = 0xFF70; // 0.01s
  TIMSK1 = 0x01; 
  }
/**********************************
Function name	:	low_pass_filter
Functionality	:	low pass filter for accelerometer data
Arguments		:	raw value of ax,ay and az
Return Value	:	filtered ax,ay and az value
Example Call	:	low_pass_filter(ax,ay,az)
***********************************/
void low_pass_filter(int16_t ax,int16_t ay,int16_t az){ //low pass filter for accelerometer data
  float dT = 0.01;
  float Tau= 1/(2*3.1416*f_cut);
  float alpha = Tau/(Tau+dT);//0.031;
  float ax2=(float)ax;
  float ay2=(float)ay;
  float az2=(float)az;
  AX1=(1-alpha)*ax2+alpha*old1;
  old1 = AX1;
  AY1=(1-alpha)*ay2+alpha*old2;
  old2 = AY1;
  AZ1=(1-alpha)*az2+alpha*old3;
  old3 = AZ1;
  AX1 = (AX1)/16384;
  AY1 = (AY1)/16384;
  AZ1 = (AZ1)/16384;
}
/**********************************
Function name	:	high_pass_filter
Functionality	:	high pass filter for gyroscope data
Arguments		:	raw value of gx,gy and gz
Return Value	:	filtered gx,gy and gz value
Example Call	:	low_pass_filter(gx,gy,gz)
***********************************/
void high_pass_filter(int16_t gx,int16_t gy,int16_t gz){ //high pass filter for gyroscope data
  float dT = 0.01;
  float Tau= 1/(2*3.141*f_cut);
  float alpha = Tau/(Tau+dT);
  float gx2=(float)gx;
  float gy2=(float)gy;
  float gz2=(float)gz;
  new_gx = gx;
  GX_N = (1-alpha)*GX_O + (1-alpha)*(new_gx - old_gx);
  old_gx = new_gx;
  GX_O = GX_N;
  new_gy = gy;
  GY_N = (1-alpha)*GY_O + (1-alpha)*(new_gy - old_gy);
  old_gy = new_gy;
   GY_O = GY_N;
  new_gz = gz;
  GZ_N = (1-alpha)*GZ_O + (1-alpha)*(new_gz - old_gz);
  old_gz = new_gz;
  GZ_O = GZ_N;
  GX_N=GX_N/131;
  GY_N=GY_N/131;
  GZ_N=GZ_N/131;
}
/**********************************
Function name	:	comp_pitch
Functionality	:	First order complimentary filter for sensor fusion
Arguments		:	Sensor data to fuse
Return Value	:	Fused value
Example Call	:	comp_pitch(ax,ay,az,gx,gy,gz)
***********************************/
void comp_pitch(float ax,float ay,float az,float gx,float gy,float gz){ // this fucntion calculates pitch angle
  alpha1 = 0.08;
  pitch_acc =atan2f(-ax,abs(az));
  pitch_acc=(pitch_acc*180)/3.14;
  pitch_gyro =(gy*0.01);
  new_pitch_angle = (1-alpha1)*(old_pitch_angle+pitch_gyro) + (alpha1*pitch_acc);
  old_pitch_angle = new_pitch_angle;
  new_pitch_angle = new_pitch_angle*(-1);
  new_pitch_angle = (new_pitch_angle)/57*3;
}
/**********************************
Function name	:	balance_using_motor
Functionality	:	used to constrain the lqr value in the range of pwm ie.(0-255)
Arguments		:	input from lqr controller
Return Value	:	constrained value of pwm value
Example Call	:	balance_using_motor(input_1,input_2)
***********************************/
void balance_using_motor(float input_1, float input_2){//this function takes u as input maps it and depending on its sign moves motors in particular directions and adjust their speed
 
  uint8_t VR=(constrain((abs(input_1)), 0, 180));
  uint8_t VL=(constrain((abs(input_2)), 00,150));
 if(input_1>0){
   motorForwardR(VL);   
}
 else if(input_1<0){
   motorBackwardR(VL);   
 }
 if(input_2<0){
   motorBackwardL(VR);
}
 else if(input_2>0){
   motorForwardL(VR); 
 }
}
/**********************************
Function name	:	process_encoder
Functionality	:	function to process the ticks from encoder
Arguments		:	None
Return Value	:	velocity
Example Call	:	process_encoder()
***********************************/
void process_encoder()
{
   v0 = (((newposition_1-oldposition_1)/(0.02*270))*(2*3.14*0.035));//left count
   v1 = -1*(((newposition_2-oldposition_2)/(0.02*270))*(2*3.14*0.035));//right count
   vel = (v0 + v1)*0.5;
   vela=vela +vel*0.02;
}
/**********************************
Function name	:	lqr
Functionality	:	Finding the value of u (using u=-k.x)
Arguments		:	sensor and encoder data
Return Value	:	u value
Example Call	:	lqr()
***********************************/
void lqr(){ // this function takes values of encoders and mpu6050 sensor as x and theta and solves equation u=-kx and pass value of u as pwm for motor controlling
  y_setpoint=0;
  newposition_1 = left_count;
  newposition_2 = right_count;
  process_encoder();
  oldposition_1=newposition_1;
  oldposition_2=newposition_2;
  ul=(-1*((vela*(-7.67037)) + (vel*(-12.10118))+(theta*(-53.42332))+(theta_dot*(-5.47997))));
  ul=ul*5;
  balance_using_motor(ul,ul);
  }
  /**********************************
Function name	:	handle_flags()
Functionality	:	handle the different flag condition
Arguments		:	None
Return Value	:	None
Example Call	:	handle_flags()
***********************************/
  void handle_flags()
  {
    if(flagstop==1)
    {
      left_motor(0,0);
      right_motor(0,0);
      }
    if (flagright==1)
    {
      right(0,1);
      }
    if (flagleft==1)
    {
      left(1,0);
    }
    if (flagforward==1)
    {
      left_motor(1,0);
      right_motor(0,1);
 }
    if (flagbackward==1)
    {
      left_motor(0,1);
    right_motor(1,0);
    }
    flagright=0;
    flagleft=0;
    flagforward=0;
    flagbackward=0;
    flagstop=0;
    }
/**********************************
Function name	:	read_joystick
Functionality	:	To read the data from the controller XBee Module and store it
Arguments		:	None
Return Value	:	None
Example Call	:	read_joystick()
***********************************/
void joystick_read()
{
    //int sum = 131;
    //unsigned char byte_discard, checksum;
    //unsigned char digital_data = 0;
    //unsigned char AD0[2] = {0, 0}, AD1[2] = {0, 0};
if (Serial3.read() == 0x7E) {
    detachInterrupt(0);
    detachInterrupt(1);
    // 7E is the start byte
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
if (analogReading>=0 && analogReading<1024 && analog1Reading>=0 && analog1Reading<1024)
{
   a = analogReading;
   b = analog1Reading;  
}

  if (a==1023 || b==1023)
{ 
 flagstop=1;
flagbackward=0;
flagforward=0;
flagleft=0;
flagright=0;
}
 if (a>511 && a<1020)
{
  flagstop=0;
flagbackward=0;
flagforward=1;
flagleft=0;
flagright=0;
}
else if (a<541)
{
  flagstop=0;
flagbackward=1;
flagforward=0;
flagleft=0;
flagright=0;
}
else if( b>540 && b<1020)
{
  flagstop=0;
flagbackward=0;
flagforward=0;
flagleft=1;
flagright=0;
}
else if( b<240)
{
  flagstop=0;
flagbackward=0;
flagforward=0;
flagleft=0;
flagright=1;
}
}    
}
/**********************************
Function name	:	left_motor
Functionality	:	To set the directin of left motor 
Arguments		:	x1 and x2
Return Value	:	pin value of left motor
Example Call	:	left_motor(x1,x2)
***********************************/
void left_motor(int x1,int x2)
{
if (x1==0 && x2==0)
{digitalWrite(InL1,LOW);
digitalWrite(InL2,LOW);
analogWrite(PWML,0);}
else if (x1==1 && x2==0)
{
digitalWrite(InL1,HIGH);
digitalWrite(InL2,LOW);
analogWrite(PWML,255);}
else if (x1==0 && x2==1)
{digitalWrite(InL1,LOW);
digitalWrite(InL2,HIGH);
analogWrite(PWML,255);}
}
/**********************************
Function name	:	right_motor
Functionality	:	To set the directin of left motor 
Arguments		:	y1 and y2
Return Value	:	pin value of right motor
Example Call	:	right_motor(y1,y2)
***********************************/
void right_motor(int y1,int y2)
{
if (y1==0 && y2==0)
{
digitalWrite(InR1,LOW);
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
/**********************************
Function name	:	left
Functionality	:	To set the directin of left motor initially
Arguments		:	a1 and a2
Return Value	:	pin value of left motor
Example Call	:	left(a1,a2)
***********************************/
void left(int a1,int a2)
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
/**********************************
Function name	:	right
Functionality	:	To set the directin of right motor initially 
Arguments		:	b1 and b2
Return Value	:	pin value of right motor
Example Call	:	right(b1,b2)
***********************************/
void right(int b1,int b2)
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
