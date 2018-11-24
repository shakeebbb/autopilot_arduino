#include <Servo.h>
#include <math.h>
#include "TinyGPS++.h"
TinyGPSPlus gps;

///////////////////       PID CLASS     ////////////////////
typedef struct
{
  float kp;//1.268;//1.120;
  float ki;//0.007;//0.019;
  float kd;//85;
  float currInput;
  float prevInput;
  float currRef;
  float propErr;
  float sumErr;
  double diffErr;
  float output;  
}PID;

PID& pid_fun(float, float, PID&);

float gps_thresh = 600;
float maxHeight = 300;
int Lidar_flag = 0;
float roll = 0;
float pitch = 0;
float yaw = 0;
float height = 0;
float lati;
float longi;
float roll_prev = 0;
float pitch_prev = 0;
float yaw_prev = 0;
float height_prev = 0;
int test_count = 0;
int timestart;
int timestop;
int kill = 1;  // Kill=1 => maunual
float hover_rpm = 6000;//5840;

// Assign your channel in pins
#define LidarPwmPin 34
#define LidarTrigPin 13
#define CHANNEL1_IN_PIN 52
#define CHANNEL2_IN_PIN 50
#define CHANNEL3_IN_PIN 48
#define CHANNEL4_IN_PIN 46
#define CHANNEL5_IN_PIN 45
#define CHANNEL6_IN_PIN 42

#define CHANNEL7_IN_PIN 40
#define CHANNEL8_IN_PIN 38
// These bit flags are set in bUpdateFlagsShared to indicate which
// channels have new signals
#define CHANNEL1_FLAG 1
#define CHANNEL2_FLAG 2
#define CHANNEL3_FLAG 4
#define CHANNEL4_FLAG 8
#define CHANNEL5_FLAG 16
#define CHANNEL6_FLAG 32
#define CHANNEL7_FLAG 64
#define CHANNEL8_FLAG 128      //LIDAR
// holds the update flags defined above
volatile uint32_t bUpdateFlagsShared;

// shared variables are updated by the ISR and read by loop.
// In loop we immediatley take local copies so that the ISR can keep ownership of the
// shared ones. To access these in loop
// we first turn interrupts off with noInterrupts
// we take a copy to use in loop and the turn interrupts back on
// as quickly as possible, this ensures that we are always able to receive new signals
volatile uint32_t unChannel1InShared;
volatile uint32_t unChannel2InShared;
volatile uint32_t unChannel3InShared;
volatile uint32_t unChannel4InShared;
volatile uint32_t unChannel5InShared;
volatile uint32_t unChannel6InShared;
volatile uint32_t unChannel7InShared;
volatile uint32_t unChannel8InShared;


/*
//////////////////// Roll PID Variables ////////////////////
float currInputR,prevInputR;
float currRefR=0;
float prevRefR=0;
float currErrR=0;
double dErrR=0;
float propErrR;
float sumErrR=0;
double diffErrR=0;
float iTermsR=0;
double delErrR=0;
float outputR=0;
float kp_roll=0;//1.268;//1.120;
float ki_roll=0;//0.007;//0.019;
float kd_roll=100;//85;
int numR=0;
*/

float ref_roll = 0;
float ref_pitch = 0;
float ref_yaw = 0;
float ref_height=0;
//float ref_lati2 = 33.622593;//33.622791;//33.622993;//33.622944;
//float ref_longi2 = 72.961975;//72.961761;//72.961449;//72.959618;

//float ref_lati;//33.622849;//33.622993;//33.622944;
//float ref_longi; //72.961617;//72.961449;//72.959618;

int curr_coord_num = 0;
float ref_lati_array[4] = {33.623032,33.623032,33.623032,33.623032};//{33.622807,33.622807,33.622410,33.622475}; cmdt ground
float ref_longi_array[4] = {72.958351,72.958351,72.958351,72.958351};//{72.962021,72.961548,72.961586,72.962074};

float ref_lati = ref_lati_array[0];
float ref_longi = ref_longi_array[0];

////////////////// Motor values ////////////////////////
float motorL, motorR, motorF, motorB;

Servo MotorF;
Servo MotorR;
Servo MotorL;
Servo MotorB;

char buff[4];

int thrust = 1000;
float RPM = 0;

//////////////////// PID Objects //////////////////////////
PID Roll = {13.9,0.005,1182,0,0,0,0,0,0,0};
PID Pitch ={12.94,0.007,1168.5,0,0,0,0,0,0,0};
PID Yaw = {13.9,0.005,1182,0,0,0,0,0,0};
PID Height = {5.44,0.006,900,0,0,0,0,0,0,0};
PID Lati = {400,0,0,0,0,0,0,0,0,0};
PID Longi = {400,0,0,0,0,0,0,0,0,0};
///////////////////////////////////////////////////////////

double kalman_q = 0.001; //process noise covariance
double kalman_r = 0.1; //measurement noise covariance
double kalman_x = 0.00; //value
double kalman_p = 1; //estimation error covariance
double kalman_k = 2; //kalman gain
unsigned long pulse_width;

void setup() {
  Serial1.begin(115200);
  Serial3.begin(115200);
  Serial.begin(9600);  
     
  MotorR.attach(5);
  MotorF.attach(4);
  MotorL.attach(3);
  MotorB.attach(2);
  
  MotorF.writeMicroseconds(2000);// To initialize ESCs
  MotorB.writeMicroseconds(2000);
  MotorR.writeMicroseconds(2000);
  MotorL.writeMicroseconds(2000);
  delay(1000);
  MotorF.writeMicroseconds(1000);
  MotorB.writeMicroseconds(1000);
  MotorR.writeMicroseconds(1000);
  MotorL.writeMicroseconds(1000);
  delay(2000);
  MotorF.writeMicroseconds(1000); 
  MotorB.writeMicroseconds(1000); 
  MotorR.writeMicroseconds(1000); 
  MotorL.writeMicroseconds(1000);
  
       // attach the interrupts used to read the channels
  attachInterrupt(CHANNEL1_IN_PIN, calcChannel1,CHANGE);
  attachInterrupt(CHANNEL2_IN_PIN, calcChannel2,CHANGE);
  attachInterrupt(CHANNEL3_IN_PIN, calcChannel3,CHANGE);
  attachInterrupt(CHANNEL4_IN_PIN, calcChannel4,CHANGE);
  attachInterrupt(CHANNEL5_IN_PIN, calcChannel5,CHANGE);
  attachInterrupt(CHANNEL6_IN_PIN, calcChannel6,CHANGE);
  attachInterrupt(CHANNEL7_IN_PIN, calcChannel7,CHANGE);
  attachInterrupt(CHANNEL8_IN_PIN, calcChannel8,CHANGE);
  attachInterrupt(LidarPwmPin,calcLidar,CHANGE);
  
  pinMode(LidarTrigPin, OUTPUT); // Set pin 13 as trigger pin
  //pinMode(LidarPwmPin, INPUT); // Set pin 34 as monitor pin
  digitalWrite(LidarTrigPin, LOW); // Set trigger LOW for continuous read
  
  
  ref_roll=0; 
  ref_pitch=0; 
  ref_yaw=0; 
  ref_height=0;
}
void loop()
{
  //Serial.println();
  if (Serial.available()) 
  {
   buff[0] = Serial.read();
   if(buff[0] == 'k')
   {
     thrust = 1000;
   MotorF.writeMicroseconds(thrust); 
   MotorR.writeMicroseconds(thrust); 
   MotorB.writeMicroseconds(thrust); 
   MotorL.writeMicroseconds(thrust); 
  Serial.println("ALARM !!!!!!!!!!");
  while(1);
   }
   else if(buff[0] == 'r')
   {
   Serial.readBytesUntil('/', buff, 4);
   thrust = atoi(buff); 
   MotorF.writeMicroseconds(thrust); 
   MotorR.writeMicroseconds(thrust); 
   MotorB.writeMicroseconds(thrust); 
   MotorL.writeMicroseconds(thrust); 
   }
   
  }

  
  //////////////////////////////////////////////////////////////
  
    // create local variables to hold a local copies of the channel inputs
  // these are declared static so that thier values will be retained
  // between calls to loop.
  static uint32_t unChannel1In;
  static uint32_t unChannel2In;
  static uint32_t unChannel3In;
  static uint32_t unChannel4In;
  static uint32_t unChannel5In;
  static uint32_t unChannel6In;
  static uint32_t unChannel7In;
  static uint32_t unChannel8In;
 
  // local copy of update flags
  static uint32_t bUpdateFlags;
  
  // check shared update flags to see if any channels have a new signal
  if(bUpdateFlagsShared)
  {
    noInterrupts(); // turn interrupts off quickly while we take local copies of the shared variables

    // take a local copy of which channels were updated in case we need to use this in the rest of loop
    bUpdateFlags = bUpdateFlagsShared;
  
    // in the current code, the shared values are always populated
    // so we could copy them without testing the flags
    // however in the future this could change, so lets
    // only copy when the flags tell us we can.
  
    if(bUpdateFlags & CHANNEL1_FLAG)
    {
      unChannel1In = unChannel1InShared;
    }
  
    if(bUpdateFlags & CHANNEL2_FLAG)
    {
      unChannel2In = unChannel2InShared;
    }
  
    if(bUpdateFlags & CHANNEL3_FLAG)
    {
      unChannel3In = unChannel3InShared;
    }
   
    if(bUpdateFlags & CHANNEL4_FLAG)
    {
      unChannel4In = unChannel4InShared;
    }
  
    if(bUpdateFlags & CHANNEL5_FLAG)
    {
      unChannel5In = unChannel5InShared;
    }
  
    if(bUpdateFlags & CHANNEL6_FLAG)
    {
      unChannel6In = unChannel6InShared;
    }
   
    if(bUpdateFlags & CHANNEL7_FLAG)
    {
      unChannel7In = unChannel7InShared;
    }
  
    if(bUpdateFlags & CHANNEL8_FLAG)
    {
      unChannel8In = unChannel8InShared;
    }
    // clear shared copy of updated flags as we have already taken the updates
    // we still have a local copy if we need to use it in bUpdateFlags
    bUpdateFlagsShared = 0;
  
    interrupts(); // we have local copies of the inputs, so now we can turn interrupts back on
    // as soon as interrupts are back on, we can no longer use the shared copies, the interrupt
    // service routines own these and could update them at any time. During the update, the
    // shared copies may contain junk. Luckily we have our local copies to work with :-)
  }
 
  // do any processing from here onwards
  // only use the local values unChannel1, unChannel2, unChannel3, unChannel4, unChannel5, unChannel6, unChannel7, unChannel8
  // variables unChannel1InShared, unChannel2InShared, etc are always owned by the
  // the interrupt routines and should not be used in loop
 
  if(bUpdateFlags & CHANNEL1_FLAG) //Roll
  {
      // remove the // from the line below to implement pass through updates to the servo on this channel -
      //servoChannel1.writeMicroseconds(unChannel1In);
      //Serial.println();
     // Serial.print(unChannel1In);
     // Serial.print(",");
    if(unChannel1In < 1200)
    unChannel1In = 1200;
    if(unChannel1In > 1700)
    unChannel1In = 1700;
    
    if(height<gps_thresh)
    ref_roll = map(unChannel1In,1200,1700,7,-7);
 //   Serial.print(ref_roll);
 //   Serial.print(" , ");
  }
  
  if(bUpdateFlags & CHANNEL2_FLAG) //Pitch
  {
      // remove the // from the line below to implement pass through updates to the servo on this channel -
      //servoChannel2.writeMicroseconds(unChannel2In);
      //Serial.println(unChannel2In);
      //Serial.print(",");

    if(unChannel2In < 1200)
    unChannel2In = 1200;
    if(unChannel2In > 1700)
    unChannel2In = 1700;
    
    if(height<gps_thresh)
    ref_pitch = map(unChannel2In,1200,1700,-7,7);

   // Serial.print(ref_pitch);
  //  Serial.print(" , ");
  }

  if(bUpdateFlags & CHANNEL3_FLAG) //Thrust
  {
      // remove the // from the line below to implement pass through updates to the servo on this channel -
      //servoChannel2.writeMicroseconds(unChannel2In);
      //Serial.print(unChannel3In);
      //Serial.print(",");
      thrust = unChannel3In;
      if(thrust<1150)
      {
      thrust = 1000;
      RPM = 500;
      }
      if(thrust>1800)
      thrust = 1800;
      
    if(kill == 1) //Manual
    {
    //RPM = map(thrust,1100,1800,1200,4500);
    RPM = map(thrust,1150,1800,3000,6200);
    //Serial.println(RPM);
    }
      
  }


  if(bUpdateFlags & CHANNEL4_FLAG) //Yaw
  {
    // remove the // from the line below to implement pass through updates to the servo on this channel -
    // servoChannel4.writeMicroseconds(unChannel4In);
    //Serial.print(unChannel4In);
    //Serial.print(",");

    if(unChannel4In < 1200)
    unChannel4In = 1200;
    if(unChannel4In > 1700)
    unChannel4In = 1700;
    
    ref_yaw = map(unChannel4In,1200,1700,-30,30);
    //Serial.println(ref_yaw);
  }
 
  if(bUpdateFlags & CHANNEL5_FLAG) //Kp
  {
    // remove the // from the line below to implement pass through updates to the servo on this channel -
    // servoChannel5.writeMicroseconds(unChannel5In);
    //Serial.print("Channel 5 : ");
    //Serial.print(unChannel5In);
    //Serial.print(",");

     // Yaw.kp = unChannel5In;
     // Yaw.kp = 0.020*(Yaw.kp) - 20;  
  }
 
  if(bUpdateFlags & CHANNEL6_FLAG) //Kd
  {
    // remove the // from the line below to implement pass through updates to the servo on this channel -
    // servoChannel6.writeMicroseconds(unChannel6In);
    //Serial.print(unChannel6In);
    //Serial.print("Channel 6 : ");
    //Serial.print(unChannel6In);
    //Serial.print(",");

   //Yaw.kd = unChannel6In;
   //Yaw.kd = 1.5000*Yaw.kd - 1500;
   
   //hover_rpm = unChannel6In;
   //hover_rpm = 0.05*hover_rpm - 50 + 3120;
   
   
  }
 
  if(bUpdateFlags & CHANNEL7_FLAG) //Kill
  {
    // remove the // from the line below to implement pass through updates to the servo on this channel -
    // servoChannel7.writeMicroseconds(unChannel7In);
    //Serial.print(unChannel7In);
    //Serial.print(",");
    
    
     if(unChannel7In>1500)
     kill = 0;                 //Auto
     else
     kill = 1;                 //Manual
    
    
   //Yaw.ki = unChannel7In;
   //Yaw.ki = 0.0001*Yaw.ki - 0.1;
    
  }
 
  if(bUpdateFlags & CHANNEL8_FLAG)
  {
    // remove the // from the line below to implement pass through updates to the servo on this channel -
    // servoChannel8.writeMicroseconds(unChannel8In);
   // Serial.println(unChannel8In);
   //pulse_width = unChannel8In;

//    Serial.print(",");
  }
  if(Lidar_flag)
  {
    Lidar_flag = 0;
    if (kill==0) //Auto
    {
    ref_height = map(thrust,1150,1800,0,maxHeight);
    //Serial.print("Ref Height : ");
    //Serial.println(ref_height);
    auto_control();
    }
  }

  
  
    /*
      Serial.print("Kp = ");
      Serial.print(Yaw.kp,5);  
   
      Serial.print(" , ");   
      Serial.print("Kd = ");
      Serial.print(Yaw.kd,5);
      Serial.print(" , "); 
      Serial.print("Ki = ");
      Serial.print(Yaw.ki,5);
      Serial.print(" , "); 
      Serial.print("Hover RPM : ");
      Serial.print(hover_rpm);
      Serial.print(" , ");
      Serial.print("RPM : ");
      Serial.print(RPM);
      Serial.println(" , ");
   */
 
 
  bUpdateFlags = 0;
  
  /*
   if(test_count == 0)
  {
  timestart = millis();
  test_count = 1;
  }
  else if(test_count == 1)
  {
  timestop = millis();
  test_count = 2;
  }
  else if(test_count == 2)
  {
  Serial.println(timestop-timestart);
  test_count = 0;
  }
  */
  
 // Serial.println(millis());
  
  
  
  //Serial.println();
  
 /////////////////////////////////////////////////GPS Code//////////////////////////////////
       while (Serial3.available() > 0)
    gps.encode(Serial3.read());
    
   if (gps.location.isUpdated())
  {
    lati = gps.location.lat();
    longi = gps.location.lng();
    //Serial.print("LAT="); Serial.print(lati, 6);
    //Serial.print("LNG="); Serial.println(longi, 6);
    Lati = pid_fun(lati,ref_lati,Lati);
    Longi = pid_fun(longi,ref_longi,Longi);
  /*  
  Serial.print(Lati.output,6);
  Serial.print(",");
  Serial.print(Longi.output,6);
  Serial.print(",");
  */
    
    Lati.output = asin(Lati.output)*180/3.14;
    Longi.output = asin(Longi.output)*180/3.14;
    Lati.output = -1*Lati.output;
    
    if(height>=gps_thresh)
    {
    ref_roll = Longi.output;
    ref_pitch = Lati.output;
    
    if(ref_pitch > 5)  //+ pitch
    ref_pitch = 5;
    if(ref_pitch < -7)   //-pitch
    ref_pitch = -7;
    if(ref_roll < -5)  //- roll
    ref_roll = -5;
    if(ref_roll > 5)   //+roll
    ref_roll = 5;
    
    if(Longi.output<3.5 && Longi.output>-3.5 && Lati.output<3.5 && Lati.output>-3.5)
    {
      if(curr_coord_num == 3)
      curr_coord_num = 3;
      else
      curr_coord_num = curr_coord_num + 1;
      
    ref_lati = ref_lati_array[curr_coord_num];
    ref_longi = ref_longi_array[curr_coord_num];
    }
    //Serial.print("opqr");
    } 
    //Serial.println(height);
    

  }
  /*
  Serial.print(curr_coord_num);
  Serial.print(",");
  Serial.print(ref_lati,6);
  Serial.print(",");
  Serial.print(ref_longi,6);
  Serial.print(",");
  Serial.print(ref_roll,6);
  Serial.print(",");
  Serial.print(ref_pitch,6);
  Serial.print(",");
  Serial.print(RPM);
  Serial.print(",");
  Serial.println(height);
  */
  
  ////////////////////////////////////////////////////////////////////////////////////
  
 
  imu(); 
  
  //height = height_sense(); 
  
  //ref_pitch = ref_pitch-3;
  Roll = pid_fun(roll,ref_roll,Roll);
  Pitch = pid_fun(pitch,ref_pitch,Pitch);
  Yaw = pid_fun(yaw,ref_yaw,Yaw);
  
  //float outputR_new=1287*outputR/160;
  //float outputR_new=1*outputR;
  
  //Serial.println(RPM);
  motorL=RPM+Roll.output-Yaw.output;
  motorR=RPM-Roll.output-Yaw.output;
  
  motorF=RPM+Pitch.output+Yaw.output;
  motorB=RPM-Pitch.output+Yaw.output;
  
  /*
  Serial.print("motorL : ");
  Serial.print(motorL);
  Serial.print(" , ");
  Serial.print("motorR : ");
  Serial.print(motorR);
  Serial.print(" , ");
  */
  
  /*
  Serial.print("motorB : ");
  Serial.print(motorB);
  Serial.print(" , ");
  Serial.print("motorF : ");
  Serial.print(motorF);
  */
  
  //float thrustL = motorL;
  //float thrustR = motorR;
  float thrustL=0.00002235*motorL*motorL+0.010386*motorL+955.84;
  float thrustR=0.000019857*motorR*motorR+0.031049*motorR+918.78;
  
  float thrustB=0.000021246*motorB*motorB+0.020337*motorB+946.46;
  float thrustF=0.000019246*motorF*motorF+0.042609*motorF+898.63;
  
  //float thrustR=0.00002152*motorR*motorR-0.01643*motorR+1070;
 
  //Serial.println();
  

  
 
  //Serial.print(" , ");
  
  if(thrustR<1100)
  {
  thrustR=1000;
  }
  if(thrustL<1100)
  {
  thrustL=1000;
  }
  if(thrustL>2000)
  {
  thrustL=2000;
  }
  if(thrustR>2000)
  {
  thrustR=2000;
  }
  
  
  if(thrustF<1100)
  {
  thrustF=1000;
  }
  if(thrustB<1100)
  {
  thrustB=1000;
  }
  if(thrustF>2000)
  {
  thrustF=2000;
  }
  if(thrustB>2000)
  {
  thrustB=2000;
  }
  
  if (thrust < 1150)
  {
  thrustL = 1000;
  thrustR = 1000;
  thrustB = 1000;
  thrustF = 1000;
  Serial.println("killed");
  }

  MotorL.writeMicroseconds(thrustL);
  MotorR.writeMicroseconds(thrustR);
  
  MotorF.writeMicroseconds(thrustF);
  MotorB.writeMicroseconds(thrustB);
  
  /*
  Serial.print(" ThrustB : ");
  Serial.print(thrustB);
  Serial.print(" , ");
  Serial.print("ThrustF : ");
  Serial.println(thrustF);
  */
  // Serial.print("I am running at RPM : ");
  // Serial.println(power);

}


 ///////////////////////////////////Height Sense////////////////////////////////
  void auto_control(void)
{
  
  height_sense();
  height = height*cos(roll*3.14/180)*cos(pitch*3.14/180);
  //Serial.println(height);
  
  Height = pid_fun(height,ref_height,Height);
  if(Height.output>150)
  Height.output = 150;
  if(Height.output<-150)
  Height.output = -150;
  
  RPM = hover_rpm-Height.output;
  //RPM = RPM / ( cos(roll*3.14/180) * cos(pitch*3.14/180) );
  
  /*
  Serial.print(ref_height);
  Serial.print(" , ");
  Serial.print(height);
  Serial.print(" , ");
  Serial.println(RPM);
  */
  
}


void height_sense(void)
{
  //pulse_width = pulseIn(LidarPwmPin, HIGH); // Count how long the pulse is high in microseconds
  //Serial.println(pulse_width);
  if(pulse_width != 0){ // If we get a reading that isn't zero, let's print it
        pulse_width = pulse_width/10; // 10usec = 1 cm of distance for LIDAR-Lite
        kalman_update(pulse_width);
        height = kalman_x-25;

  //	Serial.println(height); // Print the distance
  
  //Serial.println("ME");
 }
  
  //////////////////////////////////////////////////////////////////////////////////////////////////////
     
}
void kalman_init(double p,double q,double r,double x)
{
   kalman_q = q; //process noise covariance
   kalman_r = r; //measurement noise covariance
   kalman_x = x; //value
   kalman_p = p; //estimation error covariance
}
void kalman_update(double measurement)
{
  //prediction update
  //omit x = x
  kalman_p = kalman_p + kalman_q;

  //measurement update
  kalman_k = kalman_p / (kalman_p + kalman_r);
  kalman_x = kalman_x + kalman_k * (measurement - kalman_x);
  kalman_p = (1 - kalman_k) * kalman_p;
}

PID& pid_fun(float input, float ref, PID& parameter)
{
  float currErr = 0;
  float dErr = 0;
  float iTerms = 0;
  
  
//currInput=input-5;
parameter.currInput=input;
parameter.currRef=ref;
currErr=parameter.currInput-parameter.currRef;
//Serial.print("currErrR : ");
//Serial.println(currErrR);
//dErr=currRef-prevRef-currInput+prevInput;
dErr=parameter.prevInput-parameter.currInput;
iTerms=parameter.ki*currErr;
parameter.propErr=parameter.kp*currErr;
/*
 Serial.print(" P_Err: ");
 Serial.print(propErrR);
 Serial.print(" , ");
*/
parameter.sumErr += iTerms;
parameter.diffErr=parameter.kd*dErr;
/*
 Serial.print(" Sum_ErrR: ");
 Serial.print(sumErrR);
 Serial.print(" , ");
*/ 
parameter.diffErr=constrain(parameter.diffErr,-500,500);
parameter.output=parameter.propErr-parameter.diffErr+parameter.sumErr;
parameter.prevInput=parameter.currInput;

int outputMax = 700;
int outputMin = -700;
int sumErrMax = 200;
int sumErrMin = -200;

/*
Serial.print(" OutputR: ");
Serial.print(outputR);
Serial.print(" , ");
*/


if(parameter.output > outputMax)
{
parameter.output=outputMax;
//Serial.println("OM");
}
if(parameter.output < outputMin)
{
parameter.output=outputMin;
//Serial.println("OM");
}

if(parameter.sumErr > sumErrMax) 
{
//Serial.println("SM");
parameter.sumErr=sumErrMax;
}
if(parameter.sumErr < sumErrMin)
{
//Serial.println("SM");
parameter.sumErr=sumErrMin;
}

return parameter;
}


void imu() {
  
  //int datalength = 20;  //112-116 (5 Registers) 
  int i = -1;
  char ch = 0;
  int count = 0;
  uint16_t temp;
  uint8_t in[20] = {NULL};  // 's','n','p',PT,Address,Data,Checksum[1],Checksum[0]

  while(1)
  {
  i = Serial1.read();
  if(i == -1);
  //Serial.println("No Data");
  
  else
  {
    //Serial.println("SUCCESS");
    ch = i;
    //Serial.println(ch);
    if(in[0] == 's' && in[1] == 'n' && in[2] =='p')
    {
      count = count+1;
      in[count] = ch;
    }
    else if(ch == 's')
    {
    count = 0;
    in[count] = 's';
    }
    else if(ch == 'n')
    {
      count = 1;
      in[count] = 'n';
    }
    else if(ch == 'p')
    {
      count = 2;
      in[count] = 'p';
    }
  }
  
  if(count == 17)
  {
    count = 0;
  /*  
    Serial.print("PT : ");
    Serial.print(in[3]);
    Serial.print("  A : ");
    Serial.print(in[4]);
    Serial.print("  R[1] : ");
    Serial.print(in[5]);
    Serial.print("  R[0] : ");
    Serial.print(in[6]);
    Serial.print("  P[1] : ");
    Serial.print(in[7]);
    Serial.print("  P[0] : ");
    Serial.print(in[8]);
    Serial.print("  Y[1] : ");
    Serial.print(in[9]);
    Serial.print("  Y[0] : ");
    Serial.println(in[10]);
    */
    
    if(in[4] == 112)
    {
    
    temp = in[5] * 256 + in[6];
    roll = ((int16_t) temp) / 91.02222; 
    
    temp = in[7] * 256 + in[8];
    pitch = ((int16_t) temp) / 91.02222 ; 
    
    temp = in[9] * 256 + in[10];
    yaw = ((int16_t) temp) / 91.02222;
 
    if((roll-roll_prev)<-50 || (roll-roll_prev)>50)
    {
      roll = roll_prev;
    }
    roll_prev = roll;
    
    if((pitch-pitch_prev)<-50 || (pitch-pitch_prev)>50)
    {
      pitch = pitch_prev;
    }
    pitch_prev = pitch;
    
    if((yaw-yaw_prev)<-50 || (yaw-yaw_prev)>50)
    {
      yaw = yaw_prev;
    }
    yaw_prev = yaw;


    
    //Serial.println(millis());
    
    //temp = in[13] * 256 + in[14];
    //int16_t rollrate = (int16_t) temp;
    /*
    if(roll > 90)
    roll = 360-roll;
    if(roll < -90)
    roll = 360+roll;
    */
    
    //Serial.println("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
    
    //Serial.print(" , ");
    //Serial.print(" Roll : ");
//    Serial.print(roll);
//    Serial.print(" , ");
    
    //Serial.print("ref_roll : ");
    //Serial.println(ref_roll);
    
    
    //Serial.print(" , ");
    //Serial.print(" Pitch : ");
 //   Serial.print(pitch);
    
  //  Serial.print(" , ");
    //Serial.print(" Yaw : ");
 //   Serial.println(yaw);
    //Serial.print(" , ");
    
    //Serial.print("ROLL RATE : ");
    //Serial.println(rollrate = rollrate/16.0);
    //in = {NULL};
    
    break;      // Break if Roll,Pitch,Yaw Acquired
    }   
    for(i=0 ; i<20 ; i++) 
    in[i] = NULL;
  }
  }
   
}

////////////////////////////        CHANNEL ISRs      /////////////////////////////
void calcChannel1()
{
  static uint32_t ulStart;
 
  if(digitalRead(CHANNEL1_IN_PIN))
  {
    ulStart = micros();
  }
  else
  {
    unChannel1InShared = (uint32_t)(micros() - ulStart);
    bUpdateFlagsShared |= CHANNEL1_FLAG;
  }
}

void calcChannel2()
{
  static uint32_t ulStart;
 
  if(digitalRead(CHANNEL2_IN_PIN))
  {
    ulStart = micros();
  }
  else
  {
    unChannel2InShared = (uint32_t)(micros() - ulStart);
    bUpdateFlagsShared |= CHANNEL2_FLAG;
  }
}

void calcChannel3()
{
  static uint32_t ulStart;
 
  if(digitalRead(CHANNEL3_IN_PIN))
  {
    ulStart = micros();
  }
  else
  {
    unChannel3InShared = (uint32_t)(micros() - ulStart);
    bUpdateFlagsShared |= CHANNEL3_FLAG;
  }
}

void calcChannel4()
{
  static uint32_t ulStart;
 
  if(digitalRead(CHANNEL4_IN_PIN))
  {
    ulStart = micros();
  }
  else
  {
    unChannel4InShared = (uint32_t)(micros() - ulStart);
    bUpdateFlagsShared |= CHANNEL4_FLAG;
  }
}

void calcChannel5()
{
  static uint32_t ulStart;
 
  if(digitalRead(CHANNEL5_IN_PIN))
  {
    ulStart = micros();
  }
  else
  {
    unChannel5InShared = (uint32_t)(micros() - ulStart);
    bUpdateFlagsShared |= CHANNEL5_FLAG;
  }
}

void calcChannel6()
{
  static uint32_t ulStart;
 
  if(digitalRead(CHANNEL6_IN_PIN))
  {
    ulStart = micros();
  }
  else
  {
    unChannel6InShared = (uint32_t)(micros() - ulStart);
    bUpdateFlagsShared |= CHANNEL6_FLAG;
  }
}

void calcChannel7()
{
  static uint32_t ulStart;
 
  if(digitalRead(CHANNEL7_IN_PIN))
  {
    ulStart = micros();
  }
  else
  {
    unChannel7InShared = (uint32_t)(micros() - ulStart);
    bUpdateFlagsShared |= CHANNEL7_FLAG;
  }
}

void calcChannel8()
{
  static uint32_t ulStart;
 
  if(digitalRead(CHANNEL8_IN_PIN))
  {
    ulStart = micros();
  }
  else
  {
    unChannel8InShared = (uint32_t)(micros() - ulStart);
    bUpdateFlagsShared |= CHANNEL8_FLAG;
  }
}
void calcLidar()
{
  static uint32_t ulStart;
    if(digitalRead(LidarPwmPin))
  {
    ulStart = micros();
  }
  else
  {
    pulse_width = (uint32_t)(micros() - ulStart);
    Lidar_flag = 1;
  }
  //Serial.println("ok");
}





