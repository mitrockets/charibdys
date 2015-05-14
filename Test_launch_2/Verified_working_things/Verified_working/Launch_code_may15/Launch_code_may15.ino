/*
This is the final launch code for Friday, May 13 2015

The main loop runs through once. Each function within it is its own loop.
Meant to be used with bluetooth. 
At all points, entering a value into serial monitor will start the code 
over from scratch (unless it is asking for a specific value at that point)

The flow is as follows:
-Asks permission to arm
-Commands the servos to be straight- asks verification
-Commands servos to max- asks verification
-Commands servos to min- asks verification
-Asks whether you are ready to do final arming (starts listening to accelerometer for
launch detection.
-Starts listening to accelerometer. If acceleration past set limit, then launch detected
-Collects data for a set period of time. The key point of this function is that it only
collects data and presents no danger of deploying fins early in launch
-Starts listening to altimeter pyro signal and collects data at the same time
-Deploys the fins and collects data at the same time
-Starts actuating the fins and collects data
-Does an end task that does not save to SD card
*/

//IMU related things
#include <math.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_BMP085_U.h>
#include <Adafruit_L3GD20_U.h>
#include <Adafruit_10DOF.h>
#include <Adafruit_Simple_AHRS.h>
Adafruit_LSM303_Accel_Unified accel(30301);
Adafruit_LSM303_Mag_Unified   mag(30302);
Adafruit_BMP085_Unified       bmp(18001);
Adafruit_L3GD20_Unified       gyro  = Adafruit_L3GD20_Unified(20);
#define sampleFreq   25.0f        // sample frequency in Hz
#define betaDef      3.0f         // 2 * proportional gain
float psi, theta, phi;
float invSqrt(float x);
volatile float beta = betaDef;    // 2 * proportional gain (Kp)
volatile float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;   // quaternion of sensor frame relative to auxiliary frame
float invSqrt(float x) {
  float halfx = 0.5f * x;
  float y = x;
  long i = *(long*)&y;
  i = 0x5f3759df - (i >> 1);
  y = *(float*)&i;
  y = y * (1.5f - (halfx * y * y));
  return y;
}
float az;

//Servo related things/////////////////////////////////MODIFY
#include <Servo.h>
Servo servo_1;
Servo servo_2;
int servo1_center = 90;
int servo1_max = 110;
int servo1_min = 70;
int servo2_center = 90;
int servo2_max = 110;
int servo2_min = 70;

//PID things
#include <PID_v4.h>
double Input, Output, pid_out; 
double Setpoint = 30;
float P;
float I;
float D;
PID myPID(&Input, &Output, &Setpoint, 1, 0, 0,DIRECT);//pid///////////////
int pid_time = 60000;
double servo1Output, servo2Output;

//Timing related things
unsigned long now;
unsigned long start;

//SD card related things
#include <SD.h>
#include <SPI.h>
double dataFile_values[10];
const int chipSelect = 53;
String printablestring;
File dataFile;

//Preset actuation things
int count = 0;
int delay_vals[] =   {5000,5000,5000,5000,5000,5000};///////////////////MODIFY
int actuate_vals[] = {100 ,80  ,110 ,70  ,120 ,60  };
//Make a check to prevent overshooting limits

//Used for restarting the code
bool start_again = false;

//Function code is currently in
String function;

//Launch acceleration detection limit
int launch_accel = 500;////////////////////////////////////////////////MODIFY

//Pyro check things
int altimeter_check_pin = 3;/////////////////////////////////////////MODIFY    
int altimeter_value = 0;
int deploypin = 5;///////////////////////////////////////////////////MODIFY
int pyro_on = 20;////////////////////////////////////////////////////MODIFY

//Deploy fins things
int time_wire_hot = 10000;//////////////////////////////////////////MODIFY

void setup() {
  Serial.begin(9600);  
  Serial.println("Setting up...");
  initialize_SD_card();
  dataFile = SD.open("data.txt", FILE_WRITE);//////////////////////MODIFY
  now = millis();
  dataFile.println("Setting up");
  dataFile.print("Start Time : ");
  dataFile.println(now);
  
  //Attatch servos;
  servo_1.attach(8);
  servo_2.attach(9);
  
  //Deploy setup
  pinMode(deploypin,OUTPUT);
  digitalWrite(deploypin,LOW);
  
  //Set up IMU
  accel.begin();
  mag.begin();
  bmp.begin();
  gyro.begin();
  
  //PID setup
  myPID.SetMode(AUTOMATIC); //turn PID on
  myPID.SetOutputLimits(-400,400);
  
  Serial.println("Done with Setup");
  dataFile.println("Done with Setup");
}

void loop() {
  beginning:
  Serial.println("Starting main loop now...");
  dataFile.println("Starting main loop now...");
  
  start_again = false;
  
  //Asks for permission to arm and sends the state of the system
  //Look at the state and make sure that values comply with checklist
  arm_seq();
  
            if (start_again == true){goto beginning;}
  
  //Sets the servos straight and asks verification
  servos_straight();
  
            if (start_again == true){goto beginning;}
  
  servos_max();
  
            if (start_again == true){goto beginning;}
  
  servos_min();
  
            if (start_again == true){goto beginning;}

  //Asks user if ready to launch. ONLY DO THIS IF ROCKET IS SETUP! 
  final_arming();
  
            if (start_again == true){goto beginning;}
  
  //Starts detecting launch by listening to accelerometer
  launch_detect();
  
            if (start_again == true){goto beginning;}
  
  //Delays any activity other than collecting data to prevent premature fin deployment          
  safe_delay();
            
            if (start_again == true){goto beginning;}
  
  //Listens for pyro signal from altimeter   
  pyro_detect();
  
            if (start_again == true){goto beginning;}
            
  //Deploys the fins by making the wire hot          
  deploy_fins();
  
            if (start_again == true){goto beginning;}
            
  actuate_preset();
  
  actuate_control();
  end_task();
  //dataFile.close();
  
}

void end_task(){
  function = "end_task";
  get_state();
  print_state();
  while (1){delay(1000);}
}

void actuate_PID(){
  servo1Output = map(Output, -400, 400, servo1_min, servo1_max);////////////////////Set servo constraint here
  servo2Output = map(Output, -400, 400, servo2_min, servo2_max);
  
  if (servo1Output > servo1_max){
    servo1Output = servo1_max;
  }
  if (servo1Output < servo1_min){
    servo1Output = servo1_min;
  }
  
  if (servo2Output > servo2_max){
    servo2Output = servo2_max;
  }
  if (servo2Output < servo2_min){
    servo2Output = servo2_min;
  }
}


void actuate_control(){
  function = "actuate_control";
  get_state();
  print_state();
  start = millis();
  now = millis();
  while (now-start < pid_time){
    function = "actuate_control_whileLoop";
    get_state();
    print_state();
    pid();
    actuate_PID;
  }
}

void pid(){
  Input = psi;
  myPID.Compute();
  pid_out = Output;
}

void actuate_preset(){
  function = "actuate_preset";
  get_state();
  print_state();
  while (count < sizeof(actuate_vals)/sizeof(int)){ 
    actuate_preset_vals();
    actuate_delay();
  }
}

void actuate_delay(){
  function = "actuate_delay";
  start = millis();
  now = millis();
  while (now-start < delay_vals[count]){
    get_state();
    print_state();
    function = "actuate_delay_whileLoop";
  }
}

void actuate_preset_vals(){
    function = "actuate_preset";
    servo_1.write(actuate_vals[count]);///////////////////////////////MODIFY
    servo_2.write(actuate_vals[count]);
    servo1Output = actuate_vals[count];
    servo2Output = actuate_vals[count];
    get_state();
    print_state();
    count += 1;
}

void deploy_fins(){
  function = "deploy_fins";
  start = millis();
  now = millis();
  while (now-start < time_wire_hot){   
    if (Serial.available()) {
        begin_again();
        break;
    }
    digitalWrite(deploypin, HIGH);
    get_state();
    print_state();
    function = "deploy_fins_whileLoop";
  }
  digitalWrite(deploypin, LOW);
}

void pyro_detect(){
  function = "pyro_detect";
  while (altimeter_value < pyro_on){
    if (Serial.available()) {
        begin_again();
        break;
    }
    get_state();
    print_state();
    function = "pyro_detect_whileLoop";
    altimeter_value = analogRead(altimeter_check_pin);
    Serial.print(altimeter_value);
  }
}

void safe_delay(){
  function = "safe_delay";
  start = millis();
  now = millis();
  while (now-start < 20000){
    function = "safe_delay_while_loop";
    if (Serial.available()) {
        begin_again();
        break;
    }
    get_state();
    print_state();
  }
}

void arm_seq(){
  while (Serial.available() == false) {
    get_state();
    print_state();
    Serial.println("Ready to be armed. Type 'a' to continue");
    delay(1000);
  }
  if (Serial.available()) {
      // Read command
      byte arm = Serial.read();
      now = millis();
      if (arm == 'a'){
        Serial.println("System stage 1 ARMED");
        function = "Stage 1 armed";
      }
      else {
        Serial.println("Incorrect key. Press 'a' to arm. Starting over...");
        begin_again();
      }
  }
  delay(1000);
}

void print_state(){
  //Current function, roll, pitch, yaw, altitude, acceleration, Servo command, Time
  printablestring = "";
  printablestring.concat("Function,");
  printablestring.concat("Psi, ");
  printablestring.concat("Theta, ");
  printablestring.concat("Phi, ");
  printablestring.concat("Servo 1 Command, ");
  printablestring.concat("Servo 2 Command, ");
  printablestring.concat("PID Output, ");
  printablestring.concat("Acceleration_z, ");
  printablestring.concat("Count, ");
  printablestring.concat("Time, ");
  printablestring.concat(function);
  printablestring.concat(", ");
  printablestring.concat(psi);
  printablestring.concat(", ");
  printablestring.concat(theta);
  printablestring.concat(", ");
  printablestring.concat(phi);
  printablestring.concat(", ");
  printablestring.concat(servo1Output);
  printablestring.concat(", ");
  printablestring.concat(servo2Output);
  printablestring.concat(", ");
  printablestring.concat(Output);
  printablestring.concat(", ");
  printablestring.concat(az);
  printablestring.concat(", ");
  printablestring.concat(count);
  printablestring.concat(", ");
  printablestring.concat(now);
 
  dataFile.println(printablestring);
}

void get_state(){
  //other states recorded in other functions
  now = millis();
  collect_data();
}

void begin_again(){
  start_again = true;
}

void servos_straight(){
  while(Serial.available() == false){
    function = "servos_straight";
    get_state();
    print_state();
    servo_1.write(servo1_center);
    servo_2.write(servo2_center);
    servo1Output = servo1_center;
    servo2Output = servo2_center;
    Serial.println("Commanding Servos to be straight. Are they? Type 'y'"); 
    delay(1000);
  }
  if (Serial.available()) {
      byte straight = Serial.read();
      if (straight == 'y'){
        function = "Flaps are straight";
        get_state();
        print_state();
        Serial.println("Flaps confirmed straight. Continuing...");
      }
      else {
        Serial.println("Type 'y' if they are good. Starting over...");
        begin_again();
      }
  }
  delay(1000);
}

void servos_min(){
  while(Serial.available() == false){
    function = "servos_min";
    get_state();
    print_state();
    servo_1.write(servo1_min);
    servo_2.write(servo2_min);
    servo1Output = servo1_min;
    servo2Output = servo2_min;
    Serial.println("Commanding Servos to min. Listen for weird noise. Type 'm'"); 
    delay(1000);
  }
  if (Serial.available()) {
      byte straight = Serial.read();
      if (straight == 'm'){
        function = "Flaps are at min and are OK";
        get_state();
        print_state();
        Serial.println("Servo min confirmed OK. Continuing...");
      }
      else {
        Serial.println("Type 'm' if they are good. Starting over...");
        begin_again();
      }
  }
  delay(1000);
}

void final_arming(){
  while(Serial.available() == false){
    function = "Final Arming";
    get_state();
    print_state();
    Serial.println("Ready to do FINAL Arming. Press A to continue."); 
    delay(1000);
  }
  if (Serial.available()) {
      byte straight = Serial.read();
      if (straight == 'A'){
        function = "INITIATE LAUNCH SEQUENCE";
        get_state();
        print_state();
        Serial.println("Launch sequence INITIATED");
      }
      else {
        Serial.println("Type 'A' to initiate launch sequence. Starting over...");
        begin_again();
      }
  }
  delay(1000);
}

void servos_max(){
  while(Serial.available() == false){
    function = "servos_max";
    get_state();
    print_state();
    servo_1.write(servo1_max);
    servo_2.write(servo2_max);
    servo1Output = servo1_max;
    servo2Output = servo2_max;
    Serial.println("Commanding Servos to max. Listen for weird noise. Type 'm'"); 
    delay(1000);
  }
  if (Serial.available()) {
      byte straight = Serial.read();
      if (straight == 'm'){
        function = "Flaps are at max and are OK";
        get_state();
        print_state();
        Serial.println("Servo max confirmed OK. Continuing...");
      }
      else {
        Serial.println("Type 'm' if they are good. Starting over...");
        begin_again();
      }
  }
  delay(1000);
}

void collect_data(){
  MadgwickAHRSupdate();
  
  psi = atan2((2 * q1 * q2) - (2 * q0 * q3), (2 * pow(q0, 2)) + (2 * pow(q1, 2)) - 1);
  psi *= 180.0/PI;
  if (psi < 0) psi+=360;
  
  theta = asin((2 * q1 * q3) + (2 * q0 * q2));
  theta *= 180.0/PI;
  if (theta < 0) theta+=360;
  
  phi = atan2((2 * q2 * q3) - (2 * q0 * q1), (2 * pow(q0, 2)) + (2 * pow(q3, 2)) - 1);
  phi *= 180.0/PI;
  if (phi < 0) phi+=360;
  
  delay(5);
}

void launch_detect(){
  function = "launch_detect";
  get_state();
  print_state();
  while (az < launch_accel){
    function = "launch_detect_whileLoop";
    if (Serial.available()) {
      begin_again();
      break;
      }
     get_state();
     print_state();  
  }
}

void initialize_SD_card() {
   Serial.print("Initializing SD card...");
  // On the Ethernet Shield, CS is pin 4. It's set as an output by default.
  // Note that even if it's not used as the CS pin, the hardware SS pin
  // (10 on most Arduino boards, 53 on the Mega) must be left as an output
  // or the SD library functions will not work.
  pinMode(53, OUTPUT);

  if (!SD.begin(chipSelect)) {
    Serial.println("initialization failed, pin not connected");
    return;
  }
  Serial.println("SD card initialized.");
 }
 
 void MadgwickAHRSupdate() {
  /* Get a new sensor event */
  sensors_event_t eventg;
  gyro.getEvent(&eventg);
  float  gx = eventg.gyro.x;
  float  gy = eventg.gyro.y;
  float  gz = eventg.gyro.z;

  /* Get a new sensor event */
  sensors_event_t eventa;
  accel.getEvent(&eventa);
  float  ax = (eventa.acceleration.x)*101;
  float  ay = eventa.acceleration.y*101;
  az = eventa.acceleration.z*101;
  /* Get a new sensor event */
  sensors_event_t eventm;
  mag.getEvent(&eventm);
  float  mx = eventm.magnetic.x;
  float  my = eventm.magnetic.y;
  float  mz = eventm.magnetic.z;
  float recipNorm;
  float s0, s1, s2, s3;
  float qDot1, qDot2, qDot3, qDot4;
  float hx, hy;
  float _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, _4bz, _2q0, _2q1, _2q2, _2q3, _2q0q2, _2q2q3, q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;

  // Use IMU algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation)
  if ((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f)) {
    MadgwickAHRSupdateIMU();
    return;
  }

  // Rate of change of quaternion from gyroscope
  qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
  qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
  qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
  qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

  // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
  if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

    // Normalise accelerometer measurement
    recipNorm = invSqrt(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    // Normalise magnetometer measurement
    recipNorm = invSqrt(mx * mx + my * my + mz * mz);
    mx *= recipNorm;
    my *= recipNorm;
    mz *= recipNorm;

    // Auxiliary variables to avoid repeated arithmetic
    _2q0mx = 2.0f * q0 * mx;
    _2q0my = 2.0f * q0 * my;
    _2q0mz = 2.0f * q0 * mz;
    _2q1mx = 2.0f * q1 * mx;
    _2q0 = 2.0f * q0;
    _2q1 = 2.0f * q1;
    _2q2 = 2.0f * q2;
    _2q3 = 2.0f * q3;
    _2q0q2 = 2.0f * q0 * q2;
    _2q2q3 = 2.0f * q2 * q3;
    q0q0 = q0 * q0;
    q0q1 = q0 * q1;
    q0q2 = q0 * q2;
    q0q3 = q0 * q3;
    q1q1 = q1 * q1;
    q1q2 = q1 * q2;
    q1q3 = q1 * q3;
    q2q2 = q2 * q2;
    q2q3 = q2 * q3;
    q3q3 = q3 * q3;

    // Reference direction of Earth's magnetic field
    hx = mx * q0q0 - _2q0my * q3 + _2q0mz * q2 + mx * q1q1 + _2q1 * my * q2 + _2q1 * mz * q3 - mx * q2q2 - mx * q3q3;
    hy = _2q0mx * q3 + my * q0q0 - _2q0mz * q1 + _2q1mx * q2 - my * q1q1 + my * q2q2 + _2q2 * mz * q3 - my * q3q3;
    _2bx = sqrt(hx * hx + hy * hy);
    _2bz = -_2q0mx * q2 + _2q0my * q1 + mz * q0q0 + _2q1mx * q3 - mz * q1q1 + _2q2 * my * q3 - mz * q2q2 + mz * q3q3;
    _4bx = 2.0f * _2bx;
    _4bz = 2.0f * _2bz;

    // Gradient decent algorithm corrective step
    s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - ax) + _2q1 * (2.0f * q0q1 + _2q2q3 - ay) - _2bz * q2 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q3 + _2bz * q1) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
    s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - ax) + _2q0 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q1 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + _2bz * q3 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q2 + _2bz * q0) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q3 - _4bz * q1) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
    s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - ax) + _2q3 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q2 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + (-_4bx * q2 - _2bz * q0) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q1 + _2bz * q3) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q0 - _4bz * q2) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
    s3 = _2q1 * (2.0f * q1q3 - _2q0q2 - ax) + _2q2 * (2.0f * q0q1 + _2q2q3 - ay) + (-_4bx * q3 + _2bz * q1) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q0 + _2bz * q2) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
    recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
    s0 *= recipNorm;
    s1 *= recipNorm;
    s2 *= recipNorm;
    s3 *= recipNorm;

    // Apply feedback step
    qDot1 -= beta * s0;
    qDot2 -= beta * s1;
    qDot3 -= beta * s2;
    qDot4 -= beta * s3;
  }

  // Integrate rate of change of quaternion to yield quaternion
  q0 += qDot1 * (1.0f / sampleFreq);
  q1 += qDot2 * (1.0f / sampleFreq);
  q2 += qDot3 * (1.0f / sampleFreq);
  q3 += qDot4 * (1.0f / sampleFreq);

  // Normalise quaternion
  recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  q0 *= recipNorm;
  q1 *= recipNorm;
  q2 *= recipNorm;
  q3 *= recipNorm;
}

void MadgwickAHRSupdateIMU() {
  /* Get a new sensor event */
  sensors_event_t eventg;
  gyro.getEvent(&eventg);
  float  gx = eventg.gyro.x;
  float  gy = eventg.gyro.y;
  float  gz = eventg.gyro.z;

  /* Get a new sensor event */
  sensors_event_t eventa;
  accel.getEvent(&eventa);
  float  ax = eventa.acceleration.x;
  float  ay = eventa.acceleration.y;
  az = eventa.acceleration.z;
  /* Get a new sensor event */
  sensors_event_t eventm;
  mag.getEvent(&eventm);
  float  mx = eventm.magnetic.x;
  float  my = eventm.magnetic.y;
  float  mz = eventm.magnetic.z;
  float recipNorm;
  float s0, s1, s2, s3;
  float qDot1, qDot2, qDot3, qDot4;
  float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 , _8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

  // Rate of change of quaternion from gyroscope
  qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
  qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
  qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
  qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

  // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
  if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

    // Normalise accelerometer measurement
    recipNorm = invSqrt(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    // Auxiliary variables to avoid repeated arithmetic
    _2q0 = 2.0f * q0;
    _2q1 = 2.0f * q1;
    _2q2 = 2.0f * q2;
    _2q3 = 2.0f * q3;
    _4q0 = 4.0f * q0;
    _4q1 = 4.0f * q1;
    _4q2 = 4.0f * q2;
    _8q1 = 8.0f * q1;
    _8q2 = 8.0f * q2;
    q0q0 = q0 * q0;
    q1q1 = q1 * q1;
    q2q2 = q2 * q2;
    q3q3 = q3 * q3;

    // Gradient decent algorithm corrective step
    s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
    s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
    s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
    s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;
    recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
    s0 *= recipNorm;
    s1 *= recipNorm;
    s2 *= recipNorm;
    s3 *= recipNorm;

    // Apply feedback step
    qDot1 -= beta * s0;
    qDot2 -= beta * s1;
    qDot3 -= beta * s2;
    qDot4 -= beta * s3;
  }

  // Integrate rate of change of quaternion to yield quaternion
  q0 += qDot1 * (1.0f / sampleFreq);
  q1 += qDot2 * (1.0f / sampleFreq);
  q2 += qDot3 * (1.0f / sampleFreq);
  q3 += qDot4 * (1.0f / sampleFreq);

  // Normalise quaternion
  recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  q0 *= recipNorm;
  q1 *= recipNorm;
  q2 *= recipNorm;
  q3 *= recipNorm;
}
