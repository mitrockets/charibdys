#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_BMP085_U.h>
#include <Adafruit_L3GD20_U.h>
#include <Adafruit_10DOF.h>
#include <SFE_LSM9DS0.h>
#include <SPI.h>
#include <SD.h>
#include <Servo.h>

//------------------- IMPORT LIBRARY OBJECTS ----------------------------------
/* Adafruit - one object per sensor assignment
 * -- not in use: GYRO and thermometer */
Adafruit_10DOF                dof10   = Adafruit_10DOF();
Adafruit_LSM303_Accel_Unified accel10 = Adafruit_LSM303_Accel_Unified(30301);
Adafruit_LSM303_Mag_Unified   mag10   = Adafruit_LSM303_Mag_Unified(30302);
Adafruit_BMP085_Unified       bmp     = Adafruit_BMP085_Unified(18001);
/* Update this with the correct SLP for accurate altitude measurements */
float seaLevelPressure = SENSORS_PRESSURE_SEALEVELHPA;

/* Sparkfun one object for all sensors */
#define LSM9DS0_XM  0x1D // Would be 0x1E if SDO_XM is LOW
#define LSM9DS0_G   0x6B // Would be 0x6A if SDO_G is LOW
LSM9DS0                       dof9    = LSM9DS0(MODE_I2C, LSM9DS0_G, LSM9DS0_XM);

/* microSD save data*/
File collected_data;
bool print_data = true;
char fileName[] = "arduino_test_data_5_6_15.csv";
//------------------------------------------------------------------------------

//------------------------- GLOBAL VARIABLES ----------------------------------
// TODO: Minimize this list:
//Put whatever bluetooth import here
unsigned long start;
unsigned long now;
unsigned long now_1;
unsigned long deploy_start;
unsigned long now_2;
unsigned long Begin;
bool start_again = false;
bool hasArmed = false;
Servo servo_1;
Servo servo_2;
int altimeter = 0;
int value_delay;
int pos = 0;
int launch_accel = 4;/////////////////////UPDATE
//In collect data- must have variable called acceleration
float acceleration = 0;///////////////UPDATE OR DELETE WHEN IMU INTEGRATED
int analogPin = 1;////////////////////////////////////////edit
int deploypin = 4;////////////////////////////////////////edit
int count = 0;
int delay_vals[] =   {7000, 500, 7000, 500, 7000, 7000, 6000, 7000};
int actuate_vals[] = {100 , 135, 100 , 65 , 100 , 135 , 100 , 100 };
//------------------------------------------------------------------------------

//----------------------------  SETUP  -----------------------------------------
void setup() {
  Serial.begin(9600);
  initialize_SD_card();
  save_string("INFO, Arduino boot\n");

  /* Activate sensors */
  accel.begin();
  mag.begin();
  bmp.begin();
  uint16_t status = dof9.begin(dof.G_SCALE_2000DPS,
                               dof.A_SCALE_6G, dof.M_SCALE_2GS);
  /* Activate actuators */
  servo_1.attach(2);
  servo_2.attach(3);
  pinMode(deploypin, OUTPUT);
  digitalWrite(deploypin, HIGH);

  save_string("INFO, Setup part done\n");
}
//------------------------------------------------------------------------------

//--------------------------------  LOOP  --------------------------------------
void loop()
{
  Serial.print("INFO, Starting loop\n");
  beginning:
  
  //Checking for bluetooth-- "ok" signal
  start_again = false;
  arm_seq();
  //When "a" input continue

  if (start_again == true) {
    goto beginning;
  }


  //Command servos to 0 degrees
  servos_straight();
  //When "y" input continue

  if (start_again == true) {
    goto beginning;
  }


  //Ask whether to start data collect
  do_i_data_collect();
  //When "s" input continue and start collecting data

  if (start_again == true) {
    goto beginning;
  }

  //Start collecting data- stay in this loop until launch
  collect_detect();
  //exit when launch

  if (start_again == true) {
    goto beginning;
  }

  //This for safety-- We know that the fins should not deploy for x seconds after launch
  //Continue collecting data- start timer
  collect_timer();
  //exit when timer end

  if (start_again == true) {
    goto beginning;
  }

  //Continue collecting data- check for altimeter signal
  collect_deploy();
  //exit when altimeter activated

  //Make the wire hot for delay
  deploy();
  //Exit after delay

  //Collect data and actuate the servos
  collect_actuate();
  //Exit when finished actuating

  //At the end do this
  end_task();
  //You can do things

  //Get it stuck on a loop
  while (1) {
    writeFile("I am stuck on a loop\n", true, true);
    collect_data();
  }

}
//------------------------------------------------------------------------------

//------------------------  CUSTOM FUNCTIONS  ----------------------------------
void arm_seq() {
  if (hasArmed) {
    save_string("INFO, WAS ARMED- HAS GONE THROUGH CODE\n");
    delay(1000);
    return;
  }
  
  // Block the 
  while (Serial.available() == false) {
    Serial.print("INPUT, Type 'a' to arm.....\n");
    delay(1000);
  }

  // Read command
  byte arm = Serial.read();
  if (arm == 'a') {
    hasArmed = true;
    save_string("INFO, Rocket ARMED\n");
  } else {
    save_string("INPUT, didn't arm. Starting over...\n");
    begin_again();
  }
  
  delay(1000);
}

void servos_straight() {
  servo_1.write(80);
  servo_2.write(100);
  save_string("INFO, prompting for fin flaps verification\n");
  Serial.print("Check slots on rocket- Make sure fins will deploy\n");

  while (Serial.available() == false) {
    Serial.print("Are the fins clear of the slots? Type 'y'\n");
    delay(1000);
  }
  
  // Read command
  byte straight = Serial.read ();
  if (straight == 'y') {
    save_string("INFO, flaps confirmed straight. Continuing...\n");
  } else {
    save_string("INFO, flaps not confirmed. Starting over...\n");
    begin_again();
  }
  
  delay(1000);
}

void do_i_data_collect() {
  while (Serial.available() == false) {
    writeFile("INFO, Prompting for data collection \n");
    Serial.print("Do I start collecting data? Type 's'\n");
    delay(1000);
  }
  // Read command
  byte collect = Serial.read ();

  if (collect == 's') {
    save_string("INFO, Starting data collection now...\n", true, true);
    Serial.print("Starting data collection now...\n");
  } else {
    writeFile("Type 's' if you want to collect data. Starting over...\n", true, true);
    begin_again();
  }
 
  delay(1000);
}

void collect_detect() {
  float ax = dof.calcAccel(dof.ax);
  float ay = dof.calcAccel(dof.ay);
  float az = dof.calcAccel(dof.az);
  acceleration = sqrt(ax * ax + ay * ay + az * az);
  while (acceleration < launch_accel) {
    if (Serial.available()) {
      begin_again();
      break;
    }
    collect_data();
    writeFile("I am in collect_detect right now\n", true, true);
    Serial.print("I am in collect_detect right now. Waiting for 3gs of acceleration\n");
  }
}

void collect_timer() {
  writeFile("Launch detected!!!", true, true);
  Serial.print("Launch detected!!!\n");
  start = millis();
  now = millis();
  while (now - start < 33000) { /////////////////////////////HOW LONG IS LAUNCH
    if (Serial.available()) {
      begin_again();
      break;
    }
    collect_data();
    now = millis();
  }

}

void collect_deploy() {
  while (altimeter < 5) { /////////////////////////////////////UPDATE
    update_altimeter();////////////////////////Potential problem- might miss reading
    //Call update altimeter within collect data?
    //Time collect_data
    collect_data();
    altimeter += 1; ////////////////////////////////////////////////delete
    writeFile("I am in collect_deploy right now\n", true, true);
  }
  //Serial.end();//////////////////////////////////////////////////////////UNCOMMENT
}

void update_altimeter() {
  //check analog signal-- update altimeter variable
  altimeter = analogRead(analogPin);
  writeFile("Reading altimeter\n", false, true);
}

void collect_data() {
  unsigned long timestamp = millis();
  writeFile("DATA" + String(timestamp) + ",", false, false);

  printHeading((float) dof.mx, (float) dof.my);
  printOrientation(dof.calcAccel(dof.ax), dof.calcAccel(dof.ay),
                   dof.calcAccel(dof.az));
  writeFile("\n", false, false);
}

void printHeading(float hx, float hy)
{
  float heading;

  if (hy > 0)
  {
    heading = 90 - (atan(hx / hy) * (180 / PI));
  }
  else if (hy < 0)
  {
    heading = - (atan(hx / hy) * (180 / PI));
  }
  else // hy = 0
  {
    if (hx < 0) heading = 180;
    else heading = 0;
  }

  writeFile(String(heading) + ",", false, false);
}

void printOrientation(float x, float y, float z)
{
  float pitch, roll;

  pitch = atan2(x, sqrt(y * y) + (z * z));
  roll = atan2(y, sqrt(x * x) + (z * z));
  pitch *= 180.0 / PI;
  roll *= 180.0 / PI;

  writeFile(String(pitch) + "," + String(roll), false, false);
}

void deploy() {
  deploy_start = millis();
  now_1 = millis();
  while (now_1 - deploy_start < 10000) { /////////////////////Edit delay for hot wire
    digitalWrite(deploypin, LOW);
    collect_data();
    now_1 = millis();
    writeFile("Making the wire smoking hot\n", true, true);
  }
  digitalWrite(deploypin, HIGH);
}

void collect_actuate() {
  while (count < sizeof(actuate_vals) / sizeof(int)) {
    collect_delay();
    writeFile("count is " + String(count) + "\n", true, true);
    actuate();
  }
  ramp_up(65, 100);
  value_delay = 5000;
  collect_delay2();
  ramp_up(100, 135);
  collect_delay2();
  Serial.print("Leaving...\n");
}

void collect_delay() {
  now_2 = millis();
  Begin = millis();
  while (now_2 - Begin < delay_vals[count]) {
    collect_data();
    now_2 = millis();
  }
}

void actuate() {
  servo_1.write(actuate_vals[count] - 20);
  servo_2.write(actuate_vals[count]);
  count += 1;
  collect_data();
}

void end_task() {
  //You can put stuff here later
}

void begin_again() {
  start_again = true;
  hasArmed = false;
}

int ramp_up(int initial, int final) {
  for (pos = initial; pos <= final; pos += 1) // goes from 0 degrees to 10 degrees
  { // in steps of 1 degree
    servo_1.write(pos - 20);            // tell servo to go to position in variable 'pos'
    servo_2.write(pos);
    delay(300);                      // waits 15ms for the servo to reach the position
    Serial.print("yo\n");
    collect_data();
  }
}

void collect_delay2() {
  now_2 = millis();
  Begin = millis();
  while (now_2 - Begin < value_delay) {
    collect_data();
    now_2 = millis();
  }
}



