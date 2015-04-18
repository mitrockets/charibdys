#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_BMP085_U.h>
#include <SFE_LSM9DS0.h>
#include <SPI.h>
#include <SdFat.h>
#include <Servo.h>

/* Assign a unique ID to this sensor at the same time */
//Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(54321);
//Adafruit_BMP085_Unified bmp = Adafruit_BMP085_Unified(10085);

#define LSM9DS0_XM  0x1D // Would be 0x1E if SDO_XM is LOW
#define LSM9DS0_G   0x6B // Would be 0x6A if SDO_G is LOW
LSM9DS0 dof(MODE_I2C, LSM9DS0_G, LSM9DS0_XM);

// SD chip select pin
const uint8_t chipSelect = SS;
// file system object
SdFat sd;
// create Serial stream
ArduinoOutStream cout(Serial);
char fileName[] = "rocket_team_launch_data_4_11_15.csv";
//------------------------------------------------------------------------------
// store error strings in flash to save RAM
#define error(s) sd.errorHalt(F(s))

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
int delay_vals[] =   {7000,500,7000,500,7000,7000,6000,7000};
int actuate_vals[] = {100 ,135,100 ,65 ,100 ,135 ,100 ,100 };

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial.print("hello");
  servo_1.attach(2);
  servo_2.attach(3);
  pinMode(deploypin, OUTPUT);
  digitalWrite(deploypin, HIGH);
  //bmp.begin();
  uint16_t status = dof.begin(dof.G_SCALE_2000DPS, 
                              dof.A_SCALE_6G, dof.M_SCALE_2GS);
                              
  // initialize the SD card at SPI_HALF_SPEED to avoid bus errors with
  // breadboards.  use SPI_FULL_SPEED for better performance
  if (!sd.begin(chipSelect, SPI_HALF_SPEED)) {
    sd.initErrorHalt();
  }

  writeFile("INFO, Setup part done",true,true);
}

void loop() 
{
  Serial.print("Starting loop\n");
  beginning:
  
  //Checking for bluetooth-- "ok" signal
  start_again = false;
  arm_seq();
  //When "a" input continue
  
  if (start_again == true){
    goto beginning;
  }
  

  //Command servos to 0deg
  servos_straight();
  //When "y" input continue
  
  if (start_again == true){
    goto beginning;
  }
  

  //Ask whether to start data collect
  do_i_data_collect();
  //When "s" input continue and start collecting data
  
    if (start_again == true){
    goto beginning;
  }
  
  //Start collecting data- stay in this loop until launch
  collect_detect();
  //exit when launch
  
    if (start_again == true){
    goto beginning;
  }
  
  //This for safety-- We know that the fins should not deploy for x seconds after launch
  //Continue collecting data- start timer
  collect_timer();
  //exit when timer end
  
  if (start_again == true){
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
  while (1){
    writeFile("I am stuck on a loop\n", true, true);
    collect_data();
  }
  
}

void arm_seq() {
  if (hasArmed){
    writeFile("WAS ARMED- HAS GONE THROUGH CODE\n",true,true);
    Serial.print("say a\n");
    delay(1000);
    return;
  }
  while (Serial.available() == false) {
    //Serial.print("Ready to be armed. Type 'a'\n");
    delay(1000);
  }
  if (Serial.available()) {
      // Read command
      byte arm = Serial.read();
  
      if (arm == 'a'){
        writeFile("ARMED\n", true, true);
        Serial.print("ARMED");
        hasArmed = true;
      }
      else {
        writeFile("Type 'a' to arm. Starting over...\n",true,true);
        begin_again();
      }
  }
  delay(1000);
}

void servos_straight(){
  servo_1.write(80);
  servo_2.write(100);
  writeFile("Servos being commanded to 0 degrees\n",false,true);
  writeFile("Check slots on rocket- Make sure fins will deploy\n",true,true);
  Serial.print("Servos being commanded to 0 degrees\n");
  Serial.print("Check slots on rocket- Make sure fins will deploy\n");
  while (Serial.available() == false) {
    writeFile("Are the fins clear of the slots? Type 'y'\n", true, true);
    delay(1000);
  }
  if (Serial.available()) {
      // Read command
      byte straight = Serial.read ();
  
      if (straight == 'y'){
        writeFile("Flaps confirmed straight. Continuing...\n", true, true);
        Serial.print("Flaps confirmed straight. Continuing...\n");
      }
      else {
        writeFile("Type 'y' if they are good. Starting over...\n", true, true);
        begin_again();
      }
  }
  delay(1000);
}

void do_i_data_collect(){
  while (Serial.available() == false) {
    writeFile("Do I start collecting data? Type 's'\n", true, true);
    Serial.print("Do I start collecting data? Type 's'\n");
    delay(1000);
  }
  if (Serial.available()) {
      // Read command
      byte collect = Serial.read ();
  
      if (collect == 's'){
        writeFile("Starting data collection now...\n", true, true);
        Serial.print("Starting data collection now...\n");
      }
      else {
        writeFile("Type 's' if you want to collect data. Starting over...\n",true,true);
        begin_again();
      }
  }
  delay(1000);
}

void collect_detect(){
  float ax = dof.calcAccel(dof.ax);
  float ay = dof.calcAccel(dof.ay);
  float az = dof.calcAccel(dof.az);
  acceleration = sqrt(ax*ax+ay*ay+az*az);
  while (acceleration < launch_accel){
  if (Serial.available()) {
      begin_again();
      break;
  }
  collect_data();
  writeFile("I am in collect_detect right now\n",true,true);
  Serial.print("I am in collect_detect right now. Waiting for 3gs of acceleration\n");
  }
}

void collect_timer(){
  writeFile("Launch detected!!!",true,true);
  Serial.print("Launch detected!!!\n");
  start = millis();
  now = millis();
  while (now-start < 33000){  /////////////////////////////HOW LONG IS LAUNCH
    if (Serial.available()) {
        begin_again();
        break;
    }
    collect_data();
    now = millis();
  }

}

void collect_deploy(){
  while (altimeter < 5){/////////////////////////////////////UPDATE
    update_altimeter();////////////////////////Potential problem- might miss reading
    //Call update altimeter within collect data?
    //Time collect_data
    collect_data();
    altimeter+=1;////////////////////////////////////////////////delete
    writeFile("I am in collect_deploy right now\n",true,true);
  }
    //Serial.end();//////////////////////////////////////////////////////////UNCOMMENT
}

void update_altimeter(){
  //check analog signal-- update altimeter variable
  altimeter = analogRead(analogPin);
  writeFile("Reading altimeter\n", false, true);
}

void collect_data(){
  unsigned long timestamp = millis();
  writeFile("DATA"+String(timestamp)+",",false,false);
  
  printHeading((float) dof.mx, (float) dof.my);
  printOrientation(dof.calcAccel(dof.ax), dof.calcAccel(dof.ay), 
                   dof.calcAccel(dof.az));
  writeFile("\n",false,false);
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
  
   writeFile(String(heading)+",",false,false);
}

void printOrientation(float x, float y, float z)
{
  float pitch, roll;
  
  pitch = atan2(x, sqrt(y * y) + (z * z));
  roll = atan2(y, sqrt(x * x) + (z * z));
  pitch *= 180.0 / PI;
  roll *= 180.0 / PI;
  
  writeFile(String(pitch)+","+String(roll),false,false);
}

void deploy(){
  deploy_start = millis();
  now_1 = millis();
  while (now_1-deploy_start < 10000){/////////////////////Edit delay for hot wire
    digitalWrite(deploypin, LOW);
    collect_data();
    now_1 = millis();
    writeFile("Making the wire smoking hot\n", true, true);
}
  digitalWrite(deploypin, HIGH);
}

void collect_actuate(){
  while (count < sizeof(actuate_vals)/sizeof(int)){
    collect_delay();
    writeFile("count is "+String(count)+"\n", true, true);
    actuate();
  }
  ramp_up(65,100);
  value_delay = 5000;
  collect_delay2();
  ramp_up(100,135);
  collect_delay2();
  Serial.print("Leaving...\n");
}

void collect_delay(){
  now_2 = millis();
  Begin = millis();
  while (now_2-Begin < delay_vals[count]){
    collect_data();
    now_2 = millis();
  }
}

void actuate(){
  servo_1.write(actuate_vals[count]-20);
  servo_2.write(actuate_vals[count]);
  count+=1;
  collect_data();
}

void end_task(){
  //You can put stuff here later
}

void begin_again(){
  start_again = true;
  hasArmed = false;
}

int ramp_up(int initial,int final){
  for(pos = initial; pos <= final; pos += 1) // goes from 0 degrees to 10 degrees 
  {                                  // in steps of 1 degree 
    servo_1.write(pos-20);              // tell servo to go to position in variable 'pos' 
    servo_2.write(pos);
    delay(300);                      // waits 15ms for the servo to reach the position
    Serial.print("yo\n"); 
    collect_data();
  } 
}

void collect_delay2(){
  now_2 = millis();
  Begin = millis();
  while (now_2-Begin < value_delay){
    collect_data();
    now_2 = millis();
  }
}

// write to file
void writeFile(String stringIn, boolean printToSerial, boolean timeStamp) {
  Serial.print(printToSerial);
  if(printToSerial){
    Serial.print(stringIn);
  }
  if(timeStamp){
    unsigned long timestamp = millis();
    writeFile(String(timestamp)+"\n", false, false);
  }
  
  int stringLength = stringIn.length();
  char data [stringLength];
  stringIn.toCharArray(data, stringLength);
  
  // create or open and truncate output file
  ofstream sdout(fileName, O_APPEND);

  // write file from string stored in flash
  sdout << data << flush;
  sdout << F("\n") << flush;

  // check for any errors
  if (!sdout) {
    error("writeFile");
  }

  sdout.close();
}

