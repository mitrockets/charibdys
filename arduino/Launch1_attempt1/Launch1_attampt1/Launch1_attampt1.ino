#include <Servo.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include Adafruit_LSM303_U.h>
#include <Adafruit_BMP085_U.h>


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
int launch_accel = 3;/////////////////////UPDATE
//In collect data- must have variable called acceleration
float acceleration = 0;///////////////UPDATE OR DELETE WHEN IMU INTEGRATED
int analogPin = 0;////////////////////////////////////////edit
int deploypin = 1;////////////////////////////////////////edit
int count = 0;
int delay_vals[] =   {7000,500,7000,500,7000,7000,6000,7000};
int actuate_vals[] = {100 ,135,100 ,65 ,100 ,135 ,100 ,100 };
void setup() {
  // put your setup code here, to run once:
Serial.begin(9600);
servo_1.attach(10);
servo_2.attach(11);
int analogPin = 0;////////////////////////////////////////edit
int deploypin = 1;////////////////////////////////////////edit
pinMode(deploypin, OUTPUT);
digitalWrite(deploypin, LOW);

}

void loop() 
{
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
    Serial.print("I am stuck on a loop\n");
    collect_data();
  }
  
}

void arm_seq() {
  if (hasArmed){
    Serial.print("WAS ARMED- HAS GONE THROUGH CODE\n");
    delay(1000);
    return;
  }
  while (Serial.available() == false) {
    Serial.print("Ready to be armed. Type 'a'\n");
    delay(1000);
  }
  if (Serial.available()) {
      // Read command
      byte arm = Serial.read ();
  
      if (arm == 'a'){
        Serial.print("ARMED\n");
        hasArmed = true;
      }
      else {
        Serial.print("Type 'a' to arm. Starting over...\n");
        begin_again();
      }
  }
  delay(1000);
}

void servos_straight(){
  servo_1.write(80);
  servo_2.write(100);
  Serial.print("Servos being commanded to 0 degrees\n");
  Serial.print("Check slots on rocket- Make sure fins will deploy\n");
  
  while (Serial.available() == false) {
    Serial.print("Are the fins clear of the slots? Type 'y'\n");
    delay(1000);
  }
  if (Serial.available()) {
      // Read command
      byte straight = Serial.read ();
  
      if (straight == 'y'){
        Serial.print("Flaps confirmed straight. Continuing...\n");
      }
      else {
        Serial.print("Type 'y' if they are good. Starting over...\n");
        begin_again();
      }
  }
  delay(1000);
}

void do_i_data_collect(){
  while (Serial.available() == false) {
    Serial.print("Do I start collecting data? Type 's'\n");
    delay(1000);
  }
  if (Serial.available()) {
      // Read command
      byte collect = Serial.read ();
  
      if (collect == 's'){
        Serial.print("Starting data collection now...\n");
      }
      else {
        Serial.print("Type 's' if you want to collect data. Starting over...\n");
        begin_again();
      }
  }
  delay(1000);
}

void collect_detect(){
  acceleration = 0;
  while (acceleration < launch_accel){
  if (Serial.available()) {
      begin_again();
      break;
  }
  collect_data();
  ///////////////////////////////////////////////////////////////DELETE LINES below
  Serial.print("I am in collect_detect right now\n");
  delay(1000);////////////////////////////////////////////////////////////////
  acceleration+=1;
  }
}

void collect_timer(){
  start = millis();
  now = millis();
  while (now-start < 3000){  /////////////////////////////HOW LONG IS LAUNCH
  collect_data();
  now = millis();
  Serial.println(now-start);////////////////////////////////delete eventually
  }
  delay(1000);////////////////////////////////////////////REMOVE EVENTUALLY
}

void collect_deploy(){
  while (altimeter < 5){/////////////////////////////////////UPDATE
    update_altimeter();////////////////////////Potential problem- might miss reading
    //Call update altimeter within collect data?
    //Time collect_data
    collect_data();
    altimeter+=1;////////////////////////////////////////////////delete
    Serial.print("I am in collect_deploy right now\n");
  }
    //Serial.end();//////////////////////////////////////////////////////////UNCOMMENT
}

void update_altimeter(){
  //check analog signal-- update altimeter variable
  altimeter = analogRead(analogPin);
  Serial.print("Reading altimeter\n");
}
void collect_data(){
  /* Get a new sensor event */ 
  //code to read acceleration data in X, Y, and Z directions
  
  //This code applies to the 10DOF
  sensors_event_t event; 
  accel.getEvent(&event);
 
  /* Display the results (acceleration is measured in m/s^2) */
  Serial.print("Acceleration X: "); Serial.print(event.acceleration.x); Serial.print("  ");
  Serial.print("Acceleration Y: "); Serial.print(event.acceleration.y); Serial.print("  ");
  Serial.print("Acceleration Z: "); Serial.print(event.acceleration.z); Serial.print("  ");Serial.println("m/s^2 ");
  
  
    /* Get a new sensor event */ 
  sensors_event_t event; 
  mag.getEvent(&event);
 
  /* Display the results (magnetic vector values are in micro-Tesla (uT)) */
  Serial.print("Mag X: "); Serial.print(event.magnetic.x); Serial.print("  ");
  Serial.print("Mag Y: "); Serial.print(event.magnetic.y); Serial.print("  ");
  Serial.print("Mag Z: "); Serial.print(event.magnetic.z); Serial.print("  ");Serial.println("uT");
  
  
    /* Get a new sensor event */ 
  sensors_event_t event;
  bmp.getEvent(&event);
 
  /* Display the results (barometric pressure is measure in hPa) */
  if (event.pressure)
  {
    /* Display atmospheric pressure in hPa */
    Serial.print("Pressure: "); Serial.print(event.pressure); Serial.println(" hPa");
  }
  else
  {
    Serial.println("Sensor error");
  }
  delay(500);
  //code that reads data from IMU goes here
  //Max: We need to save this data every time this function runs
}

void deploy(){
  deploy_start = millis();
  now_1 = millis();
  while (now_1-deploy_start < 4000){/////////////////////Edit delay for hot wire
    digitalWrite(deploypin, HIGH);
    collect_data();
    now_1 = millis();
    Serial.print("Making the wire smoking hot\n");
}
  digitalWrite(deploypin, LOW);
}

void collect_actuate(){
  while (count < sizeof(actuate_vals)/sizeof(int)){
    collect_delay();
    Serial.print("count is ");
    Serial.println(count);
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


