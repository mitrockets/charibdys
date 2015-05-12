/*****************************************************************
 * LSM9DS0_SerialMenus.ino
 * SFE_LSM9DS0 Library Example Code: Interact With Serial Menus
 * Jim Lindblom @ SparkFun Electronics
 * Original Creation Date: February 14, 2014 (Happy Valentines Day!)
 * https://github.com/sparkfun/LSM9DS0_Breakout
 * 
 * Distributed as-is; no warranty is given.
 *****************************************************************/

#include <SPI.h> // Included for SFE_LSM9DS0 library
#include <Wire.h>
#include <PID_v1.h>
#include <SFE_LSM9DS0.h>
#include <Servo.h>

///////////////////////
// SDO_XM and SDO_G are both grounded, therefore the addresses are:
#define LSM9DS0_XM  0x1D // Would be 0x1E if SDO_XM is LOW
#define LSM9DS0_G   0x6B // Would be 0x6A if SDO_G is LOW
LSM9DS0 dof(MODE_I2C, LSM9DS0_G, LSM9DS0_XM);

const byte INT1XM = 9; // INT1XM tells us when accel data is ready
const byte INT2XM = 8; // INT2XM tells us when mag data is ready
const byte DRDYG = 4;  // DRDYG tells us when gyro data is ready
boolean printRaw = true;

double Setpoint, Input, Output;
//double Kp=0.029, Ki=0.00, Kd=0.0212;
double Kp=0.01, Ki=0.00, Kd=0.10;

PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

Servo myservo;

boolean useAccel = true;
boolean testMode = false;

float imuReading;
double pidMin, pidMax;

void setup()
{
  // Set up interrupt pins as inputs:
  pinMode(INT1XM, INPUT);
  pinMode(INT2XM, INPUT);
  pinMode(DRDYG, INPUT);

  Serial.begin(115200); // Start serial at 115200 bps
  uint16_t status = dof.begin();

  Serial.println(status, HEX);

  //initialize the variables we're linked to
  Input = dof.calcAccel(dof.ay);
  Setpoint = -7.0; 

  //turn the PID on
  myPID.SetMode(AUTOMATIC);

  myservo.attach(5);
  pidMin = -5.0;
  pidMax = 5.0;
//  pidRange();
  Serial.println(pidMin);
  Serial.println(pidMax);
}

void pidRange()
{
  for (double i = -1.0; i <= 1.0; i = i+0.1)
  {
    Input = i;
    myPID.SetTunings(Kp, Ki, Kd);
    myPID.Compute();
    if (Output > pidMax)
    {
      pidMax = Output;
    }
    else if (Output < pidMin)
    {
      pidMin = Output;
    }
  };
}
void loop()
{
  if (testMode)
  {
    // Print the control menu:
    printMenu();
    // Then wait for any serial data to come in:
    while (!Serial.available());
    // Once serial data is received, call parseMenu to act on it:
    parseMenu(Serial.read());
    Serial.print(digitalRead(INT1XM));
  }
  else //Not testing
  {
    if (useAccel) //Use accelerometer 
    {
//      if (digitalRead(INT1XM))
      {
        dof.readAccel();
        dof.setAccelScale(dof.A_SCALE_2G);
        dof.setAccelODR(dof.A_ODR_3125);
        imuReading = dof.ay/1000.0;
        Input = (-1*abs(imuReading));
        myPID.SetTunings(Kp, Ki, Kd);
        myPID.Compute();
        Serial.print("Input (Acceleration in Y): ");
        Serial.print(dof.ay/1000.0);
//        Serial.print("; Output Before: ");
        Output = abs(Output);
//        Serial.print(Output);
        Serial.print(" Output (from PID): ");
//        Output = Output - 0.22;
        if (imuReading > 0)
        {
          Output = (-1*Output);
        }
        Serial.print(Output);
        if (Output > pidMax) pidMax = Output;
        if (Output < pidMin) pidMin = Output;
        double servoOutput = ((Output - (pidMin)) * (179) / (pidMax - (pidMin)));
        servoOutput = constrain(servoOutput, 29, 149);
//        servoOutput = map(servoOutput, 0, 179, 179, 0);
        int servoWrite = servoOutput;
        if (abs(servoWrite - 90)<25)
        {
          servoWrite = 90;
        }
        myservo.write(servoWrite);
        Serial.print("; Servo Output: ");
        Serial.println(servoWrite);
      }
    }
//    else //Use gyroscope
//    {
//      if (digitalRead(DRDYG))
//      {
//        dof.readGyro();
//        dof.setGyroScale(dof.G_SCALE_245DPS);
//        dof.setGyroODR(dof.G_ODR_95_BW_125);
//        Input = (dof.calcGyro(dof.gy));
//        myPID.SetTunings(Kp, Ki, Kd);
//        myPID.Compute();
//        Serial.print(Input);
//        Serial.print("; Output: ");
//        Serial.print(Output);
//        double servoOutput ((Input + 1) * (179) / (1 - (-1)));
//        servoOutput = constrain(servoOutput, 0, 179);
//        myservo.write(servoOutput);
//        Serial.print("; Servo: ");
//        Serial.println(servoOutput);
//      }
//    }
  }
}

void printAccel()
{
  // Only read from the accelerometer if the accel interrupts,
  // which means that new data is ready.
  if (digitalRead(INT1XM))
  {
    // Use the readAccel() function to get new data from the accel.
    // After calling this function, new values will be stored in
    // the ax, ay, and az variables.
    dof.readAccel();

    Serial.print("A: ");
    if (printRaw)
    {
      Serial.print(dof.ax);
      Serial.print(", ");
      Serial.print(dof.ay);
      Serial.print(", ");
      Serial.println(dof.az);
    }
    else
    {
      // Using the calcAccel helper function, we can get the
      // accelerometer readings in g's.
      Serial.print(dof.calcAccel(dof.ax));
      Serial.print(", ");
      Serial.print(dof.calcAccel(dof.ay));
      Serial.print(", ");
      Serial.println(dof.calcAccel(dof.az));
    }
  }
}

void printGyro()
{
  // Only read from the gyro if the DRDY interrupts,
  // which means that new data is ready.
  if (digitalRead(DRDYG))
  {
    // Use the readGyro() function to get new data from the gyro.
    // After calling this function, new values will be stored in
    // the gx, gy, and gz variables.
    dof.readGyro();

    Serial.print("G: ");
    if (printRaw)
    {
      Serial.print(dof.gx);
      Serial.print(", ");
      Serial.print(dof.gy);
      Serial.print(", ");
      Serial.println(dof.gz);
    }
    else
    {
      // Using the calcGyro helper function, we can get the
      // gyroscope readings in degrees per second (DPS).
      Serial.print(dof.calcGyro(dof.gx));
      Serial.print(", ");
      Serial.print(dof.calcGyro(dof.gy));
      Serial.print(", ");
      Serial.println(dof.calcGyro(dof.gz));
    }
  }
}

void printMag()
{
  // Only read from the magnetometer if the INT2XM interrupts,
  // which means that new data is ready.
  if (digitalRead(INT2XM))
  {
    // Use the readMag() function to get new data from the mag.
    // After calling this function, new values will be stored in
    // the mx, my, and mz variables.
    dof.readMag();

    Serial.print("M: ");
    if (printRaw)
    {
      Serial.print(dof.mx);
      Serial.print(", ");
      Serial.print(dof.my);
      Serial.print(", ");
      Serial.print(dof.mz);
      Serial.print(", ");
      Serial.println(calcHeading(dof.mx, dof.my));
    }
    else
    {
      // Using the calcMg helper function, we can get the
      // magnetometer readings in gauss (Gs).
      Serial.print(dof.calcMag(dof.mx), 4);
      Serial.print(", ");
      Serial.print(dof.calcMag(dof.my), 4);
      Serial.print(", ");
      Serial.print(dof.calcMag(dof.mz), 4);
      Serial.print(", ");
      Serial.println(calcHeading(dof.mx, dof.my));
    }
  }
}

// Here's a simple example function to calculate heading based on
// magnetometer readings. This only works when the 9DOF is flat
// (x-axis normal to gravity).
float calcHeading(float hx, float hy)
{  
  if (hy > 0)
  {
    return 90 - (atan(hx / hy) * 180 / PI);
  }
  else if (hy < 0)
  {
    return 270 - (atan(hx / hy) * 180 / PI);
  }
  else // hy = 0
  {
    if (hx < 0) return 180;
    else return 0;
  }
}

// This function will print all data from all sensors at once.
// It'll wait until every sensor interrupt triggers before
// printing.
void streamAll()
{
  if ((digitalRead(INT2XM)) && (digitalRead(INT1XM)) &&
    (digitalRead(DRDYG)))
  {
    printAccel();
    printGyro();
    printMag();
  }
}

// setScale() provides an interface to switch the full-scale range
// of each sensor. This function will block until three characters
// (to select the three ranges) are received.
void setScale()
{
  char c;

  // Print the accelerometer range options:
  Serial.println(F("Set accelerometer scale:"));
  Serial.println(F("\t1) +/- 2G"));
  Serial.println(F("\t2) +/- 4G"));
  Serial.println(F("\t3) +/- 6G"));
  Serial.println(F("\t4) +/- 8G"));
  Serial.println(F("\t5) +/- 16G"));
  // Wait for a serial char to come in:
  while (Serial.available() < 1)
    ;
  c = Serial.read();
  // Use the setAccelScale function to set the accelerometer
  // full-scale range to any of the possible ranges. These ranges
  // are all defined in SFE_LSM9DS0.h.
  switch (c)
  {
  case '1':
    dof.setAccelScale(dof.A_SCALE_2G);
    break;
  case '2':
    dof.setAccelScale(dof.A_SCALE_4G);
    break;
  case '3':
    dof.setAccelScale(dof.A_SCALE_6G);
    break;
  case '4':
    dof.setAccelScale(dof.A_SCALE_8G);
    break;
  case '5':
    dof.setAccelScale(dof.A_SCALE_16G);
    break;
  }
  // Print the gyro scale ranges:
  Serial.println(F("Set gyroscope scale:"));
  Serial.println(F("\t1) +/- 245 DPS"));
  Serial.println(F("\t2) +/- 500 DPS"));
  Serial.println(F("\t3) +/- 2000 DPS"));
  // Wait for a character to come in:
  while (Serial.available() < 1)
    ;
  c = Serial.read();
  // Use the setGyroScale function to set the gyroscope
  // full-scale range to any of the possible ranges. These ranges
  // are all defined in SFE_LSM9DS0.h.
  switch (c)
  {
  case '1':
    dof.setGyroScale(dof.G_SCALE_245DPS);
    break;
  case '2':
    dof.setGyroScale(dof.G_SCALE_500DPS);
    break;
  case '3':
    dof.setGyroScale(dof.G_SCALE_2000DPS);
    break;
  }
  // Print the magnetometer scale options:
  Serial.println(F("Set magnetometer scale:"));
  Serial.println(F("\t1) +/- 2GS"));
  Serial.println(F("\t2) +/- 4GS"));
  Serial.println(F("\t3) +/- 8GS"));
  Serial.println(F("\t4) +/- 12GS"));
  // Wait for a char:
  while (Serial.available() < 1)
    ;
  c = Serial.read();
  // Use the setMagScale function to set the magnetometer
  // full-scale range to any of the possible ranges. These ranges
  // are all defined in SFE_LSM9DS0.h.
  switch (c)
  {
  case '1':
    dof.setMagScale(dof.M_SCALE_2GS);
    break;
  case '2':
    dof.setMagScale(dof.M_SCALE_4GS);
    break;
  case '3':
    dof.setMagScale(dof.M_SCALE_8GS);
    break;
  case '4':
    dof.setMagScale(dof.M_SCALE_12GS);
    break;
  }
}

// setRaw simply switches the state of the global printRaw
// variable. It'll print a message to say what it's switched to.
void setRaw()
{
  if (printRaw)
  {
    printRaw = false;
    Serial.println(F("Printing calculated readings"));
  }
  else
  {
    printRaw = true;
    Serial.println(F("Printing raw readings"));
  }
}

// setODR() provides a serial interface to set the output data
// rate (ODR) for each sensor. It will block until it receives
// three characters to set the data rates.
void setODR()
{
  char c;

  // Print the menu options for accel data rate:
  Serial.println(F("Set Accelerometer ODR (Hz):"));
  Serial.println(F("\t1) 3.125 \t 6) 100"));
  Serial.println(F("\t2) 6.25  \t 7) 200"));
  Serial.println(F("\t3) 12.5  \t 8) 400"));
  Serial.println(F("\t4) 25    \t 9) 800"));
  Serial.println(F("\t5) 50    \t A) 1600"));
  // Wait for a character to be read in:
  while (Serial.available() < 1)
    ;
  c = Serial.read();
  // Use the setAccelODR function to set the accelerometer
  // data rate to any of the possible ranges. These ranges
  // are all defined in SFE_LSM9DS0.h.
  switch (c)
  {
  case '1':
    dof.setAccelODR(dof.A_ODR_3125);
    break;
  case '2':
    dof.setAccelODR(dof.A_ODR_625);
    break;
  case '3':
    dof.setAccelODR(dof.A_ODR_125);
    break;
  case '4':
    dof.setAccelODR(dof.A_ODR_25);
    break;
  case '5':
    dof.setAccelODR(dof.A_ODR_50);
    break;
  case '6':
    dof.setAccelODR(dof.A_ODR_100);
    break;
  case '7':
    dof.setAccelODR(dof.A_ODR_200);
    break;
  case '8':
    dof.setAccelODR(dof.A_ODR_400);
    break;
  case '9':
    dof.setAccelODR(dof.A_ODR_800);
    break;
  case 'A':
  case 'a':
    dof.setAccelODR(dof.A_ODR_1600);
    break;
  }

  // Print the menu options for the gyro ODR's
  Serial.println(F("Set Gyro ODR/Cutoff (Hz):"));
  Serial.println(F("\t1) 95/12.5 \t 8) 380/25"));
  Serial.println(F("\t2) 95/25   \t 9) 380/50"));
  Serial.println(F("\t3) 190/125 \t A) 380/100"));
  Serial.println(F("\t4) 190/25  \t B) 760/30"));
  Serial.println(F("\t5) 190/50  \t C) 760/35"));
  Serial.println(F("\t6) 190/70  \t D) 760/50"));
  Serial.println(F("\t7) 380/20  \t E) 760/100"));
  // Wait for a character to arrive:
  while (Serial.available() < 1)
    ;
  c = Serial.read();
  // Use the setGyroODR function to set the gyroscope
  // data rate to any of the possible ranges. These ranges
  // are all defined in SFE_LSM9DS0.h.
  switch (c)
  {
  case '1':
    dof.setGyroODR(dof.G_ODR_95_BW_125);
    break;
  case '2':
    dof.setGyroODR(dof.G_ODR_95_BW_25);
    break;
  case '3':
    dof.setGyroODR(dof.G_ODR_190_BW_125);
    break;
  case '4':
    dof.setGyroODR(dof.G_ODR_190_BW_25);
    break;
  case '5':
    dof.setGyroODR(dof.G_ODR_190_BW_50);
    break;
  case '6':
    dof.setGyroODR(dof.G_ODR_190_BW_70);
    break;
  case '7':
    dof.setGyroODR(dof.G_ODR_380_BW_20);
    break;
  case '8':
    dof.setGyroODR(dof.G_ODR_380_BW_25);
    break;
  case '9':
    dof.setGyroODR(dof.G_ODR_380_BW_50);
    break;
  case 'A':
  case 'a':
    dof.setGyroODR(dof.G_ODR_380_BW_100);
    break;
  case 'B':
  case 'b':
    dof.setGyroODR(dof.G_ODR_760_BW_30);
    break;
  case 'C':
  case 'c':
    dof.setGyroODR(dof.G_ODR_760_BW_35);
    break;
  case 'D':
  case 'd':
    dof.setGyroODR(dof.G_ODR_760_BW_50);
    break;
  case 'E':
  case 'e':
    dof.setGyroODR(dof.G_ODR_760_BW_100);
    break;
  }

  // Print all possible range selections for the magnetometer:
  Serial.println(F("Set Magnetometer ODR (Hz):"));
  Serial.println(F("\t1) 3.125 \t 4) 25"));
  Serial.println(F("\t2) 6.25  \t 5) 50"));
  Serial.println(F("\t3) 12.5  \t 6) 100"));
  // Wait for a character to come in:
  while (Serial.available() < 1)
    ;
  c = Serial.read();
  // Use the setMagODR function to set the magnetometer
  // data rate to any of the possible ranges. These ranges
  // are all defined in SFE_LSM9DS0.h.
  switch (c)
  {
  case '1':
    dof.setMagODR(dof.M_ODR_3125);
    break;
  case '2':
    dof.setMagODR(dof.M_ODR_625);
    break;
  case '3':
    dof.setMagODR(dof.M_ODR_125);
    break;
  case '4':
    dof.setMagODR(dof.M_ODR_25);
    break;
  case '5':
    dof.setMagODR(dof.M_ODR_50);
    break;
  case '6':
    dof.setMagODR(dof.M_ODR_100);
    break;
  }
}

void printMenu()
{
  Serial.println();
  Serial.println(F("////////////////////////////////////////////"));
  Serial.println(F("// LSM9DS0 Super Awesome Amazing Fun Time //"));
  Serial.println(F("////////////////////////////////////////////"));
  Serial.println();
  Serial.println(F("1) Stream Accelerometer"));
  Serial.println(F("2) Stream Gyroscope"));
  Serial.println(F("3) Stream Magnetometer"));
  Serial.println(F("4) Stream output from all sensors"));
  Serial.println(F("5) Set Sensor Scales"));
  Serial.println(F("6) Switch To/From Raw/Calculated Readings"));
  Serial.println(F("7) Set Output Data Rates")); 
  Serial.println(); 
}

// parseMenu() takes a char parameter, which should map to one of
// the defined menu options. A switch statement will control what
// happens based on the given character input.
void parseMenu(char c)
{
  switch (c)
  {
  case '1':
    while(!Serial.available())
      printAccel(); // Print accelerometer values
    break;
  case '2':
    while(!Serial.available())
      printGyro(); // Print gyroscope values
    break;
  case '3':
    while(!Serial.available())
      printMag(); // Print magnetometer values
    break;
  case '4':
    while(!Serial.available())
      streamAll(); // Print all sensor readings
    break;
  case '5':
    setScale(); // Set the ranges of each sensor
    break;
  case '6':
    setRaw(); // Switch between calculated and raw output
    break;
  case '7':
    setODR(); // Set the data rates of each sensor
    break;
  }
}

