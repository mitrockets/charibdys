float *get_9DOF_data() {
  sensors_event_t accel_event_one;
  sensors_vec_t   orientation_one;
  float 9DOF_values = [3]

/* Calculate pitch and roll from the raw accelerometer data */
accel.getEvent(&accel_event_one);
if (dof.accelGetOrientation(&accel_event_one, &orientation_one))
{
  /* 'orientation' should have valid .roll and .pitch fields */
  Serial.print(F("Roll: "));
  Serial.print(orientation_one.roll);
  9DOF_values[0] = orientation_one.roll;
  Serial.print(F("; "));
  Serial.print(F("Pitch: "));
  Serial.print(orientation_one.pitch);
  9DOF_values[1] = orientation_one.pitch;
  Serial.print(F("; "));
}

sensors_event_t mag_event_one;
  
/* Calculate the heading using the magnetometer */
mag.getEvent(&mag_event_one);
if (dof.magGetOrientation(SENSOR_AXIS_Z, &mag_event_one, &orientation_one))
{
  /* 'orientation' should have valid .heading data now */
  Serial.print(F("Heading: "));
  Serial.print(orientation_one.heading);
  9DOF_values = orientation_one.heading;
  Serial.print(F("; "));
