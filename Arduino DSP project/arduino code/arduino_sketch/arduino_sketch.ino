#include <MPU6050_tockn.h>
#include <Wire.h>


//define the pins and their function
#define brightness_pin  3
#define south_segment 2
#define south_east_segment  4
#define dot_segment 5
#define south_west_segment 6
#define north_east_segment 7
#define north_segment 8
#define north_west_segment 9
#define center_segment 10


bool filter = true; //determines if the signals should be filtered or not
int T = 1; //Time between two samples. In miliseconds!
float angleX, angleY;
float offsetX = 0, offsetY = 0;
float voltage = 0;

MPU6050 mpu(Wire, 1, 0); //Construct the object without a filter.


void turn_on_seven_segment(float angleX, float angleY, int neighborhood_tolerance = 10)
{
  //The following two lines of code are done because the orientation of the sensor is reversed.
  //We want the north and east to be indicated by positive angle values, while the 
  //south and west are indicated using negative angle values.
  angleY = -angleY;  
  angleX = -angleX;

  //begin by turning off all pins
  for(int i = 0; i <= 10; i++)
  {
    if(i != 3) //don't turn off the brightness pin!
      digitalWrite(i, LOW);
  }


  if(angleX < neighborhood_tolerance and angleX > -neighborhood_tolerance)
  {
    if(angleY < neighborhood_tolerance and angleY > -neighborhood_tolerance)
    {
      digitalWrite(center_segment, HIGH);
    }
    else if(angleY > neighborhood_tolerance)
    {
      digitalWrite(north_segment, HIGH);
    }
    else if(angleY < -neighborhood_tolerance)
    {
      digitalWrite(south_segment, HIGH);
    }
  }  
  else if(angleX > neighborhood_tolerance)
  {
    if(angleY < neighborhood_tolerance and angleY > -neighborhood_tolerance)
    {
      digitalWrite(north_east_segment, HIGH);
      digitalWrite(south_east_segment, HIGH);
    }
    else if(angleY > neighborhood_tolerance)
    {
      digitalWrite(north_east_segment, HIGH);
      digitalWrite(north_segment, HIGH);
    }
    else if(angleY < -neighborhood_tolerance)
    {
      digitalWrite(south_east_segment, HIGH);
      digitalWrite(south_segment, HIGH);
    }
  }
  else if(angleX < -neighborhood_tolerance)
  {
    if(angleY < neighborhood_tolerance and angleY > -neighborhood_tolerance)
    {
      digitalWrite(north_west_segment, HIGH);
      digitalWrite(south_west_segment, HIGH);
    }
    else if(angleY > neighborhood_tolerance)
    {
      digitalWrite(north_west_segment, HIGH);
      digitalWrite(north_segment, HIGH);
    }
    else if(angleY < -neighborhood_tolerance)
    {
      digitalWrite(south_west_segment, HIGH);
      digitalWrite(south_segment, HIGH);
    }
  }
  else
  {
    digitalWrite(center_segment, LOW);
  }
}

float y_angle_x[3] = {0.0, 0.0, 0.0}; //an empty array for storing the current and previous two values of the output of the filter for MPU6050 (x_angle readings)
float x_angle_x[3] = {0.0, 0.0, 0.0}; //an empty array for storing the current and previous two values of the readings from the MPU6050 sensor (X_angle readings)

float read_mpu_6050_x(bool filter)
{
  if(filter)
  {
    x_angle_x[0] = mpu.getAngleX();
    y_angle_x[0] = 1.95557824*y_angle_x[1] - 0.95654368*y_angle_x[2] + 0.00024136 * x_angle_x[0] + 0.00048272*x_angle_x[1] + 0.00024136*x_angle_x[2];
    
    //delay the samples
    y_angle_x[2] = y_angle_x[1];
    y_angle_x[1] = y_angle_x[0];
    x_angle_x[2] = x_angle_x[1];
    x_angle_x[1] = x_angle_x[0];

    return y_angle_x[0];
  }
  else
  {
    return mpu.getAngleX();
  }
}


float y_angle_y[3] = {0.0, 0.0, 0.0}; //an empty array for storing the current and previous two values of the output of the filter for MPU6050 (y_angle_readings)
float x_angle_y[3] = {0.0, 0.0, 0.0}; //an empty array for storing the current and previous two values of the readings from the MPU6050 sensor (y_angle readings)

float read_mpu_6050_y(bool filter)
{
  if(filter)
  {
    x_angle_y[0] = mpu.getAngleY();
    y_angle_y[0] = 1.95557824*y_angle_y[1] - 0.95654368*y_angle_y[2] + 0.00024136 * x_angle_y[0] + 0.00048272*x_angle_y[1] + 0.00024136*x_angle_y[2];
    
    //delay the samples
    y_angle_y[2] = y_angle_y[1];
    y_angle_y[1] = y_angle_y[0];
    x_angle_y[2] = x_angle_y[1];
    x_angle_y[1] = x_angle_y[0];

    return y_angle_y[0];
  }
  else
  {
    return mpu.getAngleY();
  }
}


void setup() {
  Serial.begin(115200);
  Wire.begin();
  mpu.begin();
  
  //Calculate the sensor's starting position!
  Serial.println("\n\nCalculating the sensor's starting position. Please do not move the sensor!");
  int samples_to_get = 10000;
  for(int i = 0; i < samples_to_get; i++)
  {
    offsetX += read_mpu_6050_x(filter = filter);
    offsetY += read_mpu_6050_y(filter = filter);
    delay(T);
  }

  offsetX /= samples_to_get;
  offsetY /= samples_to_get;

  Serial.print("OffsetX: ");
  Serial.println(offsetX);
  Serial.print("OffsetY: ");
  Serial.println(offsetY);
  delay(5000);

  //set all the pins of the seven segment to output
  for(int i = 2; i <= 10; i++)
  {
    pinMode(i, OUTPUT);
  }
}


void loop() {
  mpu.update();
  angleX = read_mpu_6050_x(filter = filter) - offsetX;
  angleY = read_mpu_6050_y(filter = filter) - offsetY;
  voltage = (analogRead(A2) * 255.0 / 1024.0); //read and normalize the voltage to be between 0 and 255
  analogWrite(brightness_pin, int(voltage));
  turn_on_seven_segment(angleX, angleY);

  Serial.print("AngleX:");
  Serial.print(angleX);
  Serial.print(',');
  Serial.print("AngleY:");
  Serial.print(angleY);
  Serial.print(',');
  Serial.print("Voltage:");
  Serial.println(voltage);

  delay(T);
}
