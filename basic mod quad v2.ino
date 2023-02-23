#include "ICM_20948.h"
#define SERIAL_PORT Serial
#include <deque>
#include <iostream>
#include <Keyboard.h> // The main library for sending keystrokes.


using namespace std;

#define WIRE_PORT Wire // Your desired Wire port
// The value of the last bit of the I2C address.
// On the SparkFun 9DoF IMU breakout the default is 1, and when the ADR jumper is closed the value becomes 0
#define AD0_VAL 1
ICM_20948_I2C myICM; // Create an ICM_20948_I2C object
deque<double> dq1 = {0,0,0,0,0,0,0,0,0,0};
deque<double> dq2 = {0,0,0,0,0,0,0,0,0,0};
int ite=0;
void setup()
{
  
  SERIAL_PORT.begin(115200); // Start the serial console
  Keyboard.begin();
  SERIAL_PORT.println(F("ICM-20948 Example"));

  delay(100);

  while (SERIAL_PORT.available()) // Make sure the serial RX buffer is empty
    SERIAL_PORT.read();

  SERIAL_PORT.println(F("Press any key to continue..."));

  while (!SERIAL_PORT.available()) // Wait for the user to press a key (send any serial character)
    ;

  WIRE_PORT.begin();
  WIRE_PORT.setClock(400000);

  //myICM.enableDebugging(); // Uncomment this line to enable helpful debug messages on Serial

  bool initialized = false;
  while (!initialized)
  {

    // Initialize the ICM-20948
    // If the DMP is enabled, .begin performs a minimal startup. We need to configure the sample mode etc. manually.
    myICM.begin(WIRE_PORT, AD0_VAL);

    SERIAL_PORT.print(F("Initialization of the sensor returned: "));
    SERIAL_PORT.println(myICM.statusString());
    if (myICM.status != ICM_20948_Stat_Ok)
    {
      SERIAL_PORT.println(F("Trying again..."));
      delay(500);
    }
    else
    {
      initialized = true;
    }
  }

  SERIAL_PORT.println(F("Device connected!"));

  bool success = true; // Use success to show if the DMP configuration was successful

  // Initialize the DMP. initializeDMP is a weak function. You can overwrite it if you want to e.g. to change the sample rate
  success &= (myICM.initializeDMP() == ICM_20948_Stat_Ok);


  // Enable the DMP orientation sensor
  success &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_ORIENTATION) == ICM_20948_Stat_Ok);
  success &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_RAW_GYROSCOPE) == ICM_20948_Stat_Ok);

  success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Quat9, 0) == ICM_20948_Stat_Ok); // Set to the maximum

  // Enable the FIFO
  success &= (myICM.enableFIFO() == ICM_20948_Stat_Ok);

  // Enable the DMP
  success &= (myICM.enableDMP() == ICM_20948_Stat_Ok);

  // Reset DMP
  success &= (myICM.resetDMP() == ICM_20948_Stat_Ok);

  // Reset FIFO
  success &= (myICM.resetFIFO() == ICM_20948_Stat_Ok);

  // Check success
  if (success)
  {
    SERIAL_PORT.println(F("DMP enabled!"));
  }
  else
  {
    SERIAL_PORT.println(F("Enable DMP failed!"));
    SERIAL_PORT.println(F("Please check that you have uncommented line 29 (#define ICM_20948_USE_DMP) in ICM_20948_C.h..."));
    while (1)
      ; // Do nothing more
  }
}

void loop()
{
  icm_20948_DMP_data_t data;
  myICM.readDMPdataFromFIFO(&data);

  if ((myICM.status == ICM_20948_Stat_Ok) || (myICM.status == ICM_20948_Stat_FIFOMoreDataAvail)) // Was valid data available?
  {
    double q1 = ((double)data.Quat9.Data.Q1) / 1073741824.0; // Convert to double. Divide by 2^30
    double q2 = ((double)data.Quat9.Data.Q2) / 1073741824.0; // Convert to double. Divide by 2^30
    double q3 = ((double)data.Quat9.Data.Q3) / 1073741824.0; // Convert to double. Divide by 2^30
    double q0 = sqrt(1.0 - ((q1 * q1) + (q2 * q2) + (q3 * q3)));
    if ((data.header & DMP_header_bitmap_Quat9) > 0) // We have asked for orientation data so we should receive Quat9
    {
      q1 = ((double)data.Quat9.Data.Q1) / 1073741824.0; // Convert to double. Divide by 2^30
      q2 = ((double)data.Quat9.Data.Q2) / 1073741824.0; // Convert to double. Divide by 2^30
      q3 = ((double)data.Quat9.Data.Q3) / 1073741824.0; // Convert to double. Divide by 2^30
      q0 = sqrt(1.0 - ((q1 * q1) + (q2 * q2) + (q3 * q3)));
      
      if (ite%7==0)
      {
        dq1.push_back(10*q1);	
        dq1.pop_front();
        dq2.push_back(10*q2);	
        dq2.pop_front();
      }
      

      /*for (int i=0 ; i<21 ; i++)
      {
        arrayQ1[i]=q1;
      }*/
      /*dq1.push_back(10*q1);	
      dq1.pop_front();

      if (dq1[0]<dq1[1] && dq1[1]<dq1[2] && dq1[2]<dq1[3] && dq1[3]<dq1[4] && dq1[4]<dq1[5] && dq1[5]<dq1[6] && dq1[6]<dq1[7] && dq1[7]<dq1[8] && dq1[8]<dq1[9] )
      {
        SERIAL_PORT.print("L to R");
      }
      else if (dq1[0]>dq1[1] && dq1[1]>dq1[2] && dq1[2]>dq1[3] && dq1[3]>dq1[4] && dq1[4]>dq1[5] && dq1[5]>dq1[6] && dq1[6]>dq1[7] && dq1[7]>dq1[8] && dq1[8]>dq1[9])
      {
        SERIAL_PORT.print("R to L");
      }*/
    
      


      SERIAL_PORT.print(F("Q1:"));
      SERIAL_PORT.print(q1, 3);
      SERIAL_PORT.print(F(" Q2:"));
      SERIAL_PORT.print(q2, 3);
      SERIAL_PORT.print(F(" Q3:"));
      SERIAL_PORT.print(q3, 3);
    }

    if ((data.header & DMP_header_bitmap_Gyro) > 0) // Check for Gyro
    {
      float x = (float)data.Raw_Gyro.Data.X; // Extract the raw gyro data
      float y = (float)data.Raw_Gyro.Data.Y;
      float z = (float)data.Raw_Gyro.Data.Z;

      //SERIAL_PORT.print(x);SERIAL_PORT.print(",");SERIAL_PORT.print(y);SERIAL_PORT.print(",");SERIAL_PORT.print(z);
      if(Keyboard.press('r'))
      {
        Serial.print("non-idle");
        

        if (dq1[0]<dq1[1] && dq1[1]<dq1[2] && dq1[2]<dq1[3] && dq1[3]<dq1[4] && dq1[4]<dq1[5] && dq1[5]<dq1[6] && dq1[6]<dq1[7] && dq1[7]<dq1[8] && dq1[8]<dq1[9]) 
        //    && dq2[0]>dq2[1] && dq2[1]>dq2[2] && dq2[2]>dq2[3] && dq2[3]>dq2[4] && dq2[4]>dq2[5] && dq2[5]>dq2[6] && dq2[6]>dq2[7] && dq2[7]>dq2[8] && dq2[8]>dq2[9])
        {
          SERIAL_PORT.print("R to L");
        }
        else if (dq1[0]>dq1[1] && dq1[1]>dq1[2] && dq1[2]>dq1[3] && dq1[3]>dq1[4] && dq1[4]>dq1[5] && dq1[5]>dq1[6] && dq1[6]>dq1[7] && dq1[7]>dq1[8] && dq1[8]>dq1[9])
        {
          SERIAL_PORT.print("L to R");
        }
      }
      else
        Serial.print("idle");
 
    }
    SERIAL_PORT.println();
  }

  if (myICM.status != ICM_20948_Stat_FIFOMoreDataAvail) // If more data is available then we should read it right away - and not delay
  {
    delay(10);
  }
  ite=ite+1;

}




