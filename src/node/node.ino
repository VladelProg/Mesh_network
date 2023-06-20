#include "LoRaWan_APP.h"
#include "Arduino.h"
#include "Wire.h"

 //---------- MPU6050 Measurement & Filtering Range ----------------------------------------------
#define AFS_SEL 2  // Accelerometer Configuration Settings   AFS_SEL=2, Full Scale Range = +/- 8 [g]
#define DLPF_SEL  0  // DLPF Configuration Settings  Accel BW 260Hz, Delay 0ms / Gyro BW 256Hz, Delay 0.98ms, Fs 8KHz 
#define IMUAddress 0x68 // адрес датчика

//---------- Variables for gravity --------------------------------------------------------------
int accX, accY, accZ;  // Accelerometer values
long Cal_accX, Cal_accY, Cal_accZ; // Calibration values
float GaccX, GaccY, GaccZ; // Convert accelerometer to gravity value
float Min_GaccX=0, Max_GaccX=0, PtoP_GaccX, Min_GaccY=0, Max_GaccY=0, PtoP_GaccY, Min_GaccZ=0, Max_GaccZ=0, PtoP_GaccZ; // Finding Min, Max & Peak to Peak of gravity value
float Min = 0, Max = 0; // Initial value of Min, Max
int cnt; // Count of calibration process
float Grvt_unit; // Gravity value unit
long period, prev_time; // Period of calculation
int tempRaw;
int dataTosend = 0;
// Lora and LoraWan Settings 
#ifndef LoraWan_RGB
#define LoraWan_RGB 0
#endif

#define RF_FREQUENCY                                868000000 // Hz
#define TX_OUTPUT_POWER                             14        // dBm
#define LORA_BANDWIDTH                              0         // [0: 125 kHz,
                                                           //  1: 250 kHz,
                                                           //  2: 500 kHz,
                                                           //  3: Reserved]
#define LORA_SPREADING_FACTOR                        7         // [SF7..SF12]
#define LORA_CODINGRATE                             1         // [1: 4/5,
                                                           //  2: 4/6,
                                                           //  3: 4/7,
                                                           //  4: 4/8]
#define LORA_PREAMBLE_LENGTH                        8         // Same for Tx and Rx
#define LORA_SYMBOL_TIMEOUT                         0         // Symbols
#define LORA_FIX_LENGTH_PAYLOAD_ON                   false
#define LORA_IQ_INVERSION_ON                        false

#define RX_TIMEOUT_VALUE                            1000
#define BUFFER_SIZE                                 40 // Define the payload size here


char txpacket[BUFFER_SIZE];
char rxpacket[BUFFER_SIZE];

static RadioEvents_t RadioEvents;
int16_t gyroX;
int16_t gyroY;
int16_t gyroZ;

uint16_t boardID = 1, txNumber;

int fracPart(double val, int n)
{
  if( val >= 0 )
    return (int)((val - (int)(val))*pow(10,n));
  else
    return (int)(((int)(val) - val)*pow(10,n));
}

void setup() {

  Serial.begin(115200);
  
  init_MPU6050();

  Gravity_Range_Option();

  Calib_MPU6050(); // Calculating calibration value


  txNumber = 0;

  Radio.Init( &RadioEvents );
  Radio.SetChannel( RF_FREQUENCY );
  Radio.SetTxConfig( MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
                   LORA_SPREADING_FACTOR, LORA_CODINGRATE,
                   LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
                   true, 0, 0, LORA_IQ_INVERSION_ON, 3000 );
}

void init_MPU6050(){
  //MPU6050 Initializing & Reset
  Wire.beginTransmission(IMUAddress);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);

  //MPU6050 Clock Type
  Wire.beginTransmission(IMUAddress);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0x03);     // Selection Clock 'PLL with Z axis gyroscope reference'
  Wire.endTransmission(true);

  //MPU6050 acccelerometer Configuration Setting
  Wire.beginTransmission(IMUAddress);
  Wire.write(0x1C);  // acccelerometer Configuration register
  if(AFS_SEL == 0) Wire.write(0x00);     // AFS_SEL=0, Full Scale Range = +/- 2 [g]
  else if(AFS_SEL == 1) Wire.write(0x08);     // AFS_SEL=1, Full Scale Range = +/- 4 [g]
  else if(AFS_SEL == 2) Wire.write(0x10);     // AFS_SEL=2, Full Scale Range = +/- 8 [g]
  else  Wire.write(0x18);     // AFS_SEL=3, Full Scale Range = +/- 10 [g]
  Wire.endTransmission(true);

  //MPU6050 DLPF(Digital Low Pass Filter)
  Wire.beginTransmission(IMUAddress);
  Wire.write(0x1A);  // DLPF_CFG register
  if(DLPF_SEL == 0) Wire.write(0x00);     // acccel BW 260Hz, Delay 0ms / Gyro BW 256Hz, Delay 0.98ms, Fs 8KHz 
  else if(DLPF_SEL == 1)  Wire.write(0x01);     // acccel BW 184Hz, Delay 2ms / Gyro BW 188Hz, Delay 1.9ms, Fs 1KHz 
  else if(DLPF_SEL == 2)  Wire.write(0x02);     // acccel BW 94Hz, Delay 3ms / Gyro BW 98Hz, Delay 2.8ms, Fs 1KHz 
  else if(DLPF_SEL == 3)  Wire.write(0x03);     // acccel BW 44Hz, Delay 4.9ms / Gyro BW 42Hz, Delay 4.8ms, Fs 1KHz 
  else if(DLPF_SEL == 4)  Wire.write(0x04);     // acccel BW 21Hz, Delay 8.5ms / Gyro BW 20Hz, Delay 8.3ms, Fs 1KHz 
  else if(DLPF_SEL == 5)  Wire.write(0x05);     // acccel BW 10Hz, Delay 13.8ms / Gyro BW 10Hz, Delay 13.4ms, Fs 1KHz 
  else  Wire.write(0x06);     // acccel BW 5Hz, Delay 19ms / Gyro BW 5Hz, Delay 18.6ms, Fs 1KHz 
  Wire.endTransmission(true);
}

void Gravity_Range_Option(){
  switch(AFS_SEL) { // Selecting Gravity unit value
    case 0:
      Grvt_unit = 16384;
      break;
    case 1:
      Grvt_unit = 8192;
      break;
    case 2:
      Grvt_unit = 4096;
      break;
    case 3:
      Grvt_unit = 3276.8;
      break;
  }
}  


void getValues() {
  /* Update all the values */
  Wire.beginTransmission(IMUAddress);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(IMUAddress, 14, true); // request a total of 14 registers
  accX = Wire.read() << 8 | Wire.read(); // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
  accY = Wire.read() << 8 | Wire.read(); // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  accZ = Wire.read() << 8 | Wire.read(); // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  tempRaw = Wire.read() << 8 | Wire.read(); // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  tempRaw = ((float)tempRaw + 12412.0) / 340.0;
  gyroX = Wire.read() << 8 | Wire.read(); // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  gyroY = Wire.read() << 8 | Wire.read(); // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  gyroZ = Wire.read() << 8 | Wire.read(); // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
  //temp = ((float)tempRaw + 12412.0) / 340.0;
}

void loop()
{

  getValues();

  Calc_Grvt();

  Display_Grvt();

  char str[160];
  uint32_t starttime = millis();
  getValues();
  //int temp = 0;

  //delay(500);
  dataTosend = (accX + accY + accZ)/3;
	//if( ++txNumber >= 10000) txNumber = 0;
	sprintf(txpacket, "%d#%d", dataTosend,tempRaw);
	Serial.printf("sending \"%s\", length %d\r\n", txpacket, strlen(txpacket));

	Radio.Send( (uint8_t *)txpacket, strlen(txpacket) ); //send the package out

  //turnOnRGB(bOn ? COLOR_SEND : COLOR_RECEIVED, 0); //change rgb color
  //bOn = !bOn;

  delay(10);
  //dataTosend = 0;
}

void Calib_MPU6050() {  
  for(int i = 0 ; i < 2000 ; i++) { // Summing Iteration for finding calibration value
    if(i % 200 == 0) {  // Display progress every 200 cycles
      cnt++;
      if(cnt == 1)  { // Characcters to display first
        Serial.print("Calculating .");
        */
      }
      else  { // Display progress by point
        Serial.print(".");
      }      
    }    
    
    getValues(); // Read acccelerometer data
    
    delay(10);

    // Sum data
    Cal_accX += accX;
    Cal_accY += accY;
    Cal_accZ += accZ;
  }

  // Average Data
  Cal_accX /= 2000;
  Cal_accY /= 2000;
  Cal_accZ /= 2000;

  // Serial Print
  Serial.println("");
  Serial.println("End of Calculation");
  Serial.print("Cal_accX = "); Serial.print(Cal_accX);
  Serial.print(" | Cal_accY = "); Serial.print(Cal_accY);
  Serial.print(" | Cal_accZ = "); Serial.println(Cal_accZ);
}

void Calc_Grvt() {
  accX = (accX - Cal_accX);  // Calibrated acccelerometer value
  accY = (accY - Cal_accY);  // Calibrated acccelerometer value
  accZ = (accZ - Cal_accZ);  // Calibrated acccelerometer value
  
  GaccX = accX / Grvt_unit; // Converting the Calibrated value to Gravity value
  GaccY = accY / Grvt_unit; // Converting the Calibrated value to Gravity value
  GaccZ = accZ / Grvt_unit; // Converting the Calibrated value to Gravity value

  //---------- Calculating Min, Max & Peak to Peak of Gravity --------------------------------------

  Min_GaccX = min(Min_GaccX, GaccX);
  Max_GaccX = max(Max_GaccX, GaccX);
  PtoP_GaccX = Max_GaccX - Min_GaccX;

  Min_GaccY = min(Min_GaccY, GaccY);
  Max_GaccY = max(Max_GaccY, GaccY);
  PtoP_GaccY = Max_GaccY - Min_GaccY;

  Min_GaccZ = min(Min_GaccZ, GaccZ);
  Max_GaccZ = max(Max_GaccZ, GaccZ);
  PtoP_GaccZ = Max_GaccZ - Min_GaccZ;
}  

void Display_Grvt() {
  //---------- Serial print ----------------------------------------------------------------------
  Serial.print("accX= " + String(accX));
  Serial.print(" |accY= " + String(accY));
  Serial.println(" |accZ= " + String(accZ));

  //---------- LCD Display -----------------------------------------------------------------------
  period = millis() - prev_time;
  
  if(period > 1000) {
    
    prev_time = millis();
    Min_GaccX = 0;
    Max_GaccX = 0;
    Min_GaccY = 0;
    Max_GaccY = 0;
    Min_GaccZ = 0;
    Max_GaccZ = 0;
  }
}
