/*
 * This example bypasses the hardware motion interrupt pin
 * and polls the motion data registers at a fixed interval
 */

#include <SPI.h>
#include <avr/pgmspace.h>

// Registers
#define Product_ID  0x00
#define Revision_ID 0x01
#define Motion  0x02
#define Delta_X_L 0x03
#define Delta_X_H 0x04
#define Delta_Y_L 0x05
#define Delta_Y_H 0x06
#define SQUAL 0x07
#define Raw_Data_Sum  0x08
#define Maximum_Raw_data  0x09
#define Minimum_Raw_data  0x0A
#define Shutter_Lower 0x0B
#define Shutter_Upper 0x0C
#define Control 0x0D
#define Res_L 0x0E
#define Res_H 0x0F
#define Config2 0x10
#define Angle_Tune  0x11
#define Frame_Capture 0x12
#define SROM_Enable 0x13
#define Run_Downshift 0x14
#define Rest1_Rate_Lower  0x15
#define Rest1_Rate_Upper  0x16
#define Rest1_Downshift 0x17
#define Rest2_Rate_Lower  0x18
#define Rest2_Rate_Upper  0x19
#define Rest2_Downshift 0x1A
#define Rest3_Rate_Lower  0x1B
#define Rest3_Rate_Upper  0x1C
#define Observation 0x24
#define Data_Out_Lower  0x25
#define Data_Out_Upper  0x26
#define Raw_Data_Dump 0x29
#define SROM_ID 0x2A
#define Min_SQ_Run  0x2B
#define Raw_Data_Threshold  0x2C
#define Config5 0x2F
#define Power_Up_Reset  0x3A
#define Shutdown  0x3B
#define Inverse_Product_ID  0x3F
#define LiftCutoff_Tune3  0x41
#define Angle_Snap  0x42
#define LiftCutoff_Tune1  0x4A
#define Motion_Burst  0x50
#define LiftCutoff_Tune_Timeout 0x58
#define LiftCutoff_Tune_Min_Length  0x5A
#define SROM_Load_Burst 0x62
#define Lift_Config 0x63
#define Raw_Data_Burst  0x64
#define LiftCutoff_Tune2  0x65


// functions


void crossProduct(float a[3], float b[3], float result[3]) {
  result[0] = a[1] * b[2] - a[2] * b[1];
  result[1] = a[2] * b[0] - a[0] * b[2];
  result[2] = a[0] * b[1] - a[1] * b[0];
}

void printMatrix(float matrix[3][3]) {
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      //Serial.print(matrix[i][j]);
      //Serial.print(" ");
    }
    //Serial.println();
  }
}

float calculate_Adet(float A[3][3]) {
  float Adet = A[0][0] * (A[1][1] * A[2][2] - A[2][1] * A[1][2]) -
               A[0][1] * (A[1][0] * A[2][2] - A[2][0] * A[1][2]) +
               A[0][2] * (A[1][0] * A[2][1] - A[2][0] * A[1][1]);
  return Adet;
}

// to calculate the inverse of a 3x3 matrix
bool calculate_Ainv(float A[3][3], float Ainv[3][3]) {
  float Adet = calculate_Adet(A);
  
  if (Adet == 0) {
    return false; // Matrix is not invertible if determinant is zero
  }

  Ainv[0][0] = (A[1][1] * A[2][2] - A[2][1] * A[1][2]) / Adet;
  Ainv[0][1] = (A[0][2] * A[2][1] - A[0][1] * A[2][2]) / Adet;
  Ainv[0][2] = (A[0][1] * A[1][2] - A[0][2] * A[1][1]) / Adet;

  Ainv[1][0] = (A[1][2] * A[2][0] - A[1][0] * A[2][2]) / Adet;
  Ainv[1][1] = (A[0][0] * A[2][2] - A[0][2] * A[2][0]) / Adet;
  Ainv[1][2] = (A[1][0] * A[0][2] - A[0][0] * A[1][2]) / Adet;

  Ainv[2][0] = (A[1][0] * A[2][1] - A[2][0] * A[1][1]) / Adet;
  Ainv[2][1] = (A[2][0] * A[0][1] - A[0][0] * A[2][1]) / Adet;
  Ainv[2][2] = (A[0][0] * A[1][1] - A[1][0] * A[0][1]) / Adet;

  return true;
}

void calculate_w(float vsi, float vsj, float vsk, float Ainv[3][3], float w[3]) {

// calculate the w vector with matrix multiplication
w[0] = Ainv[0][0] * vsi + Ainv[0][1] * vsj + Ainv[0][2] * vsk;
w[1] = Ainv[1][0] * vsi + Ainv[1][1] * vsj + Ainv[1][2] * vsk;
w[2] = Ainv[2][0] * vsi + Ainv[2][1] * vsj + Ainv[2][2] * vsk;


}

void calculate_w_fast(float vsi, float vsj, float vsk, float Ainv[3][3], float w[3]) {
    #define MULT(i) Ainv[i][0] * vsi + Ainv[i][1] * vsj + Ainv[i][2] * vsk
    w[0] = MULT(0);
    w[1] = MULT(1);
    w[2] = MULT(2);
    #undef MULT
}

// Function to calculate w2 values
void calculate_w2(float vsi, float vsj, float vsk, float Ainv[3][3], float w2[3]) {
  
    float abs_sum = fabs(Ainv[0][0] * vsi) + fabs(Ainv[0][1] * vsj) + fabs(Ainv[0][1] * vsk);
    w2[0] = (abs_sum != 0) ? fabs(Ainv[0][0] * vsi + Ainv[0][1] * vsj + Ainv[0][2] * vsk) / abs_sum : 0;
    abs_sum = fabs(Ainv[1][0] * vsi) + fabs(Ainv[1][1] * vsj) + fabs(Ainv[1][1] * vsk);
    w2[1] = (abs_sum != 0) ? fabs(Ainv[1][0] * vsi + Ainv[1][1] * vsj + Ainv[1][2] * vsk) / abs_sum : 0;
    abs_sum = fabs(Ainv[2][0] * vsi) + fabs(Ainv[2][1] * vsj) + fabs(Ainv[2][1] * vsk);
    w2[2] = (abs_sum != 0) ? fabs(Ainv[2][0] * vsi + Ainv[2][1] * vsj + Ainv[2][2] * vsk) / abs_sum : 0;
}

void calculate_w2_fast(float vsi, float vsj, float vsk, float Ainv[3][3], float w2[3]) {
    #define CALC(i) \
        sum = fabs(Ainv[i][0] * vsi) + fabs(Ainv[i][1] * vsj) + fabs(Ainv[i][2] * vsk); \
        w2[i] = sum ? fabs(Ainv[i][0] * vsi + Ainv[i][1] * vsj + Ainv[i][2] * vsk) / sum : 0

    float sum;
    CALC(0);
    CALC(1);
    CALC(2);
    #undef CALC
}

// Function to calculate the weighted average of angular velocity
float calculate_avg_w(float ws[][3], float absAdets[], float w2s[][3], int num_triplets, float avg_w[3]) {
    float xn = 0;  // Numerators for wx, wy, wz
    float xd = 0;  // Denominators for wx, wy, wz
    float yn = 0;
    float yd = 0;
    float zn = 0;
    float zd = 0;

    // Loop through all triplets
    for (int i = 0; i < num_triplets; i++) {
        // Weighted sum for x, y, and z components
        xn += absAdets[i] * w2s[i][0] * ws[i][0];  // wx component
        xd += absAdets[i] * w2s[i][0];             // wx denominator

        yn += absAdets[i] * w2s[i][1] * ws[i][1];  // wy component
        yd += absAdets[i] * w2s[i][1];             // wy denominator

        zn += absAdets[i] * w2s[i][2] * ws[i][2];  // wz component
        zd += absAdets[i] * w2s[i][2];             // wz denominator
    }

    // Calculate final wx, wy, and wz (check for division by zero)
    float wx = (xd != 0) ? (xn / xd) : 0;
    float wy = (yd != 0) ? (yn / yd) : 0;
    float wz = (zd != 0) ? (zn / zd) : 0;

    // Check for NaN values (Arduino doesn't have std::isnan, so check for invalid values)
    if (isnan(wx)) wx = 0;
    if (isnan(wy)) wy = 0;
    if (isnan(wz)) wz = 0;

    // Update the last_avg_w array with the new weighted average
    avg_w[0] = wx;
    avg_w[1] = wy;
    avg_w[2] = wz;

}

void calculate_avg_w_fast(float ws[][3], float absAdets[], float w2s[][3], int num_triplets, float avg_w[3]) {
    // Direct calculation for all components at once
    float xn = absAdets[0] * w2s[0][0] * ws[0][0] +
               absAdets[1] * w2s[1][0] * ws[1][0] +
               absAdets[2] * w2s[2][0] * ws[2][0] +
               absAdets[3] * w2s[3][0] * ws[3][0];

    float xd = absAdets[0] * w2s[0][0] +
               absAdets[1] * w2s[1][0] +
               absAdets[2] * w2s[2][0] +
               absAdets[3] * w2s[3][0];

    float yn = absAdets[0] * w2s[0][1] * ws[0][1] +
               absAdets[1] * w2s[1][1] * ws[1][1] +
               absAdets[2] * w2s[2][1] * ws[2][1] +
               absAdets[3] * w2s[3][1] * ws[3][1];

    float yd = absAdets[0] * w2s[0][1] +
               absAdets[1] * w2s[1][1] +
               absAdets[2] * w2s[2][1] +
               absAdets[3] * w2s[3][1];

    float zn = absAdets[0] * w2s[0][2] * ws[0][2] +
               absAdets[1] * w2s[1][2] * ws[1][2] +
               absAdets[2] * w2s[2][2] * ws[2][2] +
               absAdets[3] * w2s[3][2] * ws[3][2];

    float zd = absAdets[0] * w2s[0][2] +
               absAdets[1] * w2s[1][2] +
               absAdets[2] * w2s[2][2] +
               absAdets[3] * w2s[3][2];

    avg_w[0] = xd != 0 ? xn / xd : 0;
    avg_w[1] = yd != 0 ? yn / yd : 0;
    avg_w[2] = zd != 0 ? zn / zd : 0;
}



//Set this to what pin your "INT0" hardware interrupt feature is on
#define Motion_Interrupt_Pin0 9
#define Motion_Interrupt_Pin1 7

const int ncs0 = 8;  //This is the SPI "slave select" pin that the sensor is hooked up to
const int ncs1 = 10;  //This is the SPI "slave select" pin that the sensor is hooked up to
const int rst0 = 5;
const int rst1 = 6;

byte initComplete=0;
volatile int xdat0[2];
volatile int ydat0[2];
volatile int xdat1[2];
volatile int ydat1[2];
volatile byte movementflag0=0;
volatile byte movementflag1=0;

byte testctr=0;
unsigned long currTime;
unsigned long timer;
unsigned long pollTimer;

//Be sure to add the SROM file into this sketch via "Sketch->Add File"
extern const unsigned short firmware_length;
extern const unsigned char firmware_data[];

// Sensors Definitions

float S0p[3] = {15.26, 8.8, -6.4};
float S0xs[3] = {-0.5, 0.87, 0};
float S0ys[3] = {0.29, 0.18, 0.94};

float S1p[3] = {-15.26, 8.8, -6.4};
float S1xs[3] = {-0.5, -0.87, 0};
float S1ys[3] = {-0.29, 0.18, 0.94};

float res = 0.00508;


// Triplets Definitions

// TRIPLET 1
  float T1p_i[3] = {S0p[0], S0p[1], S0p[2]};
  float T1s_i[3] = {S0xs[0], S0xs[1], S0xs[2]};
  float T1p_j[3] = {S0p[0], S0p[1], S0p[2]};
  float T1s_j[3] = {S0ys[0], S0ys[1], S0ys[2]};
  float T1p_k[3] = {S1p[0], S1p[1], S1p[2]};
  float T1s_k[3] = {S1xs[0], S1xs[1], S1xs[2]};

  float T1cross_i[3];
  float T1cross_j[3];
  float T1cross_k[3];

  float T1A[3][3];
  float T1Ainv[3][3];
  float T1Adet;

  float T1w[3];
  float T1w2[3];
  
// Triplet 2

float T2p_i[3] = {S0p[0], S0p[1], S0p[2]};      
float T2s_i[3] = {S0xs[0], S0xs[1], S0xs[2]};   
float T2p_j[3] = {S0p[0], S0p[1], S0p[2]};      
float T2s_j[3] = {S0ys[0], S0ys[1], S0ys[2]};   
float T2p_k[3] = {S1p[0], S1p[1], S1p[2]};      
float T2s_k[3] = {S1ys[0], S1ys[1], S1ys[2]};

 float T2cross_i[3];
 float T2cross_j[3];
 float T2cross_k[3];

 float T2A[3][3];
 float T2Ainv[3][3];
 float T2Adet;

 float T2w[3];
 float T2w2[3];

// Triplet 3 
float T3p_i[3] = {S0p[0], S0p[1], S0p[2]};      
float T3s_i[3] = {S0ys[0], S0ys[1], S0ys[2]};   
float T3p_j[3] = {S1p[0], S1p[1], S1p[2]};      
float T3s_j[3] = {S1xs[0], S1xs[1], S1xs[2]};   
float T3p_k[3] = {S1p[0], S1p[1], S1p[2]};      
float T3s_k[3] = {S1ys[0], S1ys[1], S1ys[2]};   

 float T3cross_i[3];
 float T3cross_j[3];
 float T3cross_k[3];

 float T3A[3][3];
 float T3Ainv[3][3];
 float T3Adet;

 float T3w[3];
 float T3w2[3];


// Triplet 4

float T4p_i[3] = {S0p[0], S0p[1], S0p[2]};      
float T4s_i[3] = {S0xs[0], S0xs[1], S0xs[2]};   
float T4p_j[3] = {S1p[0], S1p[1], S1p[2]};      
float T4s_j[3] = {S1xs[0], S1xs[1], S1xs[2]};   
float T4p_k[3] = {S1p[0], S1p[1], S1p[2]};      
float T4s_k[3] = {S1ys[0], S1ys[1], S1ys[2]}; 


 float T4cross_i[3];
 float T4cross_j[3];
 float T4cross_k[3];

 float T4A[3][3];
 float T4Ainv[3][3];
 float T4Adet;

 float T4w[3];
 float T4w2[3];

float avg_w[3];


void setup() {

// For triplet 1

  crossProduct(T1p_i, T1s_i, T1cross_i);
  crossProduct(T1p_j, T1s_j, T1cross_j);
  crossProduct(T1p_k, T1s_k, T1cross_k);

  T1A[0][0] = T1cross_i[0];
  T1A[0][1] = T1cross_i[1];
  T1A[0][2] = T1cross_i[2];

  T1A[1][0] = T1cross_j[0];
  T1A[1][1] = T1cross_j[1];
  T1A[1][2] = T1cross_j[2];

  T1A[2][0] = T1cross_k[0];
  T1A[2][1] = T1cross_k[1];
  T1A[2][2] = T1cross_k[2];
  
  T1Adet = calculate_Adet(T1A);
  
  calculate_Ainv(T1A, T1Ainv);

// For triplet 2
crossProduct(T2p_i, T2s_i, T2cross_i);
crossProduct(T2p_j, T2s_j, T2cross_j);
crossProduct(T2p_k, T2s_k, T2cross_k);

T2A[0][0] = T2cross_i[0];
T2A[0][1] = T2cross_i[1];
T2A[0][2] = T2cross_i[2];

T2A[1][0] = T2cross_j[0];
T2A[1][1] = T2cross_j[1];
T2A[1][2] = T2cross_j[2];

T2A[2][0] = T2cross_k[0];
T2A[2][1] = T2cross_k[1];
T2A[2][2] = T2cross_k[2];

T2Adet = calculate_Adet(T2A);
calculate_Ainv(T2A, T2Ainv);

// For triplet 3
crossProduct(T3p_i, T3s_i, T3cross_i);
crossProduct(T3p_j, T3s_j, T3cross_j);
crossProduct(T3p_k, T3s_k, T3cross_k);

T3A[0][0] = T3cross_i[0];
T3A[0][1] = T3cross_i[1];
T3A[0][2] = T3cross_i[2];

T3A[1][0] = T3cross_j[0];
T3A[1][1] = T3cross_j[1];
T3A[1][2] = T3cross_j[2];

T3A[2][0] = T3cross_k[0];
T3A[2][1] = T3cross_k[1];
T3A[2][2] = T3cross_k[2];

T3Adet = calculate_Adet(T3A);
calculate_Ainv(T3A, T3Ainv);

//For triplet 4
crossProduct(T4p_i, T4s_i, T4cross_i);
crossProduct(T4p_j, T4s_j, T4cross_j);
crossProduct(T4p_k, T4s_k, T4cross_k);

T4A[0][0] = T4cross_i[0];
T4A[0][1] = T4cross_i[1];
T4A[0][2] = T4cross_i[2];

T4A[1][0] = T4cross_j[0];
T4A[1][1] = T4cross_j[1];
T4A[1][2] = T4cross_j[2];

T4A[2][0] = T4cross_k[0];
T4A[2][1] = T4cross_k[1];
T4A[2][2] = T4cross_k[2];

T4Adet = calculate_Adet(T4A);
calculate_Ainv(T4A, T4Ainv);
  
  Serial.begin(115200);
  
  pinMode (ncs0, OUTPUT);
  pinMode (ncs1, OUTPUT);
  pinMode (rst0, OUTPUT);
  pinMode (rst1, OUTPUT);
  digitalWrite(ncs0, HIGH);
  digitalWrite(ncs1, HIGH);
  digitalWrite(rst0, HIGH);
  digitalWrite(rst1, HIGH);
  
  //pinMode(Motion_Interrupt_Pin0, INPUT);
  //digitalWrite(Motion_Interrupt_Pin0, HIGH);
  //attachInterrupt(9, UpdatePointer0, FALLING);

 // pinMode(Motion_Interrupt_Pin1, INPUT);
  //digitalWrite(Motion_Interrupt_Pin1, HIGH);
  //attachInterrupt(9, UpdatePointer1, FALLING);

  SPI.begin();
  SPI.setDataMode(SPI_MODE3);
  SPI.setBitOrder(MSBFIRST);
  SPI.setClockDivider(SPI_CLOCK_DIV128);
  //SPI.setClockDivider(4);

  Serial.println("SENSOR 0");

  performStartup(ncs0); 
    delay(1000);
    Serial.print("product ID (expected 71): ");
    Serial.println(adns_read_reg(Product_ID, ncs0));
 
  //dispRegisters(ncs0);

  delay(5);

  Serial.println();
  Serial.println();

  Serial.println("SENSOR 1");

  performStartup(ncs1);  
  
  delay(1000);
      Serial.print("product ID (expected 71): ");
      Serial.println(adns_read_reg(Product_ID, ncs1));

  //dispRegisters(ncs1);

  delay(5000);
    digitalWrite(ncs0, HIGH);
    digitalWrite(ncs1, HIGH);
    Serial.println("x,y,z");




  initComplete=9;

}

void adns_com_begin(int ncs){
  digitalWrite(ncs, LOW);
}

void adns_com_end(int ncs){
  digitalWrite(ncs, HIGH);
}

byte adns_read_reg(byte reg_addr, int ncs){
  adns_com_begin(ncs);
  
  // send adress of the register, with MSBit = 0 to indicate it's a read
  SPI.transfer(reg_addr & 0x7f );
  delayMicroseconds(100); // tSRAD
  // read data
  byte data = SPI.transfer(0);
  
  delayMicroseconds(1); // tSCLK-NCS for read operation is 120ns
  adns_com_end(ncs);
  delayMicroseconds(19); //  tSRW/tSRR (=20us) minus tSCLK-NCS

  return data;
}

void adns_write_reg(byte reg_addr, byte data, int ncs){
  adns_com_begin(ncs);
  
  //send adress of the register, with MSBit = 1 to indicate it's a write
  SPI.transfer(reg_addr | 0x80 );
  //sent data
  SPI.transfer(data);
  
  delayMicroseconds(20); // tSCLK-NCS for write operation
  adns_com_end(ncs);
  delayMicroseconds(100); // tSWW/tSWR (=120us) minus tSCLK-NCS. Could be shortened, but is looks like a safe lower bound 
}

void adns_upload_firmware(int ncs){
  // send the firmware to the chip, cf p.18 of the datasheet
  Serial.println("Uploading firmware...");

  //Write 0 to Rest_En bit of Config2 register to disable Rest mode.
  adns_write_reg(Config2, 0x20, ncs);
  
  // write 0x1d in SROM_enable reg for initializing
  adns_write_reg(SROM_Enable, 0x1d, ncs); 
  
  // wait for more than one frame period
  delay(10); // assume that the frame rate is as low as 100fps... even if it should never be that low
  
  // write 0x18 to SROM_enable to start SROM download
  adns_write_reg(SROM_Enable, 0x18, ncs); 
  
  // write the SROM file (=firmware data) 
  adns_com_begin(ncs);
  SPI.transfer(SROM_Load_Burst | 0x80); // write burst destination adress
  delayMicroseconds(15);
  
  // send all bytes of the firmware
  unsigned char c;
  for(int i = 0; i < firmware_length; i++){ 
    c = (unsigned char)pgm_read_byte(firmware_data + i);
    SPI.transfer(c);
    delayMicroseconds(15);
  }

  //Read the SROM_ID register to verify the ID before any other register reads or writes.
  adns_read_reg(SROM_ID, ncs);

  //Write 0x00 to Config2 register for wired mouse or 0x20 for wireless mouse design.
  adns_write_reg(Config2, 0x00, ncs);

  // set initial CPI resolution
  //adns_write_reg(Res_H, 0x15, ncs);
  //adns_write_reg(Res_L, 0x15, ncs);
  
  adns_com_end(ncs);
  }


void performStartup(int ncs){
  adns_com_end(ncs); // ensure that the serial port is reset
  adns_com_begin(ncs); // ensure that the serial port is reset
  adns_com_end(ncs); // ensure that the serial port is reset
  adns_write_reg(Power_Up_Reset, 0x5a, ncs); // force reset
  delay(50); // wait for it to reboot
  // read registers 0x02 to 0x06 (and discard the data)
  adns_read_reg(Motion, ncs);
  adns_read_reg(Delta_X_L, ncs);
  adns_read_reg(Delta_X_H, ncs);
  adns_read_reg(Delta_Y_L, ncs);
  adns_read_reg(Delta_Y_H, ncs);
  // upload the firmware
  adns_upload_firmware(ncs);
  delay(10);
  }

void UpdatePointer0(void){
  if(initComplete==9){

    //write 0x01 to Motion register and read from it to freeze the motion values and make them available
    adns_write_reg(Motion, 0x01, ncs0);
    adns_read_reg(Motion, ncs0);

    xdat0[0] = (int)adns_read_reg(Delta_X_L, ncs0);
    xdat0[1] = (int)adns_read_reg(Delta_X_H, ncs0);

    ydat0[0] = (int)adns_read_reg(Delta_Y_L, ncs0);
    ydat0[1] = (int)adns_read_reg(Delta_Y_H, ncs0);

    movementflag0=1;
    }
  }

  void UpdatePointer1(void){
  if(initComplete==9){

    //write 0x01 to Motion register and read from it to freeze the motion values and make them available
    adns_write_reg(Motion, 0x01, ncs1);
    adns_read_reg(Motion, ncs1);

    xdat1[0] = (int)adns_read_reg(Delta_X_L, ncs1);
    xdat1[1] = (int)adns_read_reg(Delta_X_H, ncs1);

    ydat1[0] = (int)adns_read_reg(Delta_Y_L, ncs1);
    ydat1[1] = (int)adns_read_reg(Delta_Y_H, ncs1);

    movementflag1=1;
    }
  }

  void readBurst(int ncs, int *dx, int *dy) {
  adns_write_reg(Motion_Burst, 0x01, ncs);
  adns_com_begin(ncs);
  SPI.transfer(Motion_Burst);
  delayMicroseconds(35);

  byte burstData[6];
  for(int i = 0; i < 6; i++) {
    burstData[i] = SPI.transfer(0);
  }

  adns_com_end(ncs);

  // Motion, DX_L, DX_H, DY_L, DY_H, SQUAL
  *dx = (burstData[3] << 8) | burstData[2];
  *dy = (burstData[5] << 8) | burstData[4];
}


void dispRegisters(int ncs){
  int oreg[7] = {
    0x00,0x3F,0x2A,0x02  };
  char* oregname[] = {
    "Product_ID","Inverse_Product_ID","SROM_Version","Motion"  };
  byte regres;

  digitalWrite(ncs,LOW);

  int rctr=0;
  for(rctr=0; rctr<4; rctr++){
    SPI.transfer(oreg[rctr]);
    delay(1);
    Serial.println("---");
    Serial.println(oregname[rctr]);
    Serial.println(oreg[rctr],HEX);
    regres = SPI.transfer(0);
    Serial.println(regres,BIN);  
    delay(1);
  }
  digitalWrite(ncs,HIGH);
}



// to calculate the determinant of a 3x3 matrix


void loop() {
    currTime = millis();
      if(currTime > timer){
    //Serial.println(testctr++);
    timer = currTime + 2000;
    }

  if(currTime >= pollTimer){
//T1
    int x0, y0, x1, y1;
    readBurst(ncs0, &x0, &y0);
    readBurst(ncs1, &x1, &y1);
    //UpdatePointer0();
    //UpdatePointer1();

      //int x0 = xdat0[1] << 8 | xdat0[0];
      //int y0 = ydat0[1] << 8 | ydat0[0];
      //int x1 = xdat1[1] << 8 | xdat1[0];
      //int y1 = ydat1[1] << 8 | ydat1[0];

      float x0mm = x0 * res;
      float y0mm = y0 * res;
      float x1mm = x1 * res;
      float y1mm = y1 * res;

          // Serial.print(currTime);
          // Serial.print(",");
          // Serial.print(x0);
          // Serial.print(",");
          // Serial.print(y0);
          // Serial.print(",");
          // Serial.print(x1);
          // Serial.print(",");
          // Serial.println(y1);

      calculate_w_fast(x0mm, y0mm, x1mm, T1Ainv, T1w);
      calculate_w_fast(x0mm, y0mm, y1mm, T2Ainv, T2w);
      calculate_w_fast(y0mm, x1mm, y1mm, T3Ainv, T3w);
      calculate_w_fast(x0mm, x1mm, y1mm, T4Ainv, T4w);

      // Calculate w2 for each triplet
      calculate_w2_fast(x0mm, y0mm, x1mm, T1Ainv, T1w2);
      calculate_w2_fast(x0mm, y0mm, y1mm, T2Ainv, T2w2);
      calculate_w2_fast(y0mm, x1mm, y1mm, T3Ainv, T3w2);
      calculate_w2_fast(x0mm, x1mm, y1mm, T4Ainv, T4w2);

      float ws[4][3];
      float ws2[4][3];
      float absAdets[4];
      
      for (int i = 0; i < 3; i++){
        ws[0][i] = T1w[i];
        ws[1][i] = T2w[i];
        ws[2][i] = T3w[i];
        ws[3][i] = T4w[i];

        ws2[0][i] = T1w2[i];
        ws2[1][i] = T2w2[i];
        ws2[2][i] = T3w2[i];
        ws2[3][i] = T4w2[i];
      }

      absAdets[0] = fabs(T1Adet);
      absAdets[1] = fabs(T2Adet);
      absAdets[2] = fabs(T3Adet);
      absAdets[3] = fabs(T4Adet);

      calculate_avg_w_fast(ws, absAdets, ws2, 4, avg_w);

           Serial.print('s');
           Serial.print(avg_w[0],5);
           Serial.print(",");
           Serial.print(avg_w[1],5);
           Serial.print(",");
           Serial.print(avg_w[2],5);
           Serial.println('e');
    pollTimer = currTime + 1;
    }
    
  }
