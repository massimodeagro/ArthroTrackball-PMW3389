#include <SPI.h>
#include <avr/pgmspace.h>

// PMW3389 SPI Settings: Supports up to 2MHz clock speeds
SPISettings adnsSettings(2000000, MSBFIRST, SPI_MODE3);

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

#define PACKET_SCALE        100000   // radians * 100000 -

// Pin Maps
const int ncs0 = 10;
const int ncs1 = 8;
const int rst0 = 5;
const int rst1 = 6;

byte initComplete = 0;
unsigned long pollTimer = 0;
unsigned long lastMillis = 0;
const unsigned long LOOP_PERIOD_MICROS = 2000; 
unsigned long lastLoopMicros = 0;

extern const unsigned short firmware_length;
extern const unsigned char firmware_data[];

// Sensors Geometric Configurations
const float res = 0.00508; // 5000 CPI Resolution multi-scale factor (25.4 / 5000)

float S0p[3]  = {15.26, 8.8, -6.4};
float S0xs[3] = {-0.5, 0.87, 0};
float S0ys[3] = {0.29, 0.18, 0.94};

float S1p[3]  = {-15.26, 8.8, -6.4};
float S1xs[3] = {-0.5, -0.87, 0};
float S1ys[3] = {-0.29, 0.18, 0.94};

// Map structural layout for clean indexing 
struct AxisConfig {
    float* pos;
    float* dir;
};

AxisConfig sensorAxes[4] = {
    {S0p, S0xs}, // Index 0: Sensor 0 X
    {S0p, S0ys}, // Index 1: Sensor 0 Y
    {S1p, S1xs}, // Index 2: Sensor 1 X
    {S1p, S1ys}  // Index 3: Sensor 1 Y
};

// Map the specific index groups used by the 4 triplets
const uint8_t tripletAxesMapping[4][3] = {
    {0, 1, 2}, // Triplet 1
    {0, 1, 3}, // Triplet 2
    {1, 2, 3}, // Triplet 3
    {0, 2, 3}  // Triplet 4
};

struct TripletData {
    float Ainv[3][3];
    float absAdet;
    float w[3];
    float w2[3];
} triplets[4];

float avg_w[3];

// Math Helpers
void crossProduct(const float a[3], const float b[3], float result[3]) {
    result[0] = a[1] * b[2] - a[2] * b[1];
    result[1] = a[2] * b[0] - a[0] * b[2];
    result[2] = a[0] * b[1] - a[1] * b[0];
}

float calculate_Adet(const float A[3][3]) {
    return A[0][0] * (A[1][1] * A[2][2] - A[2][1] * A[1][2]) -
           A[0][1] * (A[1][0] * A[2][2] - A[2][0] * A[1][2]) +
           A[0][2] * (A[1][0] * A[2][1] - A[2][0] * A[1][1]);
}

bool calculate_Ainv(const float A[3][3], float Ainv[3][3], float det) {
    if (det == 0) return false;
    Ainv[0][0] = (A[1][1] * A[2][2] - A[2][1] * A[1][2]) / det;
    Ainv[0][1] = (A[0][2] * A[2][1] - A[0][1] * A[2][2]) / det;
    Ainv[0][2] = (A[0][1] * A[1][2] - A[0][2] * A[1][1]) / det;

    Ainv[1][0] = (A[1][2] * A[2][0] - A[1][0] * A[2][2]) / det;
    Ainv[1][1] = (A[0][0] * A[2][2] - A[0][2] * A[2][0]) / det;
    Ainv[1][2] = (A[1][0] * A[0][2] - A[0][0] * A[1][2]) / det;

    Ainv[2][0] = (A[1][0] * A[2][1] - A[2][0] * A[1][1]) / det;
    Ainv[2][1] = (A[2][0] * A[0][1] - A[0][0] * A[2][1]) / det;
    Ainv[2][2] = (A[0][0] * A[1][1] - A[1][0] * A[0][1]) / det;
    return true;
}

void computeTripletW(int t, const float readings[4]) {
    float vsi = readings[tripletAxesMapping[t][0]];
    float vsj = readings[tripletAxesMapping[t][1]];
    float vsk = readings[tripletAxesMapping[t][2]];

    for (int i = 0; i < 3; i++) {
        triplets[t].w[i] = triplets[t].Ainv[i][0] * vsi + triplets[t].Ainv[i][1] * vsj + triplets[t].Ainv[i][2] * vsk;
        float sum = fabs(triplets[t].Ainv[i][0] * vsi) + fabs(triplets[t].Ainv[i][1] * vsj) + fabs(triplets[t].Ainv[i][2] * vsk);
        triplets[t].w2[i] = (sum != 0.0f) ? fabs(triplets[t].w[i]) / sum : 0.0f;
    }
}

void calculate_combined_avg_w(float out_avg[3]) {
    float xn[3] = {0, 0, 0};
    float xd[3] = {0, 0, 0};

    for (int i = 0; i < 4; i++) {
        float det = triplets[i].absAdet;
        for (int axis = 0; axis < 3; axis++) {
            xn[axis] += det * triplets[i].w2[axis] * triplets[i].w[axis];
            xd[axis] += det * triplets[i].w2[axis];
        }
    }

    for (int axis = 0; axis < 3; axis++) {
        out_avg[axis] = (xd[axis] != 0) ? (xn[axis] / xd[axis]) : 0;
        if (isnan(out_avg[axis])) out_avg[axis] = 0;
    }
}

// Setup & Runtime Loop
void setup() {
    Serial.begin(115200);

    pinMode(ncs0, OUTPUT);
    pinMode(ncs1, OUTPUT);
    pinMode(rst0, OUTPUT);
    pinMode(rst1, OUTPUT);
    digitalWrite(ncs0, HIGH);
    digitalWrite(ncs1, HIGH);
    digitalWrite(rst0, HIGH);
    digitalWrite(rst1, HIGH);

    SPI.begin();

    // Matrix Initialization Engine Loop
    for (int t = 0; t < 4; t++) {
        float A[3][3];
        for (int row = 0; row < 3; row++) {
            uint8_t axisIdx = tripletAxesMapping[t][row];
            float crossRes[3];
            crossProduct(sensorAxes[axisIdx].pos, sensorAxes[axisIdx].dir, crossRes);
            A[row][0] = crossRes[0];
            A[row][1] = crossRes[1];
            A[row][2] = crossRes[2];
        }
        float det = calculate_Adet(A);
        triplets[t].absAdet = fabs(det);
        calculate_Ainv(A, triplets[t].Ainv, det);
    }

    bool s0_ok = performStartup(ncs0);
    delay(1000); 
    bool s1_ok = performStartup(ncs1);
    delay(1000);
    
    digitalWrite(ncs0, HIGH);
    digitalWrite(ncs1, HIGH);

    if (s0_ok && s1_ok) {
        initComplete = 9;  
    } else if (s0_ok && !s1_ok) {
        initComplete = 1; 
    } else if (!s0_ok && s1_ok) {
        initComplete = 2;  
    } else {
        initComplete = 0; 
    }

    pollTimer = millis();
}

void loop() {
    while (micros() - lastLoopMicros < LOOP_PERIOD_MICROS) {
        // Non-blocking wait trap to lock frequency to exactly 300Hz
    }
    lastLoopMicros = micros(); // Capture the exact start time of this execution cycle

    unsigned long currentMillis = millis();
    
    // Calculate the integer delta since the last iteration
    uint16_t deltaMillis = (uint16_t)(currentMillis - lastMillis);
    lastMillis = currentMillis;

    int x0, y0, x1, y1;
    byte sq0, sq1;
    byte mot0, mot1;
    uint16_t shutter0, shutter1;

    // SPI burst reads
    readBurst(ncs0, &x0, &y0, &sq0, &mot0, &shutter0);
    readBurst(ncs1, &x1, &y1, &sq1, &mot1, &shutter1);

    float readings[4] = {
        x0 * res,
        y0 * res,
        x1 * res,
        y1 * res
    };

    for (int t = 0; t < 4; t++) {
        computeTripletW(t, readings);
    }

    calculate_combined_avg_w(avg_w);
    
    sendBinaryPacket(avg_w[0], avg_w[1], avg_w[2], x0, y0, x1, y1, sq0, sq1, mot0, shutter0, mot1, shutter1, deltaMillis);
}

// SPI Transactions Framework
void adns_com_begin(int ncs) {
    digitalWrite(ncs, LOW);
    SPI.beginTransaction(adnsSettings);
}

void adns_com_end(int ncs) {
    delayMicroseconds(1);
    digitalWrite(ncs, HIGH);
    SPI.endTransaction();
}

byte adns_read_reg(byte reg_addr, int ncs) {
    adns_com_begin(ncs);
    SPI.transfer(reg_addr & 0x7f);
    delayMicroseconds(35); // PMW3389 tSRAD delay min parameter
    byte data = SPI.transfer(0);
    delayMicroseconds(1);
    adns_com_end(ncs);
    delayMicroseconds(19);
    return data;
}

void adns_write_reg(byte reg_addr, byte data, int ncs) {
    adns_com_begin(ncs);
    SPI.transfer(reg_addr | 0x80);
    SPI.transfer(data);
    delayMicroseconds(20);
    adns_com_end(ncs);
    delayMicroseconds(100);
}

void adns_upload_firmware(int ncs) {
    adns_write_reg(Config2, 0x20, ncs);
    adns_write_reg(SROM_Enable, 0x1d, ncs);
    delay(10);
    adns_write_reg(SROM_Enable, 0x18, ncs);

    adns_com_begin(ncs);
    SPI.transfer(SROM_Load_Burst | 0x80);
    delayMicroseconds(15);

    for (int i = 0; i < firmware_length; i++) {
        byte c = pgm_read_byte(firmware_data + i);
        SPI.transfer(c);
        delayMicroseconds(15);
    }
    adns_com_end(ncs);
    delayMicroseconds(10);

    adns_read_reg(SROM_ID, ncs);
    adns_write_reg(Config2, 0x00, ncs);

    // Explicitly lock resolution configuration to 5000 CPI (5000 / 50 = 100 = 0x64)
    adns_write_reg(Res_H, 0x00, ncs);
    adns_write_reg(Res_L, 0x64, ncs);
}

bool performStartup(int ncs) {
    adns_com_end(ncs);
    adns_com_begin(ncs);
    adns_com_end(ncs);
    adns_write_reg(Power_Up_Reset, 0x5a, ncs);
    delay(50);
    adns_read_reg(Motion, ncs);
    adns_read_reg(Delta_X_L, ncs);
    adns_read_reg(Delta_X_H, ncs);
    adns_read_reg(Delta_Y_L, ncs);
    adns_read_reg(Delta_Y_H, ncs);
    adns_upload_firmware(ncs);
    delay(10);

    byte out = adns_read_reg(Product_ID, ncs); 
    delay(1000);
    if (out == 0x47) {
        return true; 
    }
    return false; 
}

void readBurst(int ncs, int *dx, int *dy, byte *squal, byte *motionStatus, uint16_t *shutterSpeed) {
    adns_write_reg(Motion_Burst, 0x01, ncs);
    adns_com_begin(ncs);
    SPI.transfer(Motion_Burst);
    delayMicroseconds(35); // PMW3389 tSRAD delay parameter

    byte buf[12]; // Expanded from 7 to 12 to capture the full frame
    for (int i = 0; i < 12; i++) {
        buf[i] = SPI.transfer(0);
    }
    adns_com_end(ncs);

    // 1. Extract Motion status flag (Byte 0)
    *motionStatus = buf[0];

    // 2. Extract Delta X and Y (Bytes 2-5)
    *dx = (int)(int16_t)((buf[3] << 8) | buf[2]);
    *dy = (int)(int16_t)((buf[5] << 8) | buf[4]);

    // 3. Extract Surface Quality (Byte 6)
    *squal = buf[6];

    // 4. Extract Real-time Shutter Speed (Bytes 10 & 11)
    *shutterSpeed = (uint16_t)((buf[10] << 8) | buf[11]);
}

void sendSerialPacket(float wx, float wy, float wz, int x0, int y0, int x1, int y1, 
                      byte sq0, byte sq1, byte mot0, uint16_t shutter0, byte mot1, uint16_t shutter1, uint16_t deltaMillis) {
    
    int16_t iwx = (int16_t)constrain((long)(wx * PACKET_SCALE), -32768, 32767);
    int16_t iwy = (int16_t)constrain((long)(wy * PACKET_SCALE), -32768, 32767);
    int16_t iwz = (int16_t)constrain((long)(wz * PACKET_SCALE), -32768, 32767);
    int16_t ix0 = (int16_t)constrain((long)x0, -32768, 32767);
    int16_t iy0 = (int16_t)constrain((long)y0, -32768, 32767);
    int16_t ix1 = (int16_t)constrain((long)x1, -32768, 32767);
    int16_t iy1 = (int16_t)constrain((long)y1, -32768, 32767);

    // Inizio della riga leggibile
    Serial.print("START_PKT -> Init:");
    Serial.print(initComplete);

    // Cinematica e coordinate grezze
    Serial.print(", Wx:");   Serial.print(iwx);
    Serial.print(", Wy:");   Serial.print(iwy);
    Serial.print(", Wz:");   Serial.print(iwz);
    Serial.print(", X0:");   Serial.print(ix0);
    Serial.print(", Y0:");   Serial.print(iy0);
    Serial.print(", X1:");   Serial.print(ix1);
    Serial.print(", Y1:");   Serial.print(iy1);
    
    // Qualità della superficie (Squal)
    Serial.print(", SQ0:");  Serial.print(sq0);
    Serial.print(", SQ1:");  Serial.print(sq1);

    // Delta tempo
    Serial.print(", DT:");   Serial.print(deltaMillis);

    // Diagnostica hardware (Motion e Shutter)
    Serial.print(", MOT0:"); Serial.print(mot0);
    Serial.print(", SH0:");  Serial.print(shutter0);
    Serial.print(", MOT1:"); Serial.print(mot1);
    Serial.print(", SH1:");  Serial.print(shutter1);

    // Fine della riga con ritorno a capo
    Serial.println(" <- END_PKT");
}

// Explicitly define packet size constants
#define RAW_PAYLOAD_SIZE 25
#define TRANSMIT_BUFFER_SIZE (RAW_PAYLOAD_SIZE + 2) // +1 for Checksum, +1 for COBS Overhead

void sendBinaryPacket(float wx, float wy, float wz, int x0, int y0, int x1, int y1, 
                      byte sq0, byte sq1, byte mot0, uint16_t shutter0, byte mot1, uint16_t shutter1, uint16_t deltaMillis) {
    
    // 1. Constrain and cast safely to unsigned equivalents for bitwise operations
    uint16_t iwx = (int16_t)constrain((long)(wx * PACKET_SCALE), -32768, 32767);
    uint16_t iwy = (int16_t)constrain((long)(wy * PACKET_SCALE), -32768, 32767);
    uint16_t iwz = (int16_t)constrain((long)(wz * PACKET_SCALE), -32768, 32767);
    uint16_t ix0 = (int16_t)constrain((long)x0, -32768, 32767);
    uint16_t iy0 = (int16_t)constrain((long)y0, -32768, 32767);
    uint16_t ix1 = (int16_t)constrain((long)x1, -32768, 32767);
    uint16_t iy1 = (int16_t)constrain((long)y1, -32768, 32767);

    // Buffer for raw data + checksum
    byte raw_buf[RAW_PAYLOAD_SIZE + 1]; 

    // Pack raw bytes explicitly using logical masking
    raw_buf[0]  = initComplete;
    raw_buf[1]  = (iwx >> 8) & 0xFF;  raw_buf[2]  = iwx & 0xFF;
    raw_buf[3]  = (iwy >> 8) & 0xFF;  raw_buf[4]  = iwy & 0xFF;
    raw_buf[5]  = (iwz >> 8) & 0xFF;  raw_buf[6]  = iwz & 0xFF;
    raw_buf[7]  = (ix0 >> 8) & 0xFF;  raw_buf[8]  = ix0 & 0xFF;
    raw_buf[9]  = (iy0 >> 8) & 0xFF;  raw_buf[10] = iy0 & 0xFF;
    raw_buf[11] = (ix1 >> 8) & 0xFF;  raw_buf[12] = ix1 & 0xFF;
    raw_buf[13] = (iy1 >> 8) & 0xFF;  raw_buf[14] = iy1 & 0xFF;
    raw_buf[15] = sq0;
    raw_buf[16] = sq1;

    raw_buf[17] = (deltaMillis >> 8) & 0xFF;
    raw_buf[18] = deltaMillis & 0xFF;

    raw_buf[19] = mot0;
    raw_buf[20] = (shutter0 >> 8) & 0xFF;    raw_buf[21] = shutter0 & 0xFF;
    raw_buf[22] = mot1;
    raw_buf[23] = (shutter1 >> 8) & 0xFF;    raw_buf[24] = shutter1 & 0xFF;

    // 2. Compute Checksum over the payload
    byte ck = 0;
    for (int i = 0; i < RAW_PAYLOAD_SIZE; i++) {
        ck ^= raw_buf[i];
    }
    raw_buf[RAW_PAYLOAD_SIZE] = ck; // Store checksum at the end of raw payload

    // 3. Encode to COBS to eliminate all 0x00 bytes from data stream
    byte tx_buf[TRANSMIT_BUFFER_SIZE];
    
    uint8_t code_index = 0;
    uint8_t code = 1;
    
    for (int i = 0; i < (RAW_PAYLOAD_SIZE + 1); i++) {
        if (raw_buf[i] == 0x00) {
            tx_buf[code_index] = code;
            code_index = i + 1;
            code = 1;
        } else {
            tx_buf[i + 1] = raw_buf[i];
            code++;
            if (code == 0xFF) {
                tx_buf[code_index] = code;
                code_index = i + 1;
                code = 1; // Safeguard for long packets (not strictly hit here)
            }
        }
    }
    tx_buf[code_index] = code;

    // 4. Stream frame out with a unique, un-collidable 0x00 delimiter
    Serial.write(tx_buf, TRANSMIT_BUFFER_SIZE);
    Serial.write((uint8_t)0x00);
    }
