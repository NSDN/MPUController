#include "quaternionFilters.h"
#include "MPU9250.h"
MPU9250 myIMU;

#ifdef SOFT_IIC
#include "SoftWire.h"
SoftWire Wire;
#endif

#include "HID-Project.h"

//#define DEBUG_FLAG

#define KEY_A 4
#define KEY_B 5
#define KEY_C 6
#define KEY_1 A0
#define KEY_2 A1
#define KEY_3 A2

bool first = true;
float data[3];
uint16_t count = 50;

void _writeByte(uint8_t address, uint8_t subAddress, uint8_t data) {
    Wire.beginTransmission(address);  // Initialize the Tx buffer
    Wire.write(subAddress);           // Put slave register address in Tx buffer
    Wire.write(data);                 // Put data in Tx buffer
    Wire.endTransmission();           // Send the Tx buffer
}

uint8_t _readByte(uint8_t address, uint8_t subAddress) {
    uint8_t data; // `data` will store the register data
    Wire.beginTransmission(address);         // Initialize the Tx buffer
    Wire.write(subAddress);                  // Put slave register address in Tx buffer
    Wire.endTransmission(false);             // Send the Tx buffer, but send a restart to keep connection alive
    Wire.requestFrom(address, (uint8_t)1);  // Read one byte from slave register address
    data = Wire.read();                      // Fill Rx buffer with result
    return data;                             // Return data read from slave register
}

void _readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * dest) {
    Wire.beginTransmission(address);   // Initialize the Tx buffer
    Wire.write(subAddress);            // Put slave register address in Tx buffer
    Wire.endTransmission(false);       // Send the Tx buffer, but send a restart to keep connection alive
    uint8_t i = 0;
    Wire.requestFrom(address, count);  // Read bytes from slave register address
    while (Wire.available()) {
        dest[i++] = Wire.read();
    }         // Put read results in the Rx buffer
}

void setup() {
    Wire.begin();
#ifdef DEBUG_FLAG
    Serial.begin(115200);
#endif
    myIMU.wB = &_writeByte;
    myIMU.rB = &_readByte;
    myIMU.rBs = &_readBytes;

    byte c = myIMU.readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);
    if (c == 0x71) {
        myIMU.MPU9250SelfTest(myIMU.SelfTest);
        myIMU.calibrateMPU9250(myIMU.gyroBias, myIMU.accelBias);
        myIMU.initMPU9250();
        myIMU.initAK8963(myIMU.magCalibration);
    }
    else {
        while (1);
    }

    pinMode(KEY_A, OUTPUT); digitalWrite(KEY_A, LOW);
    pinMode(KEY_B, OUTPUT); digitalWrite(KEY_B, LOW);
    pinMode(KEY_C, OUTPUT); digitalWrite(KEY_C, LOW);
    
    Gamepad.begin();
}

void loop() {
    if (myIMU.readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01) {
        myIMU.readAccelData(myIMU.accelCount);
        myIMU.getAres();
        myIMU.ax = (float)myIMU.accelCount[0] * myIMU.aRes;
        myIMU.ay = (float)myIMU.accelCount[1] * myIMU.aRes;
        myIMU.az = (float)myIMU.accelCount[2] * myIMU.aRes;

        myIMU.readGyroData(myIMU.gyroCount);
        myIMU.getGres();
        myIMU.gx = (float)myIMU.gyroCount[0] * myIMU.gRes;
        myIMU.gy = (float)myIMU.gyroCount[1] * myIMU.gRes;
        myIMU.gz = (float)myIMU.gyroCount[2] * myIMU.gRes;

        myIMU.readMagData(myIMU.magCount);
        myIMU.getMres();
        myIMU.magbias[0] = +470.;
        myIMU.magbias[1] = +120.;
        myIMU.magbias[2] = +125.;
        myIMU.mx = (float)myIMU.magCount[0] * myIMU.mRes * myIMU.magCalibration[0] -
            myIMU.magbias[0];
        myIMU.my = (float)myIMU.magCount[1] * myIMU.mRes * myIMU.magCalibration[1] -
            myIMU.magbias[1];
        myIMU.mz = (float)myIMU.magCount[2] * myIMU.mRes * myIMU.magCalibration[2] -
            myIMU.magbias[2];
    }

    myIMU.updateTime();

    MahonyQuaternionUpdate(myIMU.ax, myIMU.ay, myIMU.az, myIMU.gx * DEG_TO_RAD,
        myIMU.gy * DEG_TO_RAD, myIMU.gz * DEG_TO_RAD, myIMU.my,
        myIMU.mx, myIMU.mz, myIMU.deltat);

    myIMU.delt_t = millis() - myIMU.count;
    if (myIMU.delt_t > 10) {
        myIMU.yaw = atan2(2.0f * (*(getQ() + 1) * *(getQ() + 2) + *getQ() *
            *(getQ() + 3)), *getQ() * *getQ() + *(getQ() + 1) * *(getQ() + 1)
            - *(getQ() + 2) * *(getQ() + 2) - *(getQ() + 3) * *(getQ() + 3));
        myIMU.pitch = -asin(2.0f * (*(getQ() + 1) * *(getQ() + 3) - *getQ() *
            *(getQ() + 2)));
        myIMU.roll = atan2(2.0f * (*getQ() * *(getQ() + 1) + *(getQ() + 2) *
            *(getQ() + 3)), *getQ() * *getQ() - *(getQ() + 1) * *(getQ() + 1)
            - *(getQ() + 2) * *(getQ() + 2) + *(getQ() + 3) * *(getQ() + 3));
        myIMU.pitch *= RAD_TO_DEG;
        myIMU.yaw *= RAD_TO_DEG;

        myIMU.yaw -= 8.5; //ref: http://www.ngdc.noaa.gov/geomag-web/#declination
        myIMU.roll *= RAD_TO_DEG;

        if (count > 0) count -= 1;
        else if (first) {
            data[0] = myIMU.yaw;
            data[1] = myIMU.pitch;
            data[2] = myIMU.roll;
            first = false;
        }
        else {
            myIMU.yaw -= data[0];
            myIMU.pitch -= data[1];
            myIMU.roll -= data[2];
        }
        myIMU.count = millis();
    }

    work();
}

int t = 0; bool ctrl = false;
bool down = false, up = false;
bool left = false, right = false;
bool zlock = false;

float cal[] = { 0, 0, 0 };

bool mode = false; //false -> axis, true -> arrow

#define ANGLE_ARROW 45.0F
#define ANGLE_AXIS 75.0F
float HALF_ANGLE = ANGLE_AXIS;

inline float getYaw() { return myIMU.yaw - cal[0]; }
inline float getPitch() { return myIMU.pitch - cal[1]; }
inline float getRoll() { return myIMU.roll - cal[2]; }

float fix(float angle) {
    angle += 180.0F;
    while (angle < 0.0F) angle += 360.0F;
    while (angle > 360.0F) angle -= 360.0F;
    if (angle < (180.0F - HALF_ANGLE)) angle = (180.0F - HALF_ANGLE);
    if (angle > (180.0F + HALF_ANGLE)) angle = (180.0F + HALF_ANGLE);
    return angle;
}

int mapf(float in, int to) {
    in -= 180.0F;
    float result = (float) to / HALF_ANGLE * in;
    return result;
}

void work() {
    #ifdef DEBUG_FLAG
        Serial.print("Yaw, Pitch, Roll:\t");
        Serial.print(fix(getYaw() + 180), 2);
        Serial.print(",\t");
        Serial.print(fix(getPitch() + 180), 2);
        Serial.print(",\t");
        Serial.print(fix(getRoll() + 180), 2);

        Serial.print(" || rate=\t");
        Serial.print((float)myIMU.sumCount / myIMU.sum, 2);
        Serial.println(" Hz");
    #endif
  
    up = down = left = right = zlock = false;
    
    digitalWrite(KEY_A, HIGH);
    zlock = analogRead(KEY_1) > 400;
    left = analogRead(KEY_2) > 400;
    digitalWrite(KEY_A, LOW);
    
    digitalWrite(KEY_B, HIGH);
    right = analogRead(KEY_1) > 400;
    down = analogRead(KEY_2) > 400;
    up = analogRead(KEY_3) > 400;
    digitalWrite(KEY_B, LOW);   
    
    if (ctrl) {
        if (up && down) {
            cal[0] = myIMU.yaw;
            cal[1] = myIMU.pitch;
            cal[2] = myIMU.roll;
        }
        
        if (mode) {
            bool up = (fix(getPitch()) - 180.0F) < -(HALF_ANGLE / 2.0F);
            bool down = (fix(getPitch()) - 180.0F) > (HALF_ANGLE / 2.0F);
            bool left = (fix(getRoll()) - 180.0F) < -(HALF_ANGLE / 2.0F);
            bool right = (fix(getRoll()) - 180.0F) > (HALF_ANGLE / 2.0F);
            if (up) Gamepad.yAxis(-32767);
            else if (down) Gamepad.yAxis(32767);
            else Gamepad.yAxis(0);
            if (left) Gamepad.xAxis(-32767);
            else if (right) Gamepad.xAxis(32767);
            else Gamepad.xAxis(0);
        } else {
            Gamepad.xAxis(mapf(fix(getRoll()), 32767));
            Gamepad.yAxis(mapf(fix(getPitch()), 32767));
            Gamepad.zAxis(mapf(fix(getYaw()), -127));
        }

        if (up && zlock) {
            mode = true;
            HALF_ANGLE = ANGLE_ARROW;
        } else if (down && zlock) {
            mode = false;
            HALF_ANGLE = ANGLE_AXIS;
        } else if (zlock && !up && !down) {
            Gamepad.xAxis(0);
            Gamepad.yAxis(0);
        } else if (up && !down) {
            Gamepad.yAxis(0);
            Gamepad.zAxis(0);
        } else if (!up && down) {
            Gamepad.xAxis(0);
            Gamepad.zAxis(0);
        }

        if (left) Gamepad.press(1);
        else Gamepad.release(1);
        if (right) Gamepad.press(2);
        else Gamepad.release(2);
        
        Gamepad.write();
    } else {
        if (t == 0) t = millis();
        else {
            if (millis() - t > 1000) {
                cal[0] = myIMU.yaw;
                cal[1] = myIMU.pitch;
                cal[2] = myIMU.roll;
                ctrl = true;
            }
        }
    }
    
}

