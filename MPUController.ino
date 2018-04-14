#include "quaternionFilters.h"
#include "MPU9250.h"
MPU9250 myIMU;

#ifdef SOFT_IIC
#include "SoftWire.h"
SoftWire Wire;
#endif

#include "HID-Project.h"

//#define DEBUG_FLAG

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

float fix(float angle) {
    while (angle < 0) angle += 360;
    while (angle > 360) angle -= 360;
    return angle;
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

        myIMU.yaw = fix(myIMU.yaw);
        myIMU.pitch = fix(myIMU.pitch);
        myIMU.roll = fix(myIMU.roll);
        
    #ifdef DEBUG_FLAG
        Serial.print("Yaw, Pitch, Roll:\t");
        Serial.print(myIMU.yaw, 2);
        Serial.print(",\t");
        Serial.print(myIMU.pitch, 2);
        Serial.print(",\t");
        Serial.print(myIMU.roll, 2);

        Serial.print(" || rate=\t");
        Serial.print((float)myIMU.sumCount / myIMU.sum, 2);
        Serial.println(" Hz");
    #endif

        myIMU.count = millis();
    }

    work();
}

int t = 0; bool ctrl = false;
void work() {
    if (ctrl) {
        Gamepad.xAxis(map(fix(myIMU.pitch + 180), 0, 360, -32767, 32767));
        Gamepad.yAxis(map(fix(myIMU.roll + 180), 0, 360, -32767, 32767));
        //Gamepad.zAxis(map(fix(myIMU.yaw + 180), 0, 360, -127, 127));
        /*
        Gamepad.rxAxis(map(fix(myIMU.yaw + 180), 0, 360, -32767, 32767));
        Gamepad.ryAxis(map(fix(myIMU.pitch + 180), 0, 360, -32767, 32767));
        Gamepad.rzAxis(map(fix(myIMU.roll + 180), 0, 360, -127, 127));
        */
        Gamepad.write();
    } else {
        if (t == 0) t = millis();
        else {
            if (millis() - t > 3000) {
                myIMU.yaw = 0;
                myIMU.pitch = 0;
                myIMU.roll = 0;
                ctrl = true;
            }
        }
    }
}

