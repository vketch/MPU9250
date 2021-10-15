#include "MPU9250.h"

using namespace std::chrono;
//-------------------------------------------------------------------------------------------------------------------------------
//            Private Methods
//-------------------------------------------------------------------------------------------------------------------------------

void MPU9250::writeByte(uint8_t address, uint8_t subAddress, uint8_t data)
{
    char data_write[2];
    data_write[0] = subAddress;
    data_write[1] = data;
    mpuI2C->write(address, data_write, 2, 0);
}
uint8_t MPU9250::readByte(uint8_t address, uint8_t subAddress)
{
    char data[1];
    char data_write[1];
    data_write[0] = subAddress;
    mpuI2C->write(address, data_write, 1, 1);
    mpuI2C->read(address, data, 1, 0);
    return data[0];
}
void MPU9250::readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t *dest)
{
    char data[14];
    char data_write[1];
    data_write[0] = subAddress;
    mpuI2C->write(address, data_write, 1, 1);
    mpuI2C->read(address, data, count, 0);
    for (uint8_t i = 0; i < count; i++)
        dest[i] = data[i];
}

//-------------------------------------------------------------------------------------------------------------------------------
// Public Methods
//-------------------------------------------------------------------------------------------------------------------------------

MPU9250::MPU9250(PinName sda, PinName scl)
{
    Ascale = AFS_2G;
    Gscale = GFS_250DPS;
    Mscale = MFS_16BITS;

    magBias[0] = magBias[1] = magBias[2] = 0;
    magCalibration[0] = magCalibration[1] = magCalibration[2] = 0;
    Mmode = 0x06;

    sum = sumCount = delt_t = count = 0;
    GyroMeasError = PI * (40.0f / 180.0f);
    beta = sqrt(3.0f / 4.0f) * GyroMeasError;
    GyroMeasDrift = PI * (2.0f / 180.0f);
    zeta = sqrt(3.0f / 4.0f) * GyroMeasDrift;
    deltat = 0.0f;
    Now = lastUpdate = firstUpdate = 0;

    q[0] = 1.0f;
    q[1] = q[2] = q[3] = 0.0f;
    eInt[0] = eInt[1]= eInt[2] = 0.0f;

    mpuI2C = new I2C(sda, scl);
    mpuI2C->frequency(400000);

    t.start();
    reset();
    calibrate();
    initMPU9250();
    initAK8963();
    getRes();
}

MPU9250::~MPU9250(void)
{
    if (mpuI2C != NULL)
        delete mpuI2C;
}

uint8_t MPU9250::whoami(void)
{
    return readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);
}

void MPU9250::getRes(void)
{
    switch (Ascale) {
            // Possible accelerometer scales (and their register bit settings) are:
            // 2 Gs (00), 4 Gs (01), 8 Gs (10), and 16 Gs  (11).
            // Here's a bit of an algorith to calculate DPS/(ADC tick) based on that 2-bit value:
        case AFS_2G:
            aRes = 2.0 / 32768.0;
            break;
        case AFS_4G:
            aRes = 4.0 / 32768.0;
            break;
        case AFS_8G:
            aRes = 8.0 / 32768.0;
            break;
        case AFS_16G:
            aRes = 16.0 / 32768.0;
            break;
    }
    switch (Gscale) {
            // Possible gyro scales (and their register bit settings) are:
            // 250 DPS (00), 500 DPS (01), 1000 DPS (10), and 2000 DPS  (11).
            // Here's a bit of an algorith to calculate DPS/(ADC tick) based on that 2-bit value:
        case GFS_250DPS:
            gRes = 250.0 / 32768.0;
            break;
        case GFS_500DPS:
            gRes = 500.0 / 32768.0;
            break;
        case GFS_1000DPS:
            gRes = 1000.0 / 32768.0;
            break;
        case GFS_2000DPS:
            gRes = 2000.0 / 32768.0;
            break;
    }
    switch (Mscale) {
            // Possible magnetometer scales (and their register bit settings) are:
            // 14 bit resolution (0) and 16 bit resolution (1)
        case MFS_14BITS:
            mRes = 10.0*4219.0/8190.0; // Proper scale to return milliGauss
            break;
        case MFS_16BITS:
            mRes = 10.0*4219.0/32760.0; // Proper scale to return milliGauss
            break;
    }
}

void MPU9250::reset(void)
{
    // reset device, reset all registers, clear gyro and accelerometer bias registers
    writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x80); // Write a one to bit 7 reset bit; toggle reset device
    ThisThread::sleep_for(100ms);
}

void MPU9250::initMPU9250(void)
{
    writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x00);
    ThisThread::sleep_for(10ms);
    writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x01);
    writeByte(MPU9250_ADDRESS, CONFIG, 0x03);
    writeByte(MPU9250_ADDRESS, SMPLRT_DIV, 0x04);

    uint8_t c = readByte(MPU9250_ADDRESS, GYRO_CONFIG);
    writeByte(MPU9250_ADDRESS, GYRO_CONFIG, c & ~0xE0);
    writeByte(MPU9250_ADDRESS, GYRO_CONFIG, c & ~0x18);
    writeByte(MPU9250_ADDRESS, GYRO_CONFIG, c | Gscale << 3);
    c = readByte(MPU9250_ADDRESS, ACCEL_CONFIG);
    writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, c & ~0xE0);
    writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, c & ~0x18);
    writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, c | Ascale << 3);
    c = readByte(MPU9250_ADDRESS, ACCEL_CONFIG2);
    writeByte(MPU9250_ADDRESS, ACCEL_CONFIG2, c & ~0x0F);
    writeByte(MPU9250_ADDRESS, ACCEL_CONFIG2, c & 0x03);

    writeByte(MPU9250_ADDRESS, INT_PIN_CFG, 0x22);
    writeByte(MPU9250_ADDRESS, INT_ENABLE, 0x01);
}
void MPU9250::initAK8963(void)
{
    // First extract the factory calibration for each magnetometer axis
    uint8_t rawData[3];  // x/y/z gyro calibration data stored here
    writeByte(AK8963_ADDRESS, AK8963_CNTL, 0x00); // Power down magnetometer
    ThisThread::sleep_for(10ms);
    writeByte(AK8963_ADDRESS, AK8963_CNTL, 0x0F); // Enter Fuse ROM access mode
    ThisThread::sleep_for(10ms);
    readBytes(AK8963_ADDRESS, AK8963_ASAX, 3, &rawData[0]);  // Read the x-, y-, and z-axis calibration values
    magCalibration[0] =  (float)(rawData[0] - 128)/256.0f + 1.0f;   // Return x-axis sensitivity adjustment values, etc.
    magCalibration[1] =  (float)(rawData[1] - 128)/256.0f + 1.0f;
    magCalibration[2] =  (float)(rawData[2] - 128)/256.0f + 1.0f;
    writeByte(AK8963_ADDRESS, AK8963_CNTL, 0x00); // Power down magnetometer
    ThisThread::sleep_for(10ms);
    // Configure the magnetometer for continuous read and highest resolution
    // set Mscale bit 4 to 1 (0) to enable 16 (14) bit resolution in CNTL register,
    // and enable continuous mode data acquisition Mmode (bits [3:0]), 0010 for 8 Hz and 0110 for 100 Hz sample rates
    writeByte(AK8963_ADDRESS, AK8963_CNTL, Mscale << 4 | Mmode); // Set magnetometer data resolution and sample ODR
    ThisThread::sleep_for(10ms);
}

void MPU9250::calibrate(void)
{
    uint8_t data[12];
    uint16_t packet_count, fifo_count;
    int32_t gyro_bias[3] = {0, 0, 0}, accel_bias[3] = {0, 0, 0};
    // get stable time source
    writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x01);
    writeByte(MPU9250_ADDRESS, PWR_MGMT_2, 0x00);
    ThisThread::sleep_for(200ms);

    // Configure device for bias calculation
    writeByte(MPU9250_ADDRESS, INT_ENABLE, 0x00);   // Disable all interrupts
    writeByte(MPU9250_ADDRESS, FIFO_EN, 0x00);      // Disable FIFO
    writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x00);   // Turn on internal clock source
    writeByte(MPU9250_ADDRESS, I2C_MST_CTRL, 0x00); // Disable I2C master
    writeByte(MPU9250_ADDRESS, USER_CTRL, 0x00);    // Disable FIFO and I2C master modes
    writeByte(MPU9250_ADDRESS, USER_CTRL, 0x0C);    // Reset FIFO and DMP
    ThisThread::sleep_for(15ms);
    // Configure MPU9250 gyro and accelerometer for bias calculation
    writeByte(MPU9250_ADDRESS, CONFIG, 0x01);       // Set low-pass filter to 188 Hz
    writeByte(MPU9250_ADDRESS, SMPLRT_DIV, 0x00);   // Set sample rate to 1 kHz
    writeByte(MPU9250_ADDRESS, GYRO_CONFIG, 0x00);  // Set gyro full-scale to 250 degrees per second, maximum sensitivity
    writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, 0x00); // Set accelerometer full-scale to 2 g, maximum sensitivity

    uint16_t gyroSens = 131;    // = 131 LSB/degrees/sec
    uint16_t accelSens = 16384; // = 16384 LSB/g

    // Configure FIFO to capture accelerometer and gyro data for bias calculation
    writeByte(MPU9250_ADDRESS, USER_CTRL, 0x40); // Enable FIFO
    writeByte(MPU9250_ADDRESS, FIFO_EN, 0x78);   // Enable gyro and accelerometer sensors for FIFO (max size 512 bytes in MPU-9250)
    ThisThread::sleep_for(40ms);                                 // accumulate 40 samples in 80 milliseconds = 480 bytes

    // At end of sample accumulation, turn off FIFO sensor read
    writeByte(MPU9250_ADDRESS, FIFO_EN, 0x00);            // Disable gyro and accelerometer sensors for FIFO
    readBytes(MPU9250_ADDRESS, FIFO_COUNTH, 2, &data[0]); // read FIFO sample count
    fifo_count = ((uint16_t)data[0] << 8) | data[1];
    packet_count = fifo_count / 12; // How many sets of full gyro and accelerometer data for averaging

    for (uint16_t i = 0; i < packet_count; i++) {
        int16_t accel_temp[3] = {0, 0, 0}, gyro_temp[3] = {0, 0, 0};
        readBytes(MPU9250_ADDRESS, FIFO_R_W, 12, &data[0]);           // read data for averaging
        accel_temp[0] = (int16_t)(((int16_t)data[0] << 8) | data[1]); // Form signed 16-bit integer for each sample in FIFO
        accel_temp[1] = (int16_t)(((int16_t)data[2] << 8) | data[3]);
        accel_temp[2] = (int16_t)(((int16_t)data[4] << 8) | data[5]);
        gyro_temp[0] = (int16_t)(((int16_t)data[6] << 8) | data[7]);
        gyro_temp[1] = (int16_t)(((int16_t)data[8] << 8) | data[9]);
        gyro_temp[2] = (int16_t)(((int16_t)data[10] << 8) | data[11]);

        accel_bias[0] += (int32_t)accel_temp[0]; // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
        accel_bias[1] += (int32_t)accel_temp[1];
        accel_bias[2] += (int32_t)accel_temp[2];
        gyro_bias[0] += (int32_t)gyro_temp[0];
        gyro_bias[1] += (int32_t)gyro_temp[1];
        gyro_bias[2] += (int32_t)gyro_temp[2];
    }
    accel_bias[0] /= (int32_t)packet_count; // Normalize sums to get average count biases
    accel_bias[1] /= (int32_t)packet_count;
    accel_bias[2] /= (int32_t)packet_count;
    gyro_bias[0] /= (int32_t)packet_count;
    gyro_bias[1] /= (int32_t)packet_count;
    gyro_bias[2] /= (int32_t)packet_count;

    if (accel_bias[2] > 0L)
        accel_bias[2] -= (int32_t)accelSens; // Remove gravity from the z-axis accelerometer bias calculation
    else
        accel_bias[2] += (int32_t)accelSens;

    // Construct the gyro biases for push to the hardware gyro bias registers, which are reset to zero upon device startup
    data[0] = (-gyro_bias[0] / 4 >> 8) & 0xFF; // Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input format
    data[1] = (-gyro_bias[0] / 4) & 0xFF;      // Biases are additive, so change sign on calculated average gyro biases
    data[2] = (-gyro_bias[1] / 4 >> 8) & 0xFF;
    data[3] = (-gyro_bias[1] / 4) & 0xFF;
    data[4] = (-gyro_bias[2] / 4 >> 8) & 0xFF;
    data[5] = (-gyro_bias[2] / 4) & 0xFF;

    gyroBias[0] = (float)gyro_bias[0] / (float)gyroSens; // construct gyro bias in deg/s for later manual subtraction
    gyroBias[1] = (float)gyro_bias[1] / (float)gyroSens;
    gyroBias[2] = (float)gyro_bias[2] / (float)gyroSens;

    int32_t accel_bias_reg[3] = {0, 0, 0};                // A place to hold the factory accelerometer trim biases
    readBytes(MPU9250_ADDRESS, XA_OFFSET_H, 2, &data[0]); // Read factory accelerometer trim values
    accel_bias_reg[0] = (int16_t)((int16_t)data[0] << 8) | data[1];
    readBytes(MPU9250_ADDRESS, YA_OFFSET_H, 2, &data[0]);
    accel_bias_reg[1] = (int16_t)((int16_t)data[0] << 8) | data[1];
    readBytes(MPU9250_ADDRESS, ZA_OFFSET_H, 2, &data[0]);
    accel_bias_reg[2] = (int16_t)((int16_t)data[0] << 8) | data[1];

    uint32_t mask = 1uL;             // Define mask for temperature compensation bit 0 of lower byte of accelerometer bias registers
    uint8_t mask_bit[3] = {0, 0, 0}; // Define array to hold mask bit for each accelerometer bias axis

    for (uint8_t i = 0; i < 3; i++) {
        if (accel_bias_reg[i] & mask)
            mask_bit[i] = 0x01; // If temperature compensation bit is set, record that fact in mask_bit
    }

    // Construct total accelerometer bias, including calculated average accelerometer bias from above
    accel_bias_reg[0] -= (accel_bias[0] / 8); // Subtract calculated averaged accelerometer bias scaled to 2048 LSB/g (16 g full scale)
    accel_bias_reg[1] -= (accel_bias[1] / 8);
    accel_bias_reg[2] -= (accel_bias[2] / 8);

    data[0] = (accel_bias_reg[0] >> 8) & 0xFF;
    data[1] = (accel_bias_reg[0]) & 0xFF;
    data[1] = data[1] | mask_bit[0]; // preserve temperature compensation bit when writing back to accelerometer bias registers
    data[2] = (accel_bias_reg[1] >> 8) & 0xFF;
    data[3] = (accel_bias_reg[1]) & 0xFF;
    data[3] = data[3] | mask_bit[1]; // preserve temperature compensation bit when writing back to accelerometer bias registers
    data[4] = (accel_bias_reg[2] >> 8) & 0xFF;
    data[5] = (accel_bias_reg[2]) & 0xFF;
    data[5] = data[5] | mask_bit[2]; // preserve temperature compensation bit when writing back to accelerometer bias registers

    // Output scaled accelerometer biases for manual subtraction in the main program
    accelBias[0] = (float)accel_bias[0] / (float)accelSens;
    accelBias[1] = (float)accel_bias[1] / (float)accelSens;
    accelBias[2] = (float)accel_bias[2] / (float)accelSens;
}

void MPU9250::test(float *destTest)
{
    uint8_t selfTest[6];
    int16_t gyroData[3], accelData[3];
    int16_t gAvg[3], gSTAvg[3], aAvg[3], aSTAvg[3];
    float factoryTrim[6];
    uint8_t FS = 0;

    writeByte(MPU9250_ADDRESS, SMPLRT_DIV, 0x00);      // Set gyro sample rate to 1 kHz
    writeByte(MPU9250_ADDRESS, CONFIG, 0x02);          // Set gyro sample rate to 1 kHz and DLPF to 92 Hz
    writeByte(MPU9250_ADDRESS, GYRO_CONFIG, 1 << FS);  // Set full scale range for the gyro to 250 dps
    writeByte(MPU9250_ADDRESS, ACCEL_CONFIG2, 0x02);   // Set accelerometer rate to 1 kHz and bandwidth to 92 Hz
    writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, 1 << FS); // Set full scale range for the accelerometer to 2 g

    for (uint8_t i = 0; i < 200; i++) {
        readGyroData(&gyroData[0]);
        readAccelData(&accelData[0]);
        for (uint8_t j = 0; j < 3; j++) {
            gAvg[j] += gyroData[j];
            aAvg[j] += accelData[j];
        }
    }
    for (uint8_t i = 0; i < 3; i++) {
        gAvg[i] /= 200;
        aAvg[i] /= 200;
    }

    // Configure the accelerometer for self-test
    writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, 0xE0); // Enable self test on all three axes and set accelerometer range to +/- 2 g
    writeByte(MPU9250_ADDRESS, GYRO_CONFIG, 0xE0);  // Enable self test on all three axes and set gyro range to +/- 250 degrees/s
    ThisThread::sleep_for(25ms);                                    // Delay a while to let the device stabilize

    for (uint8_t i = 0; i < 200; i++) {
        readGyroData(&gyroData[0]);
        readAccelData(&accelData[0]);
        for (uint8_t j = 0; j < 3; j++) {
            gSTAvg[j] += gyroData[j];
            aSTAvg[j] += accelData[j];
        }
    }
    for (uint8_t i = 0; i < 3; i++) {
        gSTAvg[i] /= 200;
        aSTAvg[i] /= 200;
    }

    // Configure the gyro and accelerometer for normal operation
    writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, 0x00);
    writeByte(MPU9250_ADDRESS, GYRO_CONFIG, 0x00);
    ThisThread::sleep_for(25ms);                                                // Delay a while to let the device stabilize
    // Retrieve accelerometer and gyro factory Self-Test Code from USR_Reg
    selfTest[0] = readByte(MPU9250_ADDRESS, SELF_TEST_X_ACCEL); // X-axis accel self-test results
    selfTest[1] = readByte(MPU9250_ADDRESS, SELF_TEST_Y_ACCEL); // Y-axis accel self-test results
    selfTest[2] = readByte(MPU9250_ADDRESS, SELF_TEST_Z_ACCEL); // Z-axis accel self-test results
    selfTest[3] = readByte(MPU9250_ADDRESS, SELF_TEST_X_GYRO);  // X-axis gyro self-test results
    selfTest[4] = readByte(MPU9250_ADDRESS, SELF_TEST_Y_GYRO);  // Y-axis gyro self-test results
    selfTest[5] = readByte(MPU9250_ADDRESS, SELF_TEST_Z_GYRO);  // Z-axis gyro self-test results

    // Retrieve factory self-test value from self-test code reads
    factoryTrim[0] = (float)(2620 / 1 << FS) * (pow(1.01, ((float)selfTest[0] - 1.0))); // FT[Xa] factory trim calculation
    factoryTrim[1] = (float)(2620 / 1 << FS) * (pow(1.01, ((float)selfTest[1] - 1.0))); // FT[Ya] factory trim calculation
    factoryTrim[2] = (float)(2620 / 1 << FS) * (pow(1.01, ((float)selfTest[2] - 1.0))); // FT[Za] factory trim calculation
    factoryTrim[3] = (float)(2620 / 1 << FS) * (pow(1.01, ((float)selfTest[3] - 1.0))); // FT[Xg] factory trim calculation
    factoryTrim[4] = (float)(2620 / 1 << FS) * (pow(1.01, ((float)selfTest[4] - 1.0))); // FT[Yg] factory trim calculation
    factoryTrim[5] = (float)(2620 / 1 << FS) * (pow(1.01, ((float)selfTest[5] - 1.0))); // FT[Zg] factory trim calculation

    // Report results as a ratio of (STR - FT)/FT; the change from Factory Trim of the Self-Test Response
    // To get percent, must multiply by 100
    for (uint8_t i = 0; i < 3; i++) {
        destTest[i] = 100.0 * ((float)(aSTAvg[i] - aAvg[i])) / factoryTrim[i];         // Report percent differences
        destTest[i + 3] = 100.0 * ((float)(gSTAvg[i] - gAvg[i])) / factoryTrim[i + 3]; // Report percent differences
    }
}

void MPU9250::readAccelData(int16_t *destAccel)
{
    uint8_t rawData[6];
    readBytes(MPU9250_ADDRESS, ACCEL_XOUT_H, 6, &rawData[0]);
    destAccel[0] = (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]);
    destAccel[1] = (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]);
    destAccel[2] = (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]);
}
void MPU9250::readGyroData(int16_t *destGyro)
{
    uint8_t rawData[6];
    readBytes(MPU9250_ADDRESS, GYRO_XOUT_H, 6, &rawData[0]);
    destGyro[0] = (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]);
    destGyro[1] = (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]);
    destGyro[2] = (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]);
}
void MPU9250::readMagData(int16_t *destMag)
{
    uint8_t rawData[7];  // x/y/z gyro register data, ST2 register stored here, must read ST2 at end of data acquisition
    if(readByte(AK8963_ADDRESS, AK8963_ST1) & 0x01) { // wait for magnetometer data ready bit to be set
        readBytes(AK8963_ADDRESS, AK8963_XOUT_L, 7, &rawData[0]);  // Read the six raw data and ST2 registers sequentially into data array
        uint8_t c = rawData[6]; // End data read by reading ST2 register
        if(!(c & 0x08)) { // Check if magnetic sensor overflow set, if not then report data
            destMag[0] = (int16_t)(((int16_t)rawData[1] << 8) | rawData[0]);  // Turn the MSB and LSB into a signed 16-bit value
            destMag[1] = (int16_t)(((int16_t)rawData[3] << 8) | rawData[2]) ;  // Data stored as little Endian
            destMag[2] = (int16_t)(((int16_t)rawData[5] << 8) | rawData[4]) ;
        }
    }
}


void MPU9250::updateABCData(void)
{
    if (readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01) {
        readAccelData(accelCount);
        accelData[X] = (float)accelCount[0] * aRes - accelBias[0];
        accelData[Y] = (float)accelCount[1] * aRes - accelBias[1];
        accelData[Z] = (float)accelCount[2] * aRes - accelBias[2];

        //accelData[X] = (float)accelCount[0] * aRes * 9.81;
        //accelData[Y] = (float)accelCount[1] * aRes * 9.81;
        //accelData[Z] = (float)accelCount[2] * aRes * 9.81;

        //readGyroData(gyroCount);
        //gyroData[X] = (float)gyroCount[0] * gRes - gyroBias[0];
        //gyroData[Y] = (float)gyroCount[1] * gRes - gyroBias[1];
        //gyroData[Z] = (float)gyroCount[2] * gRes - gyroBias[2];

        //readMagData(magCount);  // Read the x/y/z adc values
        //magData[X] = (float)magCount[0]*mRes*magCalibration[0] - magBias[0];  // get actual magnetometer value, this depends on scale being set
        //magData[Y] = (float)magCount[1]*mRes*magCalibration[1] - magBias[1];
        //magData[Z] = (float)magCount[2]*mRes*magCalibration[2] - magBias[2];

        readMagData(magCount);  // Read the x/y/z adc values
        magData[X] = (float)magCount[0]*mRes*magCalibration[0] - magBias[0];  // get actual magnetometer value, this depends on scale being set
        magData[Y] = (float)magCount[1]*mRes*magCalibration[1] - magBias[1];
        magData[Z] = (float)magCount[2]*mRes*magCalibration[2] - magBias[2];

    }
}

void MPU9250::updateData(void)
{
    if (readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01) {
        readAccelData(accelCount);
        accelData[X] = (float)accelCount[0] * aRes - accelBias[0];
        accelData[Y] = (float)accelCount[1] * aRes - accelBias[1];
        accelData[Z] = (float)accelCount[2] * aRes - accelBias[2];

        readGyroData(gyroCount);
        gyroData[X] = (float)gyroCount[0] * gRes - gyroBias[0];
        gyroData[Y] = (float)gyroCount[1] * gRes - gyroBias[1];
        gyroData[Z] = (float)gyroCount[2] * gRes - gyroBias[2];

        readMagData(magCount);  // Read the x/y/z adc values
        magData[X] = (float)magCount[0]*mRes*magCalibration[0] - magBias[0];  // get actual magnetometer value, this depends on scale being set
        magData[Y] = (float)magCount[1]*mRes*magCalibration[1] - magBias[1];
        magData[Z] = (float)magCount[2]*mRes*magCalibration[2] - magBias[2];
    }

    Now = duration_cast<microseconds>(t.elapsed_time()).count();
    deltat = (float)((Now - lastUpdate)/1000000.0f) ;
    lastUpdate = Now;

    sum += deltat;
    sumCount++;
    //MadgwickQuaternionUpdate(accelData[X], accelData[Y], accelData[Z], gyroData[X]*PI/180.0f, gyroData[Y]*PI/180.0f, gyroData[Z]*PI/180.0f);
    MahonyQuaternionUpdate(accelData[X], accelData[Y], accelData[Z], gyroData[X]*PI/180.0f, gyroData[Y]*PI/180.0f, gyroData[Z]*PI/180.0f, magData[X], magData[Y], magData[Z]);
    delt_t = duration_cast<milliseconds>(t.elapsed_time()).count() - count;
    if (delt_t > 40) {

        yaw   = atan2(2.0f * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]);
        pitch = -asin(2.0f * (q[1] * q[3] - q[0] * q[2]));
        roll  = atan2(2.0f * (q[0] * q[1] + q[2] * q[3]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]);
        pitch *= 180.0f / PI;
        yaw   *= 180.0f / PI;
        roll  *= 180.0f / PI;

        count = duration_cast<milliseconds>(t.elapsed_time()).count();
        if(count > 1<<21) {
            t.start(); // start the timer over again if ~30 minutes has passed
            count = 0;
            deltat= 0;
            lastUpdate = duration_cast<microseconds>(t.elapsed_time()).count();
        }
        sum = 0;
        sumCount = 0;
    }
}
void MPU9250::MadgwickQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz)
{
    float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3];         // short name local variable for readability
    float norm;                                               // vector norm
    float f1, f2, f3;                                         // objetive funcyion elements
    float J_11or24, J_12or23, J_13or22, J_14or21, J_32, J_33; // objective function Jacobian elements
    float qDot1, qDot2, qDot3, qDot4;
    float hatDot1, hatDot2, hatDot3, hatDot4;
    float gerrx, gerry, gerrz, gbiasx, gbiasy, gbiasz;        // gyro bias error

    // Auxiliary variables to avoid repeated arithmetic
    float _halfq1 = 0.5f * q1;
    float _halfq2 = 0.5f * q2;
    float _halfq3 = 0.5f * q3;
    float _halfq4 = 0.5f * q4;
    float _2q1 = 2.0f * q1;
    float _2q2 = 2.0f * q2;
    float _2q3 = 2.0f * q3;
    float _2q4 = 2.0f * q4;
    float _2q1q3 = 2.0f * q1 * q3;
    float _2q3q4 = 2.0f * q3 * q4;

    // Normalise accelerometer measurement
    norm = sqrt(ax * ax + ay * ay + az * az);
    if (norm == 0.0f) return; // handle NaN
    norm = 1.0f/norm;
    ax *= norm;
    ay *= norm;
    az *= norm;

    // Compute the objective function and Jacobian
    f1 = _2q2 * q4 - _2q1 * q3 - ax;
    f2 = _2q1 * q2 + _2q3 * q4 - ay;
    f3 = 1.0f - _2q2 * q2 - _2q3 * q3 - az;
    J_11or24 = _2q3;
    J_12or23 = _2q4;
    J_13or22 = _2q1;
    J_14or21 = _2q2;
    J_32 = 2.0f * J_14or21;
    J_33 = 2.0f * J_11or24;

    // Compute the gradient (matrix multiplication)
    hatDot1 = J_14or21 * f2 - J_11or24 * f1;
    hatDot2 = J_12or23 * f1 + J_13or22 * f2 - J_32 * f3;
    hatDot3 = J_12or23 * f2 - J_33 *f3 - J_13or22 * f1;
    hatDot4 = J_14or21 * f1 + J_11or24 * f2;

    // Normalize the gradient
    norm = sqrt(hatDot1 * hatDot1 + hatDot2 * hatDot2 + hatDot3 * hatDot3 + hatDot4 * hatDot4);
    hatDot1 /= norm;
    hatDot2 /= norm;
    hatDot3 /= norm;
    hatDot4 /= norm;

    // Compute estimated gyroscope biases
    gerrx = _2q1 * hatDot2 - _2q2 * hatDot1 - _2q3 * hatDot4 + _2q4 * hatDot3;
    gerry = _2q1 * hatDot3 + _2q2 * hatDot4 - _2q3 * hatDot1 - _2q4 * hatDot2;
    gerrz = _2q1 * hatDot4 - _2q2 * hatDot3 + _2q3 * hatDot2 - _2q4 * hatDot1;

    // Compute and remove gyroscope biases
    gbiasx += gerrx * deltat * zeta;
    gbiasy += gerry * deltat * zeta;
    gbiasz += gerrz * deltat * zeta;
    gx -= gbiasx;
    gy -= gbiasy;
    gz -= gbiasz;

    // Compute the quaternion derivative
    qDot1 = -_halfq2 * gx - _halfq3 * gy - _halfq4 * gz;
    qDot2 =  _halfq1 * gx + _halfq3 * gz - _halfq4 * gy;
    qDot3 =  _halfq1 * gy - _halfq2 * gz + _halfq4 * gx;
    qDot4 =  _halfq1 * gz + _halfq2 * gy - _halfq3 * gx;

    // Compute then integrate estimated quaternion derivative
    q1 += (qDot1 -(beta * hatDot1)) * deltat;
    q2 += (qDot2 -(beta * hatDot2)) * deltat;
    q3 += (qDot3 -(beta * hatDot3)) * deltat;
    q4 += (qDot4 -(beta * hatDot4)) * deltat;

    // Normalize the quaternion
    norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);    // normalise quaternion
    norm = 1.0f/norm;
    q[0] = q1 * norm;
    q[1] = q2 * norm;
    q[2] = q3 * norm;
    q[3] = q4 * norm;
}
void MPU9250::MahonyQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz)
{
    float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3];   // short name local variable for readability
    float norm;
    float hx, hy, bx, bz;
    float vx, vy, vz, wx, wy, wz;
    float ex, ey, ez;
    float pa, pb, pc;

    // Auxiliary variables to avoid repeated arithmetic
    float q1q1 = q1 * q1;
    float q1q2 = q1 * q2;
    float q1q3 = q1 * q3;
    float q1q4 = q1 * q4;
    float q2q2 = q2 * q2;
    float q2q3 = q2 * q3;
    float q2q4 = q2 * q4;
    float q3q3 = q3 * q3;
    float q3q4 = q3 * q4;
    float q4q4 = q4 * q4;

    // Normalise accelerometer measurement
    norm = sqrt(ax * ax + ay * ay + az * az);
    if (norm == 0.0f) return; // handle NaN
    norm = 1.0f / norm;        // use reciprocal for division
    ax *= norm;
    ay *= norm;
    az *= norm;

    // Normalise magnetometer measurement
    norm = sqrt(mx * mx + my * my + mz * mz);
    if (norm == 0.0f) return; // handle NaN
    norm = 1.0f / norm;        // use reciprocal for division
    mx *= norm;
    my *= norm;
    mz *= norm;

    // Reference direction of Earth's magnetic field
    hx = 2.0f * mx * (0.5f - q3q3 - q4q4) + 2.0f * my * (q2q3 - q1q4) + 2.0f * mz * (q2q4 + q1q3);
    hy = 2.0f * mx * (q2q3 + q1q4) + 2.0f * my * (0.5f - q2q2 - q4q4) + 2.0f * mz * (q3q4 - q1q2);
    bx = sqrt((hx * hx) + (hy * hy));
    bz = 2.0f * mx * (q2q4 - q1q3) + 2.0f * my * (q3q4 + q1q2) + 2.0f * mz * (0.5f - q2q2 - q3q3);

    // Estimated direction of gravity and magnetic field
    vx = 2.0f * (q2q4 - q1q3);
    vy = 2.0f * (q1q2 + q3q4);
    vz = q1q1 - q2q2 - q3q3 + q4q4;
    wx = 2.0f * bx * (0.5f - q3q3 - q4q4) + 2.0f * bz * (q2q4 - q1q3);
    wy = 2.0f * bx * (q2q3 - q1q4) + 2.0f * bz * (q1q2 + q3q4);
    wz = 2.0f * bx * (q1q3 + q2q4) + 2.0f * bz * (0.5f - q2q2 - q3q3);

    // Error is cross product between estimated direction and measured direction of gravity
    ex = (ay * vz - az * vy) + (my * wz - mz * wy);
    ey = (az * vx - ax * vz) + (mz * wx - mx * wz);
    ez = (ax * vy - ay * vx) + (mx * wy - my * wx);
    if (Ki > 0.0f) {
        eInt[0] += ex;      // accumulate integral error
        eInt[1] += ey;
        eInt[2] += ez;
    } else {
        eInt[0] = 0.0f;     // prevent integral wind up
        eInt[1] = 0.0f;
        eInt[2] = 0.0f;
    }

    // Apply feedback terms
    gx = gx + Kp * ex + Ki * eInt[0];
    gy = gy + Kp * ey + Ki * eInt[1];
    gz = gz + Kp * ez + Ki * eInt[2];

    // Integrate rate of change of quaternion
    pa = q2;
    pb = q3;
    pc = q4;
    q1 = q1 + (-q2 * gx - q3 * gy - q4 * gz) * (0.5f * deltat);
    q2 = pa + (q1 * gx + pb * gz - pc * gy) * (0.5f * deltat);
    q3 = pb + (q1 * gy - pa * gz + pc * gx) * (0.5f * deltat);
    q4 = pc + (q1 * gz + pa * gy - pb * gx) * (0.5f * deltat);

    // Normalise quaternion
    norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);
    norm = 1.0f / norm;
    q[0] = q1 * norm;
    q[1] = q2 * norm;
    q[2] = q3 * norm;
    q[3] = q4 * norm;

}