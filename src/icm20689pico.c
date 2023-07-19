#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <memory.h>
#include "hardware/i2c.h"
#include "icm20689pico/icm20689pico.h"

// Check https://invensense.tdk.com/products/motion-tracking/6-axis/icm-20689/ for more infos.

// Original chip id
const uint8_t ID = 0x98; // 152

// Registers
const uint8_t PWR_MGMT_1 = 0x6B;
const uint8_t PWR_MGMT_2 = 0x6C;
const uint8_t WHO_AM_I = 0x75;
const uint8_t CONFIG = 0x1A;
const uint8_t GYRO_CONFIG = 0x1B;
const uint8_t ACC_CONFIG = 0x1C;
const uint8_t ACC_CONFIG2 = 0x1D;
const uint8_t SAMPLE_RATE_DIVIDER = 0x19;
const uint8_t GYRO_OUT = 0x43;
const uint8_t ACC_OUT = 0x3B;
const uint8_t TEMP_OUT = 0x41;

// Fixed values
const uint8_t AUTO_CLOCK_SELECT = 0x01; // 0b00000001
const uint8_t PWR_RESET = 0x80; // 0b10000000
const uint8_t ACC_GYRO_ON = 0x00; // 0b00000000

// Constants
const double G = 9.807f;
const double _d2r = M_PI/180.0;
const double _r2d = 180.0/M_PI;

const uint8_t _numSamples = 100;

const double _tempAmbient = 21.0f;
const double _tempSensitivity = 333.87f;

const double _defaultAccScaleFactor[3] = {1.0, 1.0, 1.0};

#ifndef ICM20689_SLEEP
#define ICM20689_SLEEP(ms) sleep_ms(ms);
#endif


int8_t write_register(uint8_t addr, uint8_t reg, uint8_t value)
{
    uint8_t buff[] = {reg, value};

    int write;
    write = i2c_write_blocking(ICM20689_I2C == 0 ? i2c0 : i2c1, addr, buff, 2, false);
     
    if(write == PICO_ERROR_GENERIC) {
        return PICO_ERROR_GENERIC;
    }
    return PICO_OK;
}

int8_t read_registers(uint8_t addr, uint8_t reg, uint8_t count, uint8_t* buffer)
{
    int read;
    i2c_write_blocking(ICM20689_I2C == 0 ? i2c0 : i2c1, addr, &reg, 1, true);
    read = i2c_read_blocking(ICM20689_I2C == 0 ? i2c0 : i2c1, addr, buffer, count, false);
    if(read < count) {
        return PICO_ERROR_GENERIC;
    }
    return PICO_OK;
}

uint8_t icm20689_init(icm20689_t* icm20689, uint addr, uint samples_num) 
{
    icm20689->addr = addr;

    for (size_t i = 0; i < 3; i++)
    {
        icm20689->accData[i] = 0.0;
        icm20689->accDataRaw[i] = 0;
        icm20689->accScaleFactor[i] = 1.0;
        icm20689->accMax[i] = 0.0;
        icm20689->accMin[i] = 0.0;

        icm20689->accOffsetsRaw[i] = 0;
        icm20689->accOffsets[i] = 0.0;

        icm20689->gyroDataRaw[i] = 0;
        icm20689->gyroData[i] = 0.0;
        icm20689->gyroOffsetsRaw[i] = 0;
        icm20689->gyroOffsets[i] = 0.0;  
    }
    
    for (size_t i = 0; i < 2; i++)
    {
        icm20689->orientation[i] = 0.0;
    }
    
    for (size_t i = 0; i < 15; i++)
    {
        icm20689->buffer[i] = 0;
    }

    ICM20689_SLEEP(100);

    if(write_register(icm20689->addr, PWR_MGMT_1, AUTO_CLOCK_SELECT) != 0) {
        return ICM20689_I2C_ERROR;
    }
    

    if(write_register(icm20689->addr, PWR_MGMT_1, PWR_RESET) != 0) {
        return ICM20689_I2C_ERROR;
    }
    
    ICM20689_SLEEP(10);
    
    if(write_register(icm20689->addr, PWR_MGMT_1, AUTO_CLOCK_SELECT) != 0) {
        return ICM20689_I2C_ERROR;
    }

    if(read_registers(icm20689->addr, WHO_AM_I, 1, &icm20689->id) != 0) {
        return ICM20689_I2C_ERROR;
    }

    if(icm20689->id != ID) {
        return ICM20689_INVALID_ID;
    }

    if(write_register(icm20689->addr, PWR_MGMT_2, ACC_GYRO_ON) != 0) {
        return ICM20689_I2C_ERROR;
    }

    // Configure accelerometer and gyroscope

    //      Set full scale

    uint8_t error = 0;

    if((error = icm20689_set_acc_fs(icm20689, ICM20689_ACC_FS_16G)) != 0) {
        return ICM20689_ACC_CONFIG_ERROR + error;
    }

    if((error = icm20689_set_gyro_fs(icm20689, ICM20689_GYRO_FS_2000DPS)) != 0) {
        return ICM20689_GYRO_CONFIG_ERROR + error;
    }

    //      Set bandwidth

    if((error = icm20689_set_acc_dlpf(icm20689, ICM20689_ACC_DLPF_218HZ)) != 0) {
        return ICM20689_ACC_CONFIG_ERROR + error;
    }

    if((error = icm20689_set_gyro_dlpf(icm20689, ICM20689_GYRO_DLPF_250HZ)) != 0) {
        return ICM20689_GYRO_CONFIG_ERROR + error;
    }

    // Set sample rate divider

    if((error = write_register(icm20689->addr, SAMPLE_RATE_DIVIDER, 0x00)) != 0) {
        return ICM20689_I2C_ERROR + error;
    }

    // Calibrate gyroscope
    if((error = icm20689_calibrate_gyro(icm20689, samples_num)) != 0) {
        return ICM20689_GYRO_CALIBRATION_ERROR + error;
    }

    // Calibrate accelerometer
    if((error = icm20689_calibrate_acc(icm20689, samples_num)) != 0) {
        return ICM20689_GYRO_CALIBRATION_ERROR + error;
    }

    return ICM20689_SUCCESS;
}

uint8_t icm20689_set_acc_fs(icm20689_t* icm20689, uint8_t fs) 
{

    if(write_register(icm20689->addr, ACC_CONFIG, fs) != 0) {
        return ICM20689_I2C_ERROR;
    }

    double fs_value = 0.0;

    switch (fs)
    {
    case ICM20689_ACC_FS_2G:
        fs_value = 2.0f;
        break;
    case ICM20689_ACC_FS_4G:
        fs_value = 4.0f;
        break;
    case ICM20689_ACC_FS_8G:
        fs_value = 8.0f;
        break;
    case ICM20689_ACC_FS_16G:
        fs_value = 16.0f;
        break;
    default:
        fs_value = 0.0;
        break;
    }

    icm20689->accFullScale = G * fs_value/32767.5f;
    icm20689->accScaleType = fs;

    return ICM20689_SUCCESS;
}

uint8_t icm20689_set_acc_dlpf(icm20689_t* icm20689, uint8_t dlpf) 
{
    if(write_register(icm20689->addr, ACC_CONFIG2, dlpf) != 0) {
        return ICM20689_I2C_ERROR;
    }

    icm20689->acc_dlpf_bandwidth = dlpf;

    return ICM20689_SUCCESS;
}

uint8_t icm20689_set_gyro_fs(icm20689_t* icm20689, uint8_t fs) 
{

    if(write_register(icm20689->addr, GYRO_CONFIG, fs) != 0) {
        return ICM20689_I2C_ERROR;
    }

    double fs_value;

    switch (fs)
    {
    case ICM20689_GYRO_FS_250DPS:
        fs_value = 250.0f;
        break;
    case ICM20689_GYRO_FS_500DPS:
        fs_value = 500.0f;
        break;
    case ICM20689_GYRO_FS_1000DPS:
        fs_value = 1000.0f;
        break;
    case ICM20689_GYRO_FS_2000DPS:
        fs_value = 2000.0f;
        break;
    default:
        fs_value = 0.0;
        break;
    }

    icm20689->gyroFullScale = fs_value/32767.5f * _d2r;
    icm20689->gyroScaleType = fs;

    return ICM20689_SUCCESS;
}

uint8_t icm20689_set_gyro_dlpf(icm20689_t* icm20689, uint8_t dlpf) 
{
    if(write_register(icm20689->addr, CONFIG, dlpf) != 0) {
        return ICM20689_I2C_ERROR;
    }

    icm20689->gyro_dlpf_bandwidth = dlpf;

    return ICM20689_SUCCESS;
}

uint8_t icm20689_calibrate_gyro(icm20689_t* icm20689, uint samples_num) 
{
    if(write_register(icm20689->addr, CONFIG, ICM20689_GYRO_FS_250DPS) != 0) {
        return ICM20689_I2C_ERROR;
    }

    if(write_register(icm20689->addr, CONFIG, ICM20689_GYRO_DLPF_20HZ) != 0) {
        return ICM20689_I2C_ERROR;
    }

    if(write_register(icm20689->addr, SAMPLE_RATE_DIVIDER, 19) != 0) {
        return ICM20689_I2C_ERROR;
    }

    // Calibration process

    icm20689->gyroOffsetsRaw[0] = 0.0;
    icm20689->gyroOffsetsRaw[1] = 0.0;
    icm20689->gyroOffsetsRaw[2] = 0.0;
    
    uint smpl_num = _numSamples;

    if(samples_num > 0)
        smpl_num = samples_num;

    for (size_t i = 0; i < smpl_num; i++)
    {
        icm20689_read_gyro(icm20689, NULL);
        icm20689->gyroOffsetsRaw[0] += (icm20689->gyroData[0] + icm20689->gyroOffsets[0]) / ((double)_numSamples); 
        icm20689->gyroOffsetsRaw[1] += (icm20689->gyroData[1] + icm20689->gyroOffsets[1]) / ((double)_numSamples); 
        icm20689->gyroOffsetsRaw[2] += (icm20689->gyroData[2] + icm20689->gyroOffsets[2]) / ((double)_numSamples);
        ICM20689_SLEEP(20); 
    }
    
    icm20689->gyroOffsets[0] = icm20689->gyroOffsetsRaw[0];
    icm20689->gyroOffsets[1] = icm20689->gyroOffsetsRaw[1];
    icm20689->gyroOffsets[2] = icm20689->gyroOffsetsRaw[2];
    
    if(icm20689_set_gyro_fs(icm20689, icm20689->gyroScaleType) != 0) {
        return ICM20689_GYRO_CONFIG_ERROR;
    }

    if(icm20689_set_gyro_dlpf(icm20689, icm20689->gyro_dlpf_bandwidth) != 0) {
        return ICM20689_GYRO_CONFIG_ERROR;
    }
    
    if(write_register(icm20689->addr, SAMPLE_RATE_DIVIDER, 0x00) != 0) {
        return ICM20689_I2C_ERROR;
    }

    return ICM20689_SUCCESS; 
}

uint8_t icm20689_calibrate_acc(icm20689_t* icm20689, uint samples_num) 
{
       if(write_register(icm20689->addr, ACC_CONFIG, ICM20689_ACC_FS_2G) != 0) {
        return ICM20689_I2C_ERROR;
    }

    if(write_register(icm20689->addr, ACC_CONFIG2, ICM20689_ACC_DLPF_21HZ) != 0) {
        return ICM20689_I2C_ERROR;
    }

    if(write_register(icm20689->addr, SAMPLE_RATE_DIVIDER, 19) != 0) {
        return ICM20689_I2C_ERROR;
    }

    // Calibration process

    icm20689->accOffsetsRaw[0] = 0.0;
    icm20689->accOffsetsRaw[1] = 0.0;
    icm20689->accOffsetsRaw[2] = 0.0;
    
    uint smpl_num = _numSamples;

    if(samples_num > 0)
        smpl_num = samples_num;

    for (size_t i = 0; i < smpl_num; i++)
    {
        icm20689_read_acc(icm20689, NULL);
        icm20689->accOffsetsRaw[0] += (icm20689->accData[0]/icm20689->accScaleFactor[0] + icm20689->accOffsets[0]) / ((double)_numSamples); 
        icm20689->accOffsetsRaw[1] += (icm20689->accData[1]/icm20689->accScaleFactor[1] + icm20689->accOffsets[1]) / ((double)_numSamples); 
        icm20689->accOffsetsRaw[2] += (icm20689->accData[2]/icm20689->accScaleFactor[2] + icm20689->accOffsets[2]) / ((double)_numSamples);
        ICM20689_SLEEP(20); 
    }
    
    if (icm20689->accOffsetsRaw[0] > 9.0f) {
        icm20689->accMax[0] = (double)icm20689->accOffsetsRaw[0];
    }
    if (icm20689->accOffsetsRaw[1] > 9.0f) {
        icm20689->accMax[1] = (double)icm20689->accOffsetsRaw[1];
    }
    if (icm20689->accOffsetsRaw[2] > 9.0f) {
        icm20689->accMax[2] = (double)icm20689->accOffsetsRaw[2];
    }
    if (icm20689->accOffsetsRaw[0] < -9.0f) {
        icm20689->accMin[0] = (double)icm20689->accOffsetsRaw[0];
    }
    if (icm20689->accOffsetsRaw[1] < -9.0f) {
        icm20689->accMin[1] = (double)icm20689->accOffsetsRaw[1];
    }
    if (icm20689->accOffsetsRaw[2] < -9.0f) {
        icm20689->accMin[2] = (double)icm20689->accOffsetsRaw[2];
    }

    if ((fabs(icm20689->accMin[0]) > 9.0f) && (fabs(icm20689->accMax[0]) > 9.0f)) {
        icm20689->accOffsets[0] = (icm20689->accMin[0] + icm20689->accMax[0]) / 2.0f;
        icm20689->accScaleFactor[0] = G/((fabs(icm20689->accMin[0]) + fabs(icm20689->accMax[0])) / 2.0f);
    }
    if ((abs(icm20689->accMin[1]) > 9.0f) && (abs(icm20689->accMax[1]) > 9.0f)) {
        icm20689->accOffsets[1] = (icm20689->accMin[1] + icm20689->accMax[1]) / 2.0f;
        icm20689->accScaleFactor[1] = G/((fabs(icm20689->accMin[1]) + fabs(icm20689->accMax[1])) / 2.0f);
    }
    if ((fabs(icm20689->accMin[2]) > 9.0f) && (fabs(icm20689->accMax[2]) > 9.0f)) {
        icm20689->accOffsets[2] = (icm20689->accMin[2] + icm20689->accMax[2]) / 2.0f;
        icm20689->accScaleFactor[2] = G/((fabs(icm20689->accMin[2]) + fabs(icm20689->accMax[2])) / 2.0f);
    }
    
    if(icm20689_set_acc_fs(icm20689, icm20689->accScaleType) != 0) {
        return ICM20689_ACC_CONFIG_ERROR;
    }

    if(icm20689_set_acc_dlpf(icm20689, icm20689->acc_dlpf_bandwidth) != 0) {
        return ICM20689_ACC_CONFIG_ERROR;
    }
    
    if(write_register(icm20689->addr, SAMPLE_RATE_DIVIDER, 0x00) != 0) {
        return ICM20689_I2C_ERROR;
    }

    return ICM20689_SUCCESS; 
}

uint8_t icm20689_read_acc(icm20689_t* icm20689, double* acc)
{
    if(read_registers(icm20689->addr, ACC_OUT, 6, icm20689->buffer) != 0) {
        return ICM20689_I2C_ERROR;
    }

    // Convert data into 16 bit data
    icm20689->accDataRaw[0] = (((int16_t)icm20689->buffer[0]) << 8) | icm20689->buffer[1];
    icm20689->accDataRaw[1] = (((int16_t)icm20689->buffer[2]) << 8) | icm20689->buffer[3];
    icm20689->accDataRaw[2] = (((int16_t)icm20689->buffer[4]) << 8) | icm20689->buffer[5];

    icm20689->accData[0] =  (((double)(icm20689->accDataRaw[1]) * icm20689->accFullScale) - icm20689->accOffsets[0]) * icm20689->accScaleFactor[0];
    icm20689->accData[1] =  (((double)(icm20689->accDataRaw[0]) * icm20689->accFullScale) - icm20689->accOffsets[1]) * icm20689->accScaleFactor[1];
    icm20689->accData[2] =  (((double)(-icm20689->accDataRaw[2]) * icm20689->accFullScale) - icm20689->accOffsets[2]) * icm20689->accScaleFactor[2];

    if(acc != NULL) {
        memcpy(acc, icm20689->accData, 3 * sizeof(double));
    }

    return ICM20689_SUCCESS;
}

uint8_t icm20689_read_gyro(icm20689_t* icm20689, double* gyro)
{
    if(read_registers(icm20689->addr, GYRO_OUT, 6, icm20689->buffer) != 0) {
        return ICM20689_I2C_ERROR;
    }

    // Convert data into 16 bit data
    icm20689->gyroDataRaw[0] = (((int16_t)icm20689->buffer[0]) << 8) | icm20689->buffer[1];
    icm20689->gyroDataRaw[1] = (((int16_t)icm20689->buffer[2]) << 8) | icm20689->buffer[3];
    icm20689->gyroDataRaw[2] = (((int16_t)icm20689->buffer[4]) << 8) | icm20689->buffer[5];

    icm20689->gyroData[0] =  ((double)(icm20689->gyroDataRaw[1]) * icm20689->gyroFullScale) - icm20689->gyroOffsets[0];
    icm20689->gyroData[1] =  ((double)(icm20689->gyroDataRaw[0]) * icm20689->gyroFullScale) - icm20689->gyroOffsets[1];
    icm20689->gyroData[2] =  ((double)(-icm20689->gyroDataRaw[2]) * icm20689->gyroFullScale) - icm20689->gyroOffsets[2];


    if(gyro != NULL) {
        memcpy(gyro, icm20689->gyroData, 3 * sizeof(double));
    }

    return ICM20689_SUCCESS;
}

uint8_t icm20689_read_gyroacc(icm20689_t* icm20689, double* acc, double* gyro)
{
    if(read_registers(icm20689->addr, ACC_OUT, 15, icm20689->buffer) != 0) {
        return ICM20689_I2C_ERROR;
    }

    // Convert data into 16 bit data
    icm20689->accDataRaw[0] = (((int16_t)icm20689->buffer[0]) << 8) | icm20689->buffer[1];
    icm20689->accDataRaw[1] = (((int16_t)icm20689->buffer[2]) << 8) | icm20689->buffer[3];
    icm20689->accDataRaw[2] = (((int16_t)icm20689->buffer[4]) << 8) | icm20689->buffer[5];

    icm20689->gyroDataRaw[0] = (((int16_t)icm20689->buffer[8]) << 8) | icm20689->buffer[9];
    icm20689->gyroDataRaw[1] = (((int16_t)icm20689->buffer[10]) << 8) | icm20689->buffer[11];
    icm20689->gyroDataRaw[2] = (((int16_t)icm20689->buffer[12]) << 8) | icm20689->buffer[13];

    icm20689->accData[0] =  (((double)(icm20689->accDataRaw[1]) * icm20689->accFullScale) - icm20689->accOffsets[0]) * icm20689->accScaleFactor[0];
    icm20689->accData[1] =  (((double)(icm20689->accDataRaw[0]) * icm20689->accFullScale) - icm20689->accOffsets[1]) * icm20689->accScaleFactor[1];
    icm20689->accData[2] =  (((double)(-icm20689->accDataRaw[2]) * icm20689->accFullScale) - icm20689->accOffsets[2]) * icm20689->accScaleFactor[2];

    icm20689->gyroData[0] =  ((double)(icm20689->gyroDataRaw[1]) * icm20689->gyroFullScale) - icm20689->gyroOffsets[0];
    icm20689->gyroData[1] =  ((double)(icm20689->gyroDataRaw[0]) * icm20689->gyroFullScale) - icm20689->gyroOffsets[1];
    icm20689->gyroData[2] =  ((double)(-icm20689->gyroDataRaw[2]) * icm20689->gyroFullScale) - icm20689->gyroOffsets[2];

    if(acc != NULL) {
        memcpy(acc, icm20689->accData, 3 * sizeof(double));
    }

    if(gyro != NULL) {
        memcpy(gyro, icm20689->gyroData, 3 * sizeof(double));
    }

    return ICM20689_SUCCESS;
}

uint8_t icm20689_read_temp(icm20689_t* icm20689, double* temp)
{
    if(read_registers(icm20689->addr, TEMP_OUT, 2, icm20689->buffer) != 0) {
        return ICM20689_I2C_ERROR;
    }

    icm20689->tempDataRaw = (((int16_t)icm20689->buffer[0]) << 8) | icm20689->buffer[1];
    icm20689->tempData = ((((double) icm20689->tempDataRaw) - _tempAmbient)/_tempSensitivity) + _tempAmbient;
    
    *temp = icm20689->tempData;

    return ICM20689_SUCCESS;
}

uint8_t icm20689_power(icm20689_t* icm20689, bool power) 
{
    return ICM20689_SUCCESS;
}