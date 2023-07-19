#ifndef ICM20689PICO_H_
#define ICM20689PICO_H_

#ifdef __cplusplus
extern "C" {
#endif

#ifndef ICM20689_I2C
#define ICM20689_I2C 0
#endif

#define ICM20689_SUCCESS 0
#define ICM20689_I2C_ERROR 1
#define ICM20689_INVALID_ID 2
#define ICM20689_ACC_CONFIG_ERROR 3
#define ICM20689_GYRO_CONFIG_ERROR 4
#define ICM20689_GYRO_CALIBRATION_ERROR 5

// Accelerometer full scale
enum {
    ICM20689_ACC_FS_2G = 0x00,
    ICM20689_ACC_FS_4G = 0x08,
    ICM20689_ACC_FS_8G = 0x10,
    ICM20689_ACC_FS_16G = 0x18
};

// Accelerometer DLPF values
enum {
    ICM20689_ACC_DLPF_218HZ = 0x01,
    ICM20689_ACC_DLPF_99HZ = 0x02,
    ICM20689_ACC_DLPF_45HZ = 0x03,
    ICM20689_ACC_DLPF_21HZ = 0x04,
    ICM20689_ACC_DLPF_10HZ = 0x05,
    ICM20689_ACC_DLPF_5HZ = 0x06
};

// Gyroscope full scale
enum {
    ICM20689_GYRO_FS_250DPS = 0x00,
    ICM20689_GYRO_FS_500DPS = 0x08,
    ICM20689_GYRO_FS_1000DPS = 0x10,
    ICM20689_GYRO_FS_2000DPS = 0x18
};

// Gyroscope DLPF values
enum {
    ICM20689_GYRO_DLPF_250HZ = 0x00,
    ICM20689_GYRO_DLPF_176HZ = 0x01,
    ICM20689_GYRO_DLPF_92HZ = 0x02,
    ICM20689_GYRO_DLPF_41HZ = 0x03,
    ICM20689_GYRO_DLPF_20HZ = 0x04,
    ICM20689_GYRO_DLPF_10HZ = 0x05,
    ICM20689_GYRO_DLPF_5HZ = 0x06
};

typedef struct _icm20689
{
    uint8_t addr;
    uint8_t id;

    uint8_t buffer[15];

    // Acceleration data

    int16_t accDataRaw[3];
    double accData[3];

    double accScaleFactor[3];
    double accFullScale;
    int8_t accScaleType;
    int8_t acc_dlpf_bandwidth;

    double accMax[3];
    double accMin[3];

    double accOffsetsRaw[3];
    double accOffsets[3];

    double orientation[2];

    // Gyroscope data

    int16_t gyroDataRaw[3];
    double gyroData[3];

    double gyroFullScale;
    int8_t gyroScaleType;
    int8_t gyro_dlpf_bandwidth;

    double gyroOffsetsRaw[3];
    double gyroOffsets[3];

    // Temperature data

    int16_t tempDataRaw;
    double tempData;
} icm20689_t;

uint8_t icm20689_init(icm20689_t* icm20689, uint addr, uint samples_num);

uint8_t icm20689_set_acc_fs(icm20689_t* icm20689, uint8_t fs);
uint8_t icm20689_set_acc_dlpf(icm20689_t* icm20689, uint8_t dlpf);
uint8_t icm20689_set_gyro_fs(icm20689_t* icm20689, uint8_t fs);
uint8_t icm20689_set_gyro_dlpf(icm20689_t* icm20689, uint8_t dlpf);

uint8_t icm20689_calibrate_gyro(icm20689_t* icm20689, uint samples_num);
uint8_t icm20689_calibrate_acc(icm20689_t* icm20689, uint samples_num);

uint8_t icm20689_read_acc(icm20689_t* icm20689, double* acc);
uint8_t icm20689_read_gyro(icm20689_t* icm20689, double* gyro);
uint8_t icm20689_read_temp(icm20689_t* icm20689, double* temp);
uint8_t icm20689_read_gyroacc(icm20689_t* icm20689, double* acc, double* gyro);
uint8_t icm20689_power(icm20689_t* icm20689, bool power);

#ifdef __cplusplus
}
#endif

#endif