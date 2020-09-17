#pragma once
#ifndef IMUDATA_H_
#define IMUDATA_H_

struct IMUData
{
    float timestamp;
    float acc[3];
    float gyro[3];
    float mag[3];
    IMUData(float* data);
};

#endif  // IMUDATA_H_
