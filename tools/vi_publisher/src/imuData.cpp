#include "imuData.h"

IMUData::IMUData(float* data)
{
    timestamp = data[0];
    for(int i=0;i<3;i++)
    {
        acc[i] = data[i+1];
        gyro[i] = data[i+4];
        mag[i] = data[i+7];
    }
}

