// This file is part of OpenMVG, an Open Multiple View Geometry C++ library.

// Copyright (c) 2015 Pierre MOULON.

// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.


#include "sequential_SfM_IMU.hpp"
#include <iostream>

namespace openMVG {
namespace sfm {

SequentialSfMReconstructionEngineIMU::SequentialSfMReconstructionEngineIMU(
    const SfM_Data & sfm_data,
    const std::string & soutDirectory,
    const std::string & loggingFile)
    : SequentialSfMReconstructionEngine(sfm_data,soutDirectory,loggingFile)
{
    integrator = nullptr;
}

SequentialSfMReconstructionEngineIMU::SequentialSfMReconstructionEngineIMU(
        const SfM_Data & sfm_data,
        const SfM_IMU & imu_data,
        const ViewStamps & view_stamps,
        const std::string & soutDirectory,
        const std::string & loggingFile)
        : SequentialSfMReconstructionEngine(sfm_data,soutDirectory,loggingFile)
{
    imu_data_ = imu_data;
    view_stamps_ = view_stamps;
    integrator = new IMUDynamics::IMUIntegrator();

    using namespace std;
    cout<<"[Inside SequentialSfMReconstructionEngineIMU Constructor]"<<endl;
    for(size_t i=0;i<imu_data.Size();i++)
    {
        cout<<"acc # "<<i<<": "<<imu_data.getAcceleration(IndexT(i)).transpose()<<endl;
        cout<<"gyro # "<<i<<": "<<imu_data.getOmega(IndexT(i)).transpose()<<endl;
        if(i>=100)
            break;
    }
}

}  // namespace sfm
}  // namespace openMVG