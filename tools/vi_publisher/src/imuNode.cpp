#include <ros/ros.h>
#include<sensor_msgs/Imu.h>

#include "vi_publisher/csvReader.h"
#include "vi_publisher/imuData.h"

int main(int argc, char** argv)
{
    if(argc < 2)
    {
        cout<<"usage: imu_node imu.csv"<<endl;
        exit(-1);
    }
    
ros::init(argc,argv,"imu_node");
ros::NodeHandle n;

ros::Publisher imu_pub = n.advertise<sensor_msgs::Imu>("imu0", 1000);
ros::Rate loop_rate(200);

csvReader reader(argv[1]);

while(ros::ok())
{
    if(reader.readline())
    {
        IMUData data(reader.data);
        sensor_msgs::Imu imu_data;

        imu_data.angular_velocity.x = data.gyro[0];
        imu_data.angular_velocity.y = data.gyro[1];
        imu_data.angular_velocity.z = data.gyro[2];

        imu_data.linear_acceleration.x = data.acc[0];
        imu_data.linear_acceleration.y = data.acc[1];
        imu_data.linear_acceleration.z = data.acc[2];

        imu_data.header.frame_id = to_string(data.timestamp);
        imu_data.header.stamp = ros::Time::now();

        imu_pub.publish(imu_data);
    }
    ros::spinOnce();
    loop_rate.sleep();
}

// while(reader.readline())
// {
//     IMUData data(reader.data);
//     cout<<"Timestamp: "<<data.timestamp<<endl;
//     cout<<"acc: "<<data.acc[0]<<" "<<data.acc[1]<<" "<<data.acc[2]<<endl;
//     cout<<"gyro: "<<data.gyro[0]<<" "<<data.gyro[1]<<" "<<data.gyro[2]<<endl;
//     cout<<"mag: "<<data.mag[0]<<" "<<data.mag[1]<<" "<<data.mag[2]<<endl;
//     cout<<endl;
// }
    
    return 0;
}