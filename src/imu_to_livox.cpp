#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

class HokuyoImuToLivox{
    private:
        ros::NodeHandle _nh;
        ros::NodeHandle _nhPrivate;
        ros::Subscriber _sub_pc;
        ros::Publisher _pub_pc;
        std::string _target_frame;

        sensor_msgs::Imu _imu_datas;

        int _dataCount;
        const int Publish_count = 7;

        const double G_value = 9.80665;

    public:
        HokuyoImuToLivox();
        void callbackPC(const sensor_msgs::Imu msgs);
        
};

HokuyoImuToLivox::HokuyoImuToLivox()
    : _nhPrivate("~")
{
    _nhPrivate.param("target_frame", _target_frame, std::string("base_link"));
    _sub_pc = _nh.subscribe("/hokuyo3d/imu", 10, &HokuyoImuToLivox::callbackPC, this);
    _pub_pc = _nh.advertise<sensor_msgs::Imu>("/hokuyo3d/imu_livox", 10);
    _dataCount = 0;
}

void HokuyoImuToLivox::callbackPC(const sensor_msgs::Imu msgs)
{
    sensor_msgs::Imu data = msgs;

    // fix pose
    data.linear_acceleration.x = msgs.linear_acceleration.z;
    data.linear_acceleration.y = msgs.linear_acceleration.x;
    data.linear_acceleration.z = msgs.linear_acceleration.y;
    data.angular_velocity.x = msgs.angular_velocity.z;
    data.angular_velocity.y = msgs.angular_velocity.x;
    data.angular_velocity.z = msgs.angular_velocity.y;

    // based on gravitational acceleration
    data.linear_acceleration.x /= G_value;
    data.linear_acceleration.y /= G_value;
    data.linear_acceleration.z /= G_value;

    // add average data
    _imu_datas.linear_acceleration.x += data.linear_acceleration.x;
    _imu_datas.linear_acceleration.y += data.linear_acceleration.y;
    _imu_datas.linear_acceleration.z += data.linear_acceleration.z;
    _imu_datas.angular_velocity.x    += data.angular_velocity.x;
    _imu_datas.angular_velocity.y    += data.angular_velocity.y;
    _imu_datas.angular_velocity.z    += data.angular_velocity.z;

    //_pub_pc.publish(data);

    _dataCount++;

    if(_dataCount >= Publish_count){
        sensor_msgs::Imu data_pub = msgs;   // set latest time data

        data_pub.linear_acceleration.x = _imu_datas.linear_acceleration.x / Publish_count;
        data_pub.linear_acceleration.y = _imu_datas.linear_acceleration.y / Publish_count;
        data_pub.linear_acceleration.z = _imu_datas.linear_acceleration.z / Publish_count;
        data_pub.angular_velocity.x    = _imu_datas.angular_velocity.x / Publish_count;
        data_pub.angular_velocity.y    = _imu_datas.angular_velocity.y / Publish_count;
        data_pub.angular_velocity.z    = _imu_datas.angular_velocity.z / Publish_count;

        _pub_pc.publish(data_pub);

        // initialize
        _imu_datas.linear_acceleration.x = 0;
        _imu_datas.linear_acceleration.y = 0;
        _imu_datas.linear_acceleration.z = 0;
        _imu_datas.angular_velocity.x = 0;
        _imu_datas.angular_velocity.y = 0;
        _imu_datas.angular_velocity.z = 0;

        _dataCount = 0;
    }

}

int main(int argc, char **argv){

    ros::init(argc, argv, "hokuyo_imu_to_livox_node");

    HokuyoImuToLivox imu_to_livox;

    ros::spin();

    return 0;

}

