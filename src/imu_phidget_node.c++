#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Bool.h>
#include <std_srvs/Empty.h>

namespace
{
const char kInputTopic[] = "/imu/data_raw";
const char kOutputTopic[] = "/imu/data";
const char kFrameId[] = "imu_link";
const char kCalibrateService[] = "/imu/calibrate";
const char kIsCalibratedTopic[] = "/imu/is_calibrated";
const bool kOverrideFrameId = true;
const int kQueueSize = 20;
const double kCalibrateServiceWaitSec = 10.0;
}  // namespace

class PhidgetImuNode
{
public:
  PhidgetImuNode()
    : nh_()
  {
    calibrated_sub_ = nh_.subscribe(kIsCalibratedTopic,
                                    kQueueSize,
                                    &PhidgetImuNode::isCalibratedCallback,
                                    this);

    calibrate_client_ = nh_.serviceClient<std_srvs::Empty>(kCalibrateService);

    calibrateImuOnStartup();

    pub_ = nh_.advertise<sensor_msgs::Imu>(kOutputTopic, kQueueSize);
    sub_ = nh_.subscribe(kInputTopic, kQueueSize, &PhidgetImuNode::imuCallback, this);

    ROS_INFO_STREAM("imu_phidget_node iniciado.");
    ROS_INFO_STREAM("Escuchando IMU Phidget en: " << kInputTopic);
    ROS_INFO_STREAM("Publicando IMU en: " << kOutputTopic);
  }

private:
  void calibrateImuOnStartup()
  {
    ROS_INFO_STREAM("Esperando servicio de calibracion: " << kCalibrateService);

    if (!ros::service::waitForService(kCalibrateService, ros::Duration(kCalibrateServiceWaitSec)))
    {
      ROS_WARN_STREAM("No se encontro el servicio " << kCalibrateService
                      << " dentro de " << kCalibrateServiceWaitSec
                      << "s. El nodo continuara sin calibracion inicial.");
      return;
    }

    std_srvs::Empty srv;
    ROS_INFO_STREAM("Iniciando calibracion de IMU...");

    if (calibrate_client_.call(srv))
    {
      ROS_INFO("Calibracion IMU finalizada correctamente (servicio respondio).");
    }
    else
    {
      ROS_ERROR("Fallo al llamar /imu/calibrate. Continuando sin confirmar calibracion.");
    }
  }

  void isCalibratedCallback(const std_msgs::Bool::ConstPtr& msg)
  {
    if (msg->data)
    {
      ROS_INFO("imu/is_calibrated: true (IMU calibrada)");
    }
    else
    {
      ROS_WARN("imu/is_calibrated: false (IMU no calibrada)");
    }
  }

  void imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
  {
    sensor_msgs::Imu out = *msg;

    if (kOverrideFrameId || out.header.frame_id.empty())
    {
      out.header.frame_id = kFrameId;
    }

    if (out.header.stamp.isZero())
    {
      out.header.stamp = ros::Time::now();
    }

    pub_.publish(out);
  }

  ros::NodeHandle nh_;
  ros::Subscriber sub_;
  ros::Subscriber calibrated_sub_;
  ros::Publisher pub_;
  ros::ServiceClient calibrate_client_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "imu_phidget_node");
  PhidgetImuNode node;
  ros::spin();
  return 0;
}
