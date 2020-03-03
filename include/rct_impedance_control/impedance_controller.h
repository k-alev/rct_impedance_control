#pragma once

#include <ros/node_handle.h>
#include <hardware_interface/joint_command_interface.h>
#include <controller_interface/controller.h>
#include <controller_interface/multi_interface_controller.h>

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/force_torque_sensor_interface.h>
#include <pluginlib/class_list_macros.h>

#include <Eigen/Dense>

#include <rct_impedance_control/array_torque_sensors.h>

#include <ros_control_toolbox/robot.h>
#include <ros_control_toolbox/status.h>

namespace rct_impedance_control
{

class ImpedanceController : public controller_interface::
                                MultiInterfaceController<hardware_interface::EffortJointInterface,
                                                         hardware_interface::ForceTorqueSensorInterface, hardware_interface::ArrayTorqueSensorsInterface>
{
public:
    ImpedanceController(void);
    ~ImpedanceController(void);

    bool init(hardware_interface::RobotHW *robot, ros::NodeHandle &n);
    void starting(const ros::Time &time);
    void update(const ros::Time &time, const ros::Duration &duration);
    void stopping(const ros::Time &time);

private:
    std::vector<hardware_interface::JointHandle> joints_;
    hardware_interface::ForceTorqueSensorHandle ft_;
    hardware_interface::ArrayTorqueSensorsHandle trq_arr_;
    //hardware_interface::EffortJointInterface *robot_;
    ros::NodeHandle node_;

    rct::robot *Mico;
};

} // namespace RCTcontrol
