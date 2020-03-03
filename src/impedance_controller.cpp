#include <rct_impedance_control/impedance_controller.h>
#include <rct_impedance_control/robot_impl.h>


namespace rct_impedance_control
{
ImpedanceController::ImpedanceController(void){};
ImpedanceController::~ImpedanceController(void){};

bool ImpedanceController::init(hardware_interface::RobotHW *robot, ros::NodeHandle &n)
{

    hardware_interface::EffortJointInterface *robot_ = robot->get<hardware_interface::EffortJointInterface>();
    hardware_interface::ForceTorqueSensorInterface *ft_sensor = robot->get<hardware_interface::ForceTorqueSensorInterface>();
    hardware_interface::ArrayTorqueSensorsInterface *arr_trq_sensors = robot->get<hardware_interface::ArrayTorqueSensorsInterface>();
    node_ = n;
    //robot_=robot;

    XmlRpc::XmlRpcValue joint_names;
    if (!node_.getParam("joints", joint_names))
    {
        ROS_ERROR("No 'joints' in controller. (namespace: %s)",
                  node_.getNamespace().c_str());
        return false;
    }

    if (joint_names.getType() != XmlRpc::XmlRpcValue::TypeArray)
    {
        ROS_ERROR("'joints' is not a struct. (namespace: %s)",
                  node_.getNamespace().c_str());
        return false;
    }

    for (int i = 0; i < joint_names.size(); i++)
    {
        XmlRpc::XmlRpcValue &name_value = joint_names[i];
        if (name_value.getType() != XmlRpc::XmlRpcValue::TypeString)
        {
            ROS_ERROR("joints are not strings. (namespace: %s)",
                      node_.getNamespace().c_str());
            return false;
        }

        hardware_interface::JointHandle j = robot_->getHandle((std::string)name_value);
        joints_.push_back(j);
    }

    ft_ = ft_sensor->getHandle("ft_sensor");
    trq_arr_ = arr_trq_sensors->getHandle("arr_trq_sensors");

    std::string robot_desc_string;
    if (!node_.getParam("/robot_description", robot_desc_string))
    {
        ROS_ERROR("Could not find '/robot_description'.");
        return false;
    }

    //YAML
    std::cout << "initializing robot..." << std::endl;
    std::string root = "root";
    std::string ee = "m1n6s200_end_effector";
    Mico = new rct::robot(robot_desc_string, root, ee, -9.81);
    // ...

    return true;
}

void ImpedanceController::starting(const ros::Time &time)
{
    ROS_INFO("Starting Controller");
}

void ImpedanceController::update(const ros::Time &time, const ros::Duration &duration)
{
    ROS_INFO("Update Controller");
    int takis = 2;
    Mico->send_commands<int>(takis);
}

void ImpedanceController::stopping(const ros::Time &time)
{
    ROS_INFO("Stopping Controller");
}
}

PLUGINLIB_EXPORT_CLASS(rct_impedance_control::ImpedanceController, controller_interface::ControllerBase)
