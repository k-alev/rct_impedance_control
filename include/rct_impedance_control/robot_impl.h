#include <ros_control_toolbox/robot.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/force_torque_sensor_interface.h>


template <class T>
void rct::robot::read_from_robot(const T &handle)
{
  std::cout << "reading from robot unspecialized" << std::endl;
}

namespace rct
{
template <>
void robot::read_from_robot<hardware_interface::ForceTorqueSensorHandle>(const hardware_interface::ForceTorqueSensorHandle &handle)
{
  const double *tmp_frc, *tmp_trq;

  tmp_frc = handle.getForce();
  tmp_trq = handle.getTorque();

  for (unsigned int i = 0; i < 3; i++)
  {
    ft.force(i) = tmp_frc[i];
    ft.torque(i) = tmp_trq[i];
  }

  std::cout << "Reading HI " << this->ft.force(2) << std::endl;
}
} // namespace rct


template <class T>
void rct::robot::write_to_robot(const T &handle)
{
  std::cout << "writing to robot unspecialized" << std::endl;
}

namespace rct
{
template <>
void robot::write_to_robot<int>(const int &handle)
{
  std::cout << "writing to specialized robot " << handle << std::endl;
}
} // namespace rct