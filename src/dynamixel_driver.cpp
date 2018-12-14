#include "dynamixel_driver/dynamixel_driver.h"
// ROS parameter loading
#include <rosparam_shortcuts/rosparam_shortcuts.h>

namespace dynamixel_driver {

DynamixelDriver::DynamixelDriver(ros::NodeHandle& nodeHandle)
  : nodeHandle_(nodeHandle)
  , name_("dynamixel_driver")
{
  // read the dxl motor names and communication settings for the Dynamixels from the parameter server
  readParameters();

  // Initialize PortHandler instance
  // Set the port path
  // Get methods and members of PortHandlerLinux or PortHandlerWindows
  port_handler_ = dynamixel::PortHandler::getPortHandler(device_name_.c_str());

  // Initialize PacketHandler instance
  // Set the protocol version
  // Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
  packet_handler_ = dynamixel::PacketHandler::getPacketHandler(protocol_version_);
}

DynamixelDriver::~DynamixelDriver()
{
  // Close port
  port_handler_->closePort();
}

bool DynamixelDriver::readParameters()
{
  // Load rosparams
  ros::NodeHandle rpnh(nodeHandle_, "dynamixel");
  std::size_t error = 0;

  error += !rosparam_shortcuts::get(name_, rpnh, "motors", dxl_names_);
  error += !rosparam_shortcuts::get(name_, rpnh, "device_name", device_name_);
  error += !rosparam_shortcuts::get(name_, rpnh, "protocol_version", protocol_version_);
  error += !rosparam_shortcuts::get(name_, rpnh, "baudrate", baudrate_);

  //for all dynamixel names, get params and fill a vector of dxl_configurations
  num_dxl_ = dxl_names_.size();
  dxl_configs_.resize(num_dxl_);

  for(std::size_t i = 0; i < num_dxl_; ++i)
  {
    std::string dxl_name = dxl_names_[i];
    ros::NodeHandle motor_nh(rpnh, dxl_name);

    //rosparam only supports ints for getting parameters so we need these temporary variables
    int id_temp;
    int series_temp;
    int cw_ang_temp;
    int ccw_ang_temp;
    int cw_margin_temp;
    int ccw_margin_temp;
    int cw_slope_temp;
    int ccw_slope_temp;
    int p_gain_temp;
    int i_gain_temp;
    int d_gain_temp;
    int torque_temp;
    int punch_temp;

    error += !rosparam_shortcuts::get(name_, motor_nh, "id", id_temp);
    dxl_configs_[i].id = id_temp;
    error += !rosparam_shortcuts::get(name_, motor_nh, "series", series_temp);
    dxl_configs_[i].series = series_temp;
    error += !rosparam_shortcuts::get(name_, motor_nh, "cw_angle_limit", cw_ang_temp);
    dxl_configs_[i].cw_angle_limit = cw_ang_temp;
    error += !rosparam_shortcuts::get(name_, motor_nh, "ccw_angle_limit", ccw_ang_temp);
    dxl_configs_[i].ccw_angle_limit = ccw_ang_temp;
    error += !rosparam_shortcuts::get(name_, motor_nh, "torque_limit", torque_temp);
    dxl_configs_[i].torque_limit = torque_temp;
    error += !rosparam_shortcuts::get(name_, motor_nh, "punch", punch_temp);
    dxl_configs_[i].punch = punch_temp;

    // if this is an AX series dynamixel, get the compliance parameters
    if(series_temp == dxl_configuration::AX)
    {
      error += !rosparam_shortcuts::get(name_, motor_nh, "cw_margin", cw_margin_temp);
      dxl_configs_[i].cw_compliance_margin = cw_margin_temp;
      error += !rosparam_shortcuts::get(name_, motor_nh, "ccw_margin", ccw_margin_temp);
      dxl_configs_[i].ccw_compliance_margin = ccw_margin_temp;
      error += !rosparam_shortcuts::get(name_, motor_nh, "cw_slope", cw_slope_temp);
      dxl_configs_[i].cw_compliance_slope = cw_slope_temp;
      error += !rosparam_shortcuts::get(name_, motor_nh, "ccw_slope", ccw_slope_temp);
      dxl_configs_[i].ccw_compliance_slope = ccw_slope_temp;
    }
    // if this is an MX series dynamixel, get the pid parameters
    else if(series_temp == dxl_configuration::MX64)
    {
      error += !rosparam_shortcuts::get(name_, motor_nh, "p_gain", p_gain_temp);
      dxl_configs_[i].p_gain = p_gain_temp;
      error += !rosparam_shortcuts::get(name_, motor_nh, "i_gain", i_gain_temp);
      dxl_configs_[i].i_gain = i_gain_temp;
      error += !rosparam_shortcuts::get(name_, motor_nh, "d_gain", d_gain_temp);
      dxl_configs_[i].d_gain = d_gain_temp;
    }
  }

  rosparam_shortcuts::shutdownIfError(name_, error);

  return true;
}

bool DynamixelDriver::configureAllDynamixels()
{
  for(std::size_t i = 0; i < num_dxl_; ++i)
  {
    configureDynamixel(i);
  }

  return true;
}
/** Configure a specific Dynamixel actuator with config_id index in the dxl_configs_ vector
 ***/
bool DynamixelDriver::configureDynamixel(std::size_t config_id)
{
  std::size_t error = 0;

  error += !setCWAngleLimit(dxl_configs_[config_id].id, dxl_configs_[config_id].cw_angle_limit);
  error += !setCCWAngleLimit(dxl_configs_[config_id].id, dxl_configs_[config_id].ccw_angle_limit);
  error += !setTorqueLimit(dxl_configs_[config_id].id, dxl_configs_[config_id].torque_limit);
  error += !setPunch(dxl_configs_[config_id].id, dxl_configs_[config_id].punch);
  // if this is an AX series dynamixel, set the compliance parameters
  if(dxl_configs_[config_id].series == dxl_configuration::AX)
  {
    error += !setCWCompMargin(dxl_configs_[config_id].id, dxl_configs_[config_id].cw_compliance_margin);
    error += !setCCWCompMargin(dxl_configs_[config_id].id, dxl_configs_[config_id].ccw_compliance_margin);
    error += !setCWCompSlope(dxl_configs_[config_id].id, dxl_configs_[config_id].cw_compliance_slope);
    error += !setCCWCompSlope(dxl_configs_[config_id].id, dxl_configs_[config_id].ccw_compliance_slope);
  }
  // if this is an MX64 series dynamixel, set the pid paramters
  else if(dxl_configs_[config_id].series == dxl_configuration::MX64)
  {
    error += !setPGain(dxl_configs_[config_id].id, dxl_configs_[config_id].p_gain);
    error += !setIGain(dxl_configs_[config_id].id, dxl_configs_[config_id].i_gain);
    error += !setDGain(dxl_configs_[config_id].id, dxl_configs_[config_id].d_gain);
  }

  if(error)
  {
    ROS_ERROR("Failed to configure Dynamixel with id %d. Shutting Down...", dxl_configs_[config_id].id);
    ros::shutdown();
  }

  return true;
}

bool DynamixelDriver::init()
{
   return init(baudrate_);
}

bool DynamixelDriver::init(int baudrate)
{
  // Open port
  if (port_handler_->openPort())
  {
    ROS_INFO("Successfully opened the dynamixel port");
  }
  else
  {
    ROS_WARN("Failed to open the dynamixel port!");
    return false;
  }

  // Set port baudrate
  if (port_handler_->setBaudRate(baudrate))
  {
    ROS_INFO("Successfully changed the dynamixel baudrate to %d", baudrate);
  }
  else
  {
    ROS_WARN("Failed to change the dynamixel baudrate!");
    return false;
  }

  configureAllDynamixels();

  return true;
}

bool DynamixelDriver::enableTorque(uint8_t id)
{
  dxl_comm_result_ = packet_handler_->write1ByteTxRx(port_handler_, id, dynamixel_constants::dxl_addr_ax_torque_en,
                                                    dynamixel_constants::dxl_torque_enable, &dxl_error_);

  return checkCommResult(dxl_comm_result_, dxl_error_);
}

bool DynamixelDriver::disableTorque(uint8_t id)
{
  dxl_comm_result_ = packet_handler_->write1ByteTxRx(port_handler_, id, dynamixel_constants::dxl_addr_ax_torque_en,
                                                    dynamixel_constants::dxl_torque_disable, &dxl_error_);

  return checkCommResult(dxl_comm_result_, dxl_error_);
}

bool DynamixelDriver::setCWAngleLimit(uint8_t id, uint16_t limit)
{
  dxl_comm_result_ = packet_handler_->write2ByteTxRx(port_handler_, id, dynamixel_constants::dxl_addr_ax_cw_limit,
                                                     limit, &dxl_error_);

  return checkCommResult(dxl_comm_result_, dxl_error_);
}

bool DynamixelDriver::setCCWAngleLimit(uint8_t id, uint16_t limit)
{
  dxl_comm_result_ = packet_handler_->write2ByteTxRx(port_handler_, id, dynamixel_constants::dxl_addr_ax_ccw_limit,
                                                     limit, &dxl_error_);

  return checkCommResult(dxl_comm_result_, dxl_error_);
}

bool DynamixelDriver::setPGain(uint8_t id, uint8_t p_gain)
{
  dxl_comm_result_ = packet_handler_->write1ByteTxRx(port_handler_, id, dynamixel_constants::dxl_addr_mx_p_gain,
                                                     p_gain, &dxl_error_);

  return checkCommResult(dxl_comm_result_, dxl_error_);
}

bool DynamixelDriver::setIGain(uint8_t id, uint8_t i_gain)
{
  dxl_comm_result_ = packet_handler_->write1ByteTxRx(port_handler_, id, dynamixel_constants::dxl_addr_mx_i_gain,
                                                     i_gain, &dxl_error_);

  return checkCommResult(dxl_comm_result_, dxl_error_);
}

bool DynamixelDriver::setDGain(uint8_t id, uint8_t d_gain)
{
  dxl_comm_result_ = packet_handler_->write1ByteTxRx(port_handler_, id, dynamixel_constants::dxl_addr_mx_d_gain,
                                                     d_gain, &dxl_error_);

  return checkCommResult(dxl_comm_result_, dxl_error_);
}

bool DynamixelDriver::setCWCompMargin(uint8_t id, uint8_t margin)
{
  dxl_comm_result_ = packet_handler_->write1ByteTxRx(port_handler_, id, dynamixel_constants::dxl_addr_ax_cw_comp_margin,
                                                     margin, &dxl_error_);

  return checkCommResult(dxl_comm_result_, dxl_error_);
}

bool DynamixelDriver::setCCWCompMargin(uint8_t id, uint8_t margin)
{
  dxl_comm_result_ = packet_handler_->write1ByteTxRx(port_handler_, id, dynamixel_constants::dxl_addr_ax_ccw_comp_margin,
                                                     margin, &dxl_error_);

  return checkCommResult(dxl_comm_result_, dxl_error_);
}

bool DynamixelDriver::setCWCompSlope(uint8_t id, uint8_t slope)
{
  dxl_comm_result_ = packet_handler_->write1ByteTxRx(port_handler_, id, dynamixel_constants::dxl_addr_ax_cw_comp_slope,
                                                     slope, &dxl_error_);

  return checkCommResult(dxl_comm_result_, dxl_error_);
}

bool DynamixelDriver::setCCWCompSlope(uint8_t id, uint8_t slope)
{
  dxl_comm_result_ = packet_handler_->write1ByteTxRx(port_handler_, id, dynamixel_constants::dxl_addr_ax_ccw_comp_slope,
                                                     slope, &dxl_error_);

  return checkCommResult(dxl_comm_result_, dxl_error_);
}

bool DynamixelDriver::setGoalPosition(uint8_t id, uint16_t goal)
{
  dxl_comm_result_ = packet_handler_->write2ByteTxRx(port_handler_, id, dynamixel_constants::dxl_addr_ax_goal_position,
                                                     goal, &dxl_error_);

  return checkCommResult(dxl_comm_result_, dxl_error_);
}

bool DynamixelDriver::setMovingSpeed(uint8_t id, uint16_t speed)
{
  dxl_comm_result_ = packet_handler_->write2ByteTxRx(port_handler_, id, dynamixel_constants::dxl_addr_ax_moving_speed,
                                                     speed, &dxl_error_);

  return checkCommResult(dxl_comm_result_, dxl_error_);
}

bool DynamixelDriver::setTorqueLimit(uint8_t id, uint16_t torque)
{
  dxl_comm_result_ = packet_handler_->write2ByteTxRx(port_handler_, id, dynamixel_constants::dxl_addr_ax_torque_limit,
                                                     torque, &dxl_error_);

  return checkCommResult(dxl_comm_result_, dxl_error_);
}

bool DynamixelDriver::setPunch(uint8_t id, uint16_t punch)
{
  dxl_comm_result_ = packet_handler_->write2ByteTxRx(port_handler_, id, dynamixel_constants::dxl_addr_ax_punch,
                                                     punch, &dxl_error_);

  return checkCommResult(dxl_comm_result_, dxl_error_);
}

bool DynamixelDriver::getCWAngleLimit(uint8_t id, uint16_t *limit)
{
  dxl_comm_result_ = packet_handler_->read2ByteTxRx(port_handler_, id, dynamixel_constants::dxl_addr_ax_cw_limit,
                                                    limit, &dxl_error_);

  return checkCommResult(dxl_comm_result_, dxl_error_);
}

bool DynamixelDriver::getCCWAngleLimit(uint8_t id, uint16_t *limit)
{
  dxl_comm_result_ = packet_handler_->read2ByteTxRx(port_handler_, id, dynamixel_constants::dxl_addr_ax_ccw_limit,
                                                    limit, &dxl_error_);

  return checkCommResult(dxl_comm_result_, dxl_error_);
}

bool DynamixelDriver::getPGain(uint8_t id, uint8_t *p_gain)
{
  dxl_comm_result_ = packet_handler_->read1ByteTxRx(port_handler_, id, dynamixel_constants::dxl_addr_mx_p_gain,
                                                    p_gain, &dxl_error_);

  return checkCommResult(dxl_comm_result_, dxl_error_);
}

bool DynamixelDriver::getIGain(uint8_t id, uint8_t *i_gain)
{
  dxl_comm_result_ = packet_handler_->read1ByteTxRx(port_handler_, id, dynamixel_constants::dxl_addr_mx_i_gain,
                                                    i_gain, &dxl_error_);

  return checkCommResult(dxl_comm_result_, dxl_error_);
}

bool DynamixelDriver::getDGain(uint8_t id, uint8_t *d_gain)
{
  dxl_comm_result_ = packet_handler_->read1ByteTxRx(port_handler_, id, dynamixel_constants::dxl_addr_mx_d_gain,
                                                    d_gain, &dxl_error_);

  return checkCommResult(dxl_comm_result_, dxl_error_);
}

bool DynamixelDriver::getCWCompMargin(uint8_t id, uint8_t *cw_margin)
{
  dxl_comm_result_ = packet_handler_->read1ByteTxRx(port_handler_, id, dynamixel_constants::dxl_addr_ax_cw_comp_margin,
                                                    cw_margin, &dxl_error_);

  return checkCommResult(dxl_comm_result_, dxl_error_);
}

bool DynamixelDriver::getCCWCompMargin(uint8_t id, uint8_t *ccw_margin)
{
  dxl_comm_result_ = packet_handler_->read1ByteTxRx(port_handler_, id, dynamixel_constants::dxl_addr_ax_ccw_comp_margin,
                                                    ccw_margin, &dxl_error_);

  return checkCommResult(dxl_comm_result_, dxl_error_);
}

bool DynamixelDriver::getCWCompSlope(uint8_t id, uint8_t *cw_slope)
{
  dxl_comm_result_ = packet_handler_->read1ByteTxRx(port_handler_, id, dynamixel_constants::dxl_addr_ax_cw_comp_slope,
                                                    cw_slope, &dxl_error_);

  return checkCommResult(dxl_comm_result_, dxl_error_);
}

bool DynamixelDriver::getCCWCompSlope(uint8_t id, uint8_t *ccw_slope)
{
  dxl_comm_result_ = packet_handler_->read1ByteTxRx(port_handler_, id, dynamixel_constants::dxl_addr_ax_ccw_comp_slope,
                                                    ccw_slope, &dxl_error_);

  return checkCommResult(dxl_comm_result_, dxl_error_);
}

bool DynamixelDriver::getGoalPosition(uint8_t id, uint16_t *goal)
{
  dxl_comm_result_ = packet_handler_->read2ByteTxRx(port_handler_, id, dynamixel_constants::dxl_addr_ax_goal_position,
                                                    goal, &dxl_error_);

  return checkCommResult(dxl_comm_result_, dxl_error_);
}

bool DynamixelDriver::getMovingSpeed(uint8_t id, uint16_t *speed)
{
  dxl_comm_result_ = packet_handler_->read2ByteTxRx(port_handler_, id, dynamixel_constants::dxl_addr_ax_moving_speed,
                                                    speed, &dxl_error_);

  return checkCommResult(dxl_comm_result_, dxl_error_);
}

bool DynamixelDriver::getTorqueLimit(uint8_t id, uint16_t *torque)
{
  dxl_comm_result_ = packet_handler_->read2ByteTxRx(port_handler_, id, dynamixel_constants::dxl_addr_ax_torque_limit,
                                                    torque, &dxl_error_);

  return checkCommResult(dxl_comm_result_, dxl_error_);
}

bool DynamixelDriver::getPresentPosition(uint8_t id, uint16_t *position)
{
  dxl_comm_result_ = packet_handler_->read2ByteTxRx(port_handler_, id, dynamixel_constants::dxl_addr_ax_position,
                                                    position, &dxl_error_);

  return checkCommResult(dxl_comm_result_, dxl_error_);
}

bool DynamixelDriver::getPresentSpeed(uint8_t id, uint16_t *pres_speed)
{
  dxl_comm_result_ = packet_handler_->read2ByteTxRx(port_handler_, id, dynamixel_constants::dxl_addr_ax_speed,
                                                    pres_speed, &dxl_error_);

  return checkCommResult(dxl_comm_result_, dxl_error_);
}

bool DynamixelDriver::getPresentLoad(uint8_t id, uint16_t *load)
{
  dxl_comm_result_ = packet_handler_->read2ByteTxRx(port_handler_, id, dynamixel_constants::dxl_addr_ax_load,
                                                    load, &dxl_error_);

  return checkCommResult(dxl_comm_result_, dxl_error_);
}

bool DynamixelDriver::getPresentVoltage(uint8_t id, uint8_t *voltage)
{
  dxl_comm_result_ = packet_handler_->read1ByteTxRx(port_handler_, id, dynamixel_constants::dxl_addr_ax_voltage,
                                                    voltage, &dxl_error_);

  return checkCommResult(dxl_comm_result_, dxl_error_);
}

bool DynamixelDriver::getPresentTemp(uint8_t id, uint8_t *temp)
{
  dxl_comm_result_ = packet_handler_->read1ByteTxRx(port_handler_, id, dynamixel_constants::dxl_addr_ax_temp,
                                                    temp, &dxl_error_);

  return checkCommResult(dxl_comm_result_, dxl_error_);
}

bool DynamixelDriver::getMovingStatus(uint8_t id, uint8_t *move_status)
{
  dxl_comm_result_ = packet_handler_->read1ByteTxRx(port_handler_, id, dynamixel_constants::dxl_addr_ax_moving,
                                                    move_status, &dxl_error_);

  return checkCommResult(dxl_comm_result_, dxl_error_);
}

bool DynamixelDriver::getPunch(uint8_t id, uint16_t *punch)
{
  dxl_comm_result_ = packet_handler_->read2ByteTxRx(port_handler_, id, dynamixel_constants::dxl_addr_ax_punch,
                                                    punch, &dxl_error_);

  return checkCommResult(dxl_comm_result_, dxl_error_);
}

/** pass the id's of the dynamixels as they are ordered in the config file to dxl_ids
 **/
bool DynamixelDriver::getIDMap(std::vector<uint8_t> &dxl_ids)
{
  dxl_ids.resize(dxl_configs_.size());
  for(std::size_t i = 0; i < dxl_configs_.size(); ++i)
  {
    dxl_ids[i] = dxl_configs_[i].id;
  }
}

bool DynamixelDriver::checkCommResult(int comm_result, uint8_t error)
{
  if (comm_result != COMM_SUCCESS)
  {
    ROS_WARN("%s\n", packet_handler_->getTxRxResult(comm_result));
    return false;
  }
  else if (error != 0)
  {
    ROS_WARN("%s\n", packet_handler_->getRxPacketError(error));
    return false;
  }

  return true;
}

} /* namespace */
