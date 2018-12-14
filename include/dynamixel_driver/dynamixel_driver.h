#ifndef DYNAMIXEL_DRIVER_H
#define DYNAMIXEL_DRIVER_H

#include <ros/ros.h>
#include <dynamixel_sdk/dynamixel_sdk.h>
#include "dynamixel_constants.h"
#include <dynamixel_driver/dxl_configuration.h>

namespace dynamixel_driver {

/*!
 * Class for providing interface functions with Dynamixel servos using the dynamixel_sdk
 */
class DynamixelDriver
{
 public:
  /*!
   * Constructor.
   */
  DynamixelDriver(ros::NodeHandle& nodeHandle);

  /*!
   * Destructor.
   */
  ~DynamixelDriver();

  bool readParameters();
  bool configureAllDynamixels();
  bool configureDynamixel(std::size_t config_id);

  bool init();
  bool init(int baudrate);

  bool checkCommResult(int comm_result, uint8_t error);

  bool enableTorque(uint8_t id);
  bool disableTorque(uint8_t id);

  bool setCWAngleLimit(uint8_t id, uint16_t limit);
  bool setCCWAngleLimit(uint8_t id, uint16_t limit);
  bool setPGain(uint8_t id, uint8_t p_gain);
  bool setIGain(uint8_t id, uint8_t i_gain);
  bool setDGain(uint8_t id, uint8_t d_gain);
  bool setCWCompMargin(uint8_t id, uint8_t margin);
  bool setCCWCompMargin(uint8_t id, uint8_t margin);
  bool setCWCompSlope(uint8_t id, uint8_t slope);
  bool setCCWCompSlope(uint8_t id, uint8_t slope);
  bool setGoalPosition(uint8_t id, uint16_t goal);
  bool setMovingSpeed(uint8_t id, uint16_t speed);
  bool setTorqueLimit(uint8_t id, uint16_t torque);
  bool setPunch(uint8_t id, uint16_t punch);

  bool getCWAngleLimit(uint8_t id, uint16_t *limit);
  bool getCCWAngleLimit(uint8_t id, uint16_t *limit);
  bool getPGain(uint8_t id, uint8_t *p_gain);
  bool getIGain(uint8_t id, uint8_t *i_gain);
  bool getDGain(uint8_t id, uint8_t *d_gain);
  bool getCWCompMargin(uint8_t id, uint8_t *cw_margin);
  bool getCCWCompMargin(uint8_t id, uint8_t *ccw_margin);
  bool getCWCompSlope(uint8_t id, uint8_t *cw_slope);
  bool getCCWCompSlope(uint8_t id, uint8_t *ccw_slope);
  bool getGoalPosition(uint8_t id, uint16_t *goal);
  bool getMovingSpeed(uint8_t id, uint16_t *speed);
  bool getTorqueLimit(uint8_t id, uint16_t *torque);
  bool getPresentPosition(uint8_t id, uint16_t *position);
  bool getPresentSpeed(uint8_t id, uint16_t *pres_speed);
  bool getPresentLoad(uint8_t id, uint16_t *load);
  bool getPresentVoltage(uint8_t id, uint8_t *voltage);
  bool getPresentTemp(uint8_t id, uint8_t *temp);
  bool getMovingStatus(uint8_t id, uint8_t *move_status);
  bool getPunch(uint8_t id, uint16_t *punch);

  bool getIDMap(std::vector<uint8_t> &dxl_ids);

 private:

  std::string name_;
  std::vector<std::string> dxl_names_;
  std::size_t num_dxl_;
  std::vector<dxl_configuration> dxl_configs_;

  // dynamixel handler object pointers
  dynamixel::PortHandler *port_handler_;
  dynamixel::PacketHandler *packet_handler_;

  uint8_t dxl_error_;
  int dxl_comm_result_;

  std::string device_name_;
  double protocol_version_;
  int baudrate_;

  // ROS node handle
  ros::NodeHandle& nodeHandle_;
};

} /* namespace */

#endif // DYNAMIXEL_DRIVER_H
