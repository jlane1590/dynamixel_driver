/**   Node for testing and debugging dynamixel_driver with Dynamixel motors
  *
  */

#include <ros/ros.h>
#include "dynamixel_driver/dynamixel_driver.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "dynamixel_driver_test_node");
  ros::NodeHandle nodeHandle("~");

  dynamixel_driver::DynamixelDriver dynamixelDriver(nodeHandle);
  dynamixelDriver.init();

  uint16_t cw_limit;
  uint16_t ccw_limit;
  uint8_t cw_margin;
  uint8_t ccw_margin;
  uint8_t cw_slope;
  uint8_t ccw_slope;

  dynamixelDriver.getCWCompMargin(1, &cw_margin);
  ROS_INFO("CW Margin = %d", cw_margin);
  dynamixelDriver.getCCWCompMargin(1, &ccw_margin);
  ROS_INFO("CCW Margin = %d", ccw_margin);
  dynamixelDriver.getCWCompSlope(1, &cw_slope);
  ROS_INFO("CW Slope = %d", cw_slope);
  dynamixelDriver.getCCWCompSlope(1, &ccw_slope);
  ROS_INFO("CCW Slope = %d", ccw_slope);
  dynamixelDriver.getCWAngleLimit(1, &cw_limit);
  ROS_INFO("CW Limit = %d", cw_limit);
  dynamixelDriver.getCCWAngleLimit(1, &ccw_limit);
  ROS_INFO("CCW Limit = %d", ccw_limit);

  double joint_goal = 0.5;
  double joint_position;
  ROS_INFO("Joint goal: %f", joint_goal);
  dynamixelDriver.setGoalRadians(1,joint_goal);
  ros::Duration(2.0).sleep();
  dynamixelDriver.getPresentRadians(1,joint_position);
  ROS_INFO("Joint position: %f", joint_position);
/*
  dynamixelDriver.setMovingSpeed(1, 90);
  dynamixelDriver.setGoalPosition(1, 710);
  ros::Duration(0.5).sleep();
  dynamixelDriver.setGoalPosition(1, 314);
  //dynamixelDriver.setGoalPosition(1, 710);

  dynamixelDriver.getCWCompMargin(2, &cw_margin);
  ROS_INFO("CW Margin = %d", cw_margin);
  dynamixelDriver.getCCWCompMargin(2, &ccw_margin);
  ROS_INFO("CCW Margin = %d", ccw_margin);
  dynamixelDriver.getCWCompSlope(2, &cw_slope);
  ROS_INFO("CW Slope = %d", cw_slope);
  dynamixelDriver.getCCWCompSlope(2, &ccw_slope);
  ROS_INFO("CCW Slope = %d", ccw_slope);
  dynamixelDriver.getCWAngleLimit(2, &cw_limit);
  ROS_INFO("CW Limit = %d", cw_limit);
  dynamixelDriver.getCCWAngleLimit(2, &ccw_limit);
  ROS_INFO("CCW Limit = %d", ccw_limit);

  std::vector<uint8_t> dxl_ids;

  dynamixelDriver.getIDMap(dxl_ids);

  ROS_INFO("DXL ID Map:");
  for(std::size_t i=0; i < dxl_ids.size(); ++i)
    ROS_INFO("%d, %d", i, dxl_ids[i]);

  uint16_t dxl_present_position = 700;
  double joint_position = ((int)dxl_present_position - 512)/(195.57);
  ROS_INFO("Joint Position: %f", joint_position);

  uint16_t dxl_goal = 512 + (195.57*joint_position);
  ROS_INFO("DXL Goal: %d", dxl_goal);

  uint8_t cw_margin;
  uint8_t ccw_margin;
  uint8_t cw_slope;
  uint8_t ccw_slope;
  uint16_t move_speed;
  uint8_t move_status;

  dynamixelDriver.setCWCompMargin(1, 12);
  dynamixelDriver.setCCWCompMargin(1, 17);
  dynamixelDriver.setCWCompSlope(1, 49);
  dynamixelDriver.setCCWCompSlope(1, 212);
  dynamixelDriver.setMovingSpeed(1, 90);

  dynamixelDriver.getCWCompMargin(1, &cw_margin);
  ROS_INFO("CW Margin = %d", cw_margin);
  dynamixelDriver.getCCWCompMargin(1, &ccw_margin);
  ROS_INFO("CCW Margin = %d", ccw_margin);
  dynamixelDriver.getCWCompSlope(1, &cw_slope);
  ROS_INFO("CW Slope = %d", cw_slope);
  dynamixelDriver.getCCWCompSlope(1, &ccw_slope);
  ROS_INFO("CCW Slope = %d", ccw_slope);
  dynamixelDriver.getMovingSpeed(1, &move_speed);
  ROS_INFO("Moving Speed = %d", move_speed);

  dynamixelDriver.setGoalPosition(1, 0);
  do{
    dynamixelDriver.getMovingStatus(1, &move_status);
  }while(move_status == 1);
  ROS_INFO("Goal Reached 1");
  dynamixelDriver.setGoalPosition(1, 1023);
  do{
    dynamixelDriver.getMovingStatus(1, &move_status);
  }while(move_status == 1);
  ROS_INFO("Goal Reached 2");
  dynamixelDriver.setGoalPosition(1, 0);
  do{
    dynamixelDriver.getMovingStatus(1, &move_status);
  }while(move_status == 1);
  ROS_INFO("Goal Reached 3");

  uint16_t curr_position;
  dynamixelDriver.setGoalPosition(3, 360);

  do{
    // Read present position
    dynamixelDriver.getPresentPosition(3, &curr_position);

    ROS_INFO("PresPos:%03d", curr_position);

  }while((abs(360 - curr_position) > 10));

  if(dynamixelDriver.enableTorque(3))
  {
    ROS_INFO("Torque enabled on servo 3");
  }
  else
  {
    ROS_INFO("Failed to enable torque on servo 3");
  }
  */
  ros::spin();
  return 0;
}
