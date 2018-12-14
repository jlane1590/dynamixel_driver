#ifndef DYNAMIXEL_CONSTANTS_H
#define DYNAMIXEL_CONSTANTS_H

namespace dynamixel_constants {

/* Dynamixel AX series EEPROM register addresses */

const int dxl_addr_ax_id = 3;
const int dxl_addr_ax_baudrate = 4;
const int dxl_addr_ax_return_delay = 5;
const int dxl_addr_ax_cw_limit = 6;
const int dxl_addr_ax_ccw_limit = 8;
const int dxl_addr_ax_temp_limit = 11;
const int dxl_addr_ax_min_volt = 12;
const int dxl_addr_ax_max_volt = 13;
const int dxl_addr_ax_max_torque = 14;
const int dxl_addr_ax_status_level = 16;
const int dxl_addr_ax_alarm_led = 17;
const int dxl_addr_ax_shutdown = 18;

/* Dynamixel AX series RAM register addresses */

const int dxl_addr_ax_torque_en = 24;
const int dxl_addr_ax_led = 25;
const int dxl_addr_ax_cw_comp_margin = 26;
const int dxl_addr_ax_ccw_comp_margin = 27;
const int dxl_addr_ax_cw_comp_slope = 28;
const int dxl_addr_ax_ccw_comp_slope = 29;
const int dxl_addr_ax_goal_position = 30;
const int dxl_addr_ax_moving_speed = 32;
const int dxl_addr_ax_torque_limit = 34;
const int dxl_addr_ax_position = 36;
const int dxl_addr_ax_speed = 38;
const int dxl_addr_ax_load = 40;
const int dxl_addr_ax_voltage = 42;
const int dxl_addr_ax_temp = 43;
const int dxl_addr_ax_registered = 44;
const int dxl_addr_ax_moving = 46;
const int dxl_addr_ax_lock = 47;
const int dxl_addr_ax_punch = 48;

/* Dynamixel MX-64 EEPROM register addresses */

const int dxl_addr_mx_id = 3;
const int dxl_addr_mx_baudrate = 4;
const int dxl_addr_mx_return_delay = 5;
const int dxl_addr_mx_cw_limit = 6;
const int dxl_addr_mx_ccw_limit = 8;
const int dxl_addr_mx_temp_limit = 11;
const int dxl_addr_mx_min_volt = 12;
const int dxl_addr_mx_max_volt = 13;
const int dxl_addr_mx_max_torque = 14;
const int dxl_addr_mx_status_level = 16;
const int dxl_addr_mx_alarm_led = 17;
const int dxl_addr_mx_shutdown = 18;
const int dxl_addr_mx_multiturn_offset = 20;
const int dxl_addr_mx_res_divider = 22;

/* Dynamixel MX-64 RAM register addresses */

const int dxl_addr_mx_torque_en = 24;
const int dxl_addr_mx_led = 25;
const int dxl_addr_mx_d_gain = 26;
const int dxl_addr_mx_i_gain = 27;
const int dxl_addr_mx_p_gain = 28;
const int dxl_addr_mx_goal_position = 30;
const int dxl_addr_mx_moving_speed = 32;
const int dxl_addr_mx_torque_limit = 34;
const int dxl_addr_mx_position = 36;
const int dxl_addr_mx_speed = 38;
const int dxl_addr_mx_load = 40;
const int dxl_addr_mx_voltage = 42;
const int dxl_addr_mx_temp = 43;
const int dxl_addr_mx_registered = 44;
const int dxl_addr_mx_moving = 46;
const int dxl_addr_mx_lock = 47;
const int dxl_addr_mx_punch = 48;
const int dxl_addr_mx_tick = 50;
const int dxl_addr_mx_current = 68;
const int dxl_addr_mx_torque_ctrl_en = 70;
const int dxl_addr_mx_goal_torque = 71;
const int dxl_addr_mx_goal_acc = 73;

const int dxl_torque_enable = 1;
const int dxl_torque_disable = 0;
}

#endif // DYNAMIXEL_CONSTANTS_H
