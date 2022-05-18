// Copyright 2020 ROBOTIS CO., LTD.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/*******************************************************************************
 * This example is written for DYNAMIXEL X(excluding XL-320) and MX(2.0) series with U2D2.
 * For other series, please refer to the product eManual and modify the Control Table addresses and other definitions.
 * To test this example, please follow the commands below.
 *
 * Author: Zerom
*******************************************************************************/

#include <ros/ros.h>

#include "std_msgs/String.h"
#include "dynamixel_sdk_examples/GetPosition.h"
#include "dynamixel_sdk_examples/SetPosition.h"
#include "dynamixel_sdk_examples/GetVelocity.h"
#include "dynamixel_sdk_examples/SetVelocity.h"
#include "dynamixel_sdk_examples/SetStop.h"
#include "dynamixel_sdk/dynamixel_sdk.h"

using namespace dynamixel;

// Control table address
#define ADDR_TORQUE_ENABLE    64
#define ADDR_GOAL_POSITION    116
#define ADDR_PRESENT_POSITION 132
#define ADDR_PRESENT_VELOCITY 128
#define ADDR_GOAL_VELOCITY    104
#define ADDR_LED_ON_OFF       65

// Protocol version
#define PROTOCOL_VERSION      2.0             // Default Protocol version of DYNAMIXEL X series.

// Default setting
#define DXL1_ID               1               // DXL1 ID
#define DXL2_ID               2               // DXL2 ID
#define DXL3_ID               3               // DXL2 ID
#define BAUDRATE              57600           // Default Baudrate of DYNAMIXEL X series
#define DEVICE_NAME           "/dev/ttyUSB0"  // [Linux] To find assigned port, use "$ ls /dev/ttyUSB*" command

PortHandler * portHandler;
PacketHandler * packetHandler;

bool getPresentVelocityCallback(
  dynamixel_sdk_examples::GetVelocity::Request & req,
  dynamixel_sdk_examples::GetVelocity::Response & res)
{
  uint8_t dxl_error = 0;
  int dxl_comm_result = COMM_TX_FAIL;

  // Velocity Value of X series is 4 byte data. For AX & MX(1.0) use 2 byte data(int16_t) for the Velocity Value.
  int32_t velocity = 0;

  // Read Present Velocity (length : 4 bytes) and Convert uint32 -> int32
  // When reading 2 byte data from AX / MX(1.0), use read2ByteTxRx() instead.
  dxl_comm_result = packetHandler->read4ByteTxRx(
    portHandler, (uint8_t)req.id, ADDR_PRESENT_VELOCITY, (uint32_t *)&velocity, &dxl_error);
  if (dxl_comm_result == COMM_SUCCESS) {
    ROS_INFO("getVelocity : [ID:%d] -> [VELOCITY:%d]", req.id, velocity);
    res.velocity = velocity;
    return true;
  } else {
    ROS_INFO("Failed to get velocity! Result: %d", dxl_comm_result);
    return false;
  }
}

void setStopCallback(const dynamixel_sdk_examples::SetStop::ConstPtr & msg)
{
  uint8_t dxl_error = 0;
  int dxl_comm_result = COMM_TX_FAIL;

  // ========================== TORQUE DISABLE ==========================
  dxl_comm_result = packetHandler->write1ByteTxRx(
    portHandler, DXL1_ID, ADDR_TORQUE_ENABLE, 0, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS) {
    ROS_ERROR("Failed to enable torque for Dynamixel ID %d", DXL1_ID);
  }

  dxl_comm_result = packetHandler->write1ByteTxRx(
    portHandler, DXL2_ID, ADDR_TORQUE_ENABLE, 0, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS) {
    ROS_ERROR("Failed to enable torque for Dynamixel ID %d", DXL2_ID);
  }

  dxl_comm_result = packetHandler->write1ByteTxRx(
    portHandler, DXL3_ID, ADDR_TORQUE_ENABLE, 0, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS) {
    ROS_ERROR("Failed to enable torque for Dynamixel ID %d", DXL3_ID);
  }
  // ====================================================================
  // ============================== LED OFF =============================
  dxl_comm_result = packetHandler->write1ByteTxRx(
    portHandler, DXL1_ID, ADDR_LED_ON_OFF, 0, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS) {
    ROS_ERROR("Failed to enable LED for Dynamixel ID %d", DXL1_ID);
  }

  dxl_comm_result = packetHandler->write1ByteTxRx(
    portHandler, DXL2_ID, ADDR_LED_ON_OFF, 0, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS) {
    ROS_ERROR("Failed to enable LED for Dynamixel ID %d", DXL2_ID);
  }

  dxl_comm_result = packetHandler->write1ByteTxRx(
    portHandler, DXL3_ID, ADDR_LED_ON_OFF, 0, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS) {
    ROS_ERROR("Failed to enable LED for Dynamixel ID %d", DXL3_ID);
  }
  // ====================================================================

  // Velocity Value of X series is 4 byte data. For AX & MX(1.0) use 2 byte data(uint16_t) for the Velocity Value.
  uint32_t velocity = (unsigned int)msg->velocity; // Convert int32 -> uint32

  // Write Goal Velocity (length : 4 bytes)
  // When writing 2 byte data to AX / MX(1.0), use write2ByteTxRx() instead.
  dxl_comm_result = packetHandler->write4ByteTxRx(
    portHandler, (uint8_t)msg->id, ADDR_GOAL_VELOCITY, velocity, &dxl_error);
  if (dxl_comm_result == COMM_SUCCESS) {
    ROS_INFO("setVelocity : [ID:%d] [VELOCITY:%d]", msg->id, msg->velocity);
  } else {
    ROS_ERROR("Failed to set velocity! Result: %d", dxl_comm_result);
  }
}

int main(int argc, char ** argv)
{
  uint8_t dxl_error = 0;
  int dxl_comm_result = COMM_TX_FAIL;

  portHandler = PortHandler::getPortHandler(DEVICE_NAME);
  packetHandler = PacketHandler::getPacketHandler(PROTOCOL_VERSION);

  if (!portHandler->openPort()) {
    ROS_ERROR("Failed to open the port!");
    return -1;
  }

  if (!portHandler->setBaudRate(BAUDRATE)) {
    ROS_ERROR("Failed to set the baudrate!");
    return -1;
  }

  ros::init(argc, argv, "read_write_stop");
  ros::NodeHandle nh;
  ros::ServiceServer get_stop_srv = nh.advertiseService("/get_velocity", getPresentVelocityCallback);
  ros::Subscriber set_stop = nh.subscribe("/set_stop", 10, setStopCallback);
  ros::spin();

  portHandler->closePort();
  return 0;
}
