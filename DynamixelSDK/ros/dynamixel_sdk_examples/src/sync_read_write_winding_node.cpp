// Copyright 2021 ROBOTIS CO., LTD.
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
 * Open terminal #1
 * $ roscore
 *
 * Open terminal #2
 * $ rosrun dynamixel_sdk_examples sync_read_write_node
 *
 * Open terminal #3 (run one of below commands at a time)
 * $ rostopic pub -1 /sync_set_position dynamixel_sdk_examples/SyncSetPosition "{id1: 1, id2: 2, position1: 0, position2: 1000}"
 * $ rostopic pub -1 /sync_set_position dynamixel_sdk_examples/SyncSetPosition "{id1: 1, id2: 2, position1: 1000, position2: 0}"
 * $ rosservice call /sync_get_position "{id1: 1, id2: 2}"
 *
 * Author: Jaehyun Shim
*******************************************************************************/

#include <ros/ros.h>

#include "std_msgs/String.h"
#include "dynamixel_sdk_examples/SyncGetVelocity.h"
#include "dynamixel_sdk_examples/SyncSetVelocity.h"
#include "dynamixel_sdk/dynamixel_sdk.h"

using namespace dynamixel;

// Control table address
#define ADDR_TORQUE_ENABLE            64
#define ADDR_PRESENT_POSITION         132
#define ADDR_PRESENT_VELOCITY         128
#define ADDR_GOAL_POSITION            116
#define ADDR_GOAL_VELOCITY            104
#define ADDR_OPERATING_MODE           11
#define OPERATING_MODE_VELOCITY       1
#define OPERATING_MODE_POSITION       3
#define ADDR_LED_ON_OFF               65

// Protocol version
#define PROTOCOL_VERSION      2.0             // Default Protocol version of DYNAMIXEL X series.

// Default setting
#define DXL1_ID               1               // DXL1 ID
#define DXL2_ID               2               // DXL2 ID
#define DXL3_ID               3               // DXL2 ID
#define BAUDRATE              57600           // Default Baudrate of DYNAMIXEL X series
#define DEVICE_NAME           "/dev/ttyUSB0"  // [Linux] To find assigned port, use "$ ls /dev/ttyUSB*" command

PortHandler * portHandler = PortHandler::getPortHandler(DEVICE_NAME);
PacketHandler * packetHandler = PacketHandler::getPacketHandler(PROTOCOL_VERSION);

GroupSyncRead groupSyncRead(portHandler, packetHandler, ADDR_PRESENT_VELOCITY, 4);
GroupSyncWrite groupSyncWrite(portHandler, packetHandler, ADDR_GOAL_VELOCITY, 4);

bool syncGetPresentVelocityCallback(
  dynamixel_sdk_examples::SyncGetVelocity::Request & req,
  dynamixel_sdk_examples::SyncGetVelocity::Response & res)
{
  uint8_t dxl_error = 0;
  int dxl_comm_result = COMM_TX_FAIL;
  int dxl_addparam_result = false;

  // Velocity Value of X series is 4 byte data. For AX & MX(1.0) use 2 byte data(int16_t) for the Velocity Value.
  int32_t velocity1 = 0;
  int32_t velocity2 = 0;
  int32_t velocity3 = 0;
  // Read Present Velocity (length : 4 bytes) and Convert uint32 -> int32
  // When reading 2 byte data from AX / MX(1.0), use read2ByteTxRx() instead.
  dxl_addparam_result = groupSyncRead.addParam((uint8_t)req.id1);
  if (dxl_addparam_result != true) {
    ROS_ERROR("Failed to addparam to groupSyncRead for Dynamixel ID %d", req.id1);
    return 0;
  }

  dxl_addparam_result = groupSyncRead.addParam((uint8_t)req.id2);
  if (dxl_addparam_result != true) {
    ROS_ERROR("Failed to addparam to groupSyncRead for Dynamixel ID %d", req.id2);
    return 0;
  }

  dxl_addparam_result = groupSyncRead.addParam((uint8_t)req.id3);
  if (dxl_addparam_result != true) {
    ROS_ERROR("Failed to addparam to groupSyncRead for Dynamixel ID %d", req.id3);
    return 0;
  }


  dxl_comm_result = groupSyncRead.txRxPacket();
  if (dxl_comm_result == COMM_SUCCESS) {
    velocity1 = groupSyncRead.getData((uint8_t)req.id1, ADDR_PRESENT_VELOCITY, 4);
    velocity2 = groupSyncRead.getData((uint8_t)req.id2, ADDR_PRESENT_VELOCITY, 4);
    velocity3 = groupSyncRead.getData((uint8_t)req.id3, ADDR_PRESENT_VELOCITY, 4);
    ROS_INFO("getVelocity : [VELOCITY:%d]", velocity1);
    ROS_INFO("getVelocity : [VELOCITY:%d]", velocity2);
    ROS_INFO("getVelocity : [VELOCITY:%d]", velocity3);

    res.velocity1 = velocity1;
    res.velocity2 = velocity2;
    res.velocity3 = velocity3;

    groupSyncRead.clearParam();
    return true;
  } else {
    ROS_ERROR("Failed to get velocity! Result: %d", dxl_comm_result);
    groupSyncRead.clearParam();
    return false;
  }
}

void syncSetVelocityCallback(const dynamixel_sdk_examples::SyncSetVelocity::ConstPtr & msg)
{
  uint8_t dxl_error = 0;
  int dxl_comm_result = COMM_TX_FAIL;
  int dxl_addparam_result = false;
  uint8_t param_goal_velocity1[4];
  uint8_t param_goal_velocity2[4];
  uint8_t param_goal_velocity3[4];

  // Velocity Value of X series is 4 byte data. For AX & MX(1.0) use 2 byte data(uint16_t) for the Velocity Value.
  uint32_t velocity1 = (unsigned int)msg->velocity1; // Convert int32 -> uint32
  param_goal_velocity1[0] = DXL_LOBYTE(DXL_LOWORD(velocity1));
  param_goal_velocity1[1] = DXL_HIBYTE(DXL_LOWORD(velocity1));
  param_goal_velocity1[2] = DXL_LOBYTE(DXL_HIWORD(velocity1));
  param_goal_velocity1[3] = DXL_HIBYTE(DXL_HIWORD(velocity1));
  uint32_t velocity2 = (unsigned int)msg->velocity2; // Convert int32 -> uint32
  param_goal_velocity2[0] = DXL_LOBYTE(DXL_LOWORD(velocity2));
  param_goal_velocity2[1] = DXL_HIBYTE(DXL_LOWORD(velocity2));
  param_goal_velocity2[2] = DXL_LOBYTE(DXL_HIWORD(velocity2));
  param_goal_velocity2[3] = DXL_HIBYTE(DXL_HIWORD(velocity2));
  uint32_t velocity3 = (unsigned int)msg->velocity3; // Convert int32 -> uint32
  param_goal_velocity3[0] = DXL_LOBYTE(DXL_LOWORD(velocity3));
  param_goal_velocity3[1] = DXL_HIBYTE(DXL_LOWORD(velocity3));
  param_goal_velocity3[2] = DXL_LOBYTE(DXL_HIWORD(velocity3));
  param_goal_velocity3[3] = DXL_HIBYTE(DXL_HIWORD(velocity3));


  // Write Goal Velocity (length : 4 bytes)
  // When writing 2 byte data to AX / MX(1.0), use write2ByteTxRx() instead.
  dxl_addparam_result = groupSyncWrite.addParam((uint8_t)msg->id1, param_goal_velocity1);
  if (dxl_addparam_result != true) {
    ROS_ERROR( "Failed to addparam to groupSyncWrite for Dynamixel ID %d", msg->id1);
  }

  dxl_addparam_result = groupSyncWrite.addParam((uint8_t)msg->id2, param_goal_velocity2);
  if (dxl_addparam_result != true) {
    ROS_ERROR( "Failed to addparam to groupSyncWrite for Dynamixel ID %d", msg->id2);
  }

  dxl_addparam_result = groupSyncWrite.addParam((uint8_t)msg->id3, param_goal_velocity3);
  if (dxl_addparam_result != true) {
    ROS_ERROR( "Failed to addparam to groupSyncWrite for Dynamixel ID %d", msg->id3);
  }

  dxl_comm_result = groupSyncWrite.txPacket();
  if (dxl_comm_result == COMM_SUCCESS) {
    ROS_INFO("setVelocity : [ID:%d] [VELOCITY:%d]", msg->id1, msg->velocity1);
    ROS_INFO("setVelocity : [ID:%d] [VELOCITY:%d]", msg->id2, msg->velocity2);
    ROS_INFO("setVelocity : [ID:%d] [VELOCITY:%d]", msg->id3, msg->velocity3);
  } else {
    ROS_ERROR("Failed to set velocity! Result: %d", dxl_comm_result);
  }

  groupSyncWrite.clearParam();
}

int main(int argc, char ** argv)
{
  uint8_t dxl_error = 0;
  int dxl_comm_result = COMM_TX_FAIL;

  if (!portHandler->openPort()) {
    ROS_ERROR("Failed to open the port!");
    return -1;
  }

  if (!portHandler->setBaudRate(BAUDRATE)) {
    ROS_ERROR("Failed to set the baudrate!");
    return -1;
  }
  // ============================ LED ON/OFF =======================
  dxl_comm_result = packetHandler->write1ByteTxRx(
    portHandler, DXL1_ID, ADDR_LED_ON_OFF, 1, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS) {
    ROS_ERROR("Failed to enable LED for Dynamixel ID %d", DXL1_ID);
    return -1;
  }


  dxl_comm_result = packetHandler->write1ByteTxRx(
    portHandler, DXL2_ID, ADDR_LED_ON_OFF, 0, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS) {
    ROS_ERROR("Failed to enable LED for Dynamixel ID %d", DXL2_ID);
    return -1;
  }

  dxl_comm_result = packetHandler->write1ByteTxRx(
    portHandler, DXL3_ID, ADDR_LED_ON_OFF, 0, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS) {
    ROS_ERROR("Failed to enable LED for Dynamixel ID %d", DXL3_ID);
    return -1;
  }
  // ================================================================

  // ======================== VELOCITY MODE =========================
  dxl_comm_result = packetHandler->write1ByteTxRx(
    portHandler, DXL1_ID, ADDR_OPERATING_MODE, 1, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS) {
    ROS_ERROR("Failed to enable LED for Dynamixel ID %d", DXL1_ID);
    return -1;
  }

  dxl_comm_result = packetHandler->write1ByteTxRx(
    portHandler, DXL2_ID, ADDR_OPERATING_MODE, 1, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS) {
    ROS_ERROR("Failed to enable LED for Dynamixel ID %d", DXL2_ID);
    return -1;
  }

  dxl_comm_result = packetHandler->write1ByteTxRx(
    portHandler, DXL3_ID, ADDR_OPERATING_MODE, 1, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS) {
    ROS_ERROR("Failed to enable LED for Dynamixel ID %d", DXL3_ID);
    return -1;
  }
  // ================================================================
  // ==================== TORQUE ENABLE/DISABLE =====================
  dxl_comm_result = packetHandler->write1ByteTxRx(
    portHandler, DXL1_ID, ADDR_TORQUE_ENABLE, 1, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS) {
    ROS_ERROR("Failed to enable torque for Dynamixel ID %d", DXL1_ID);
    return -1;
  }

  dxl_comm_result = packetHandler->write1ByteTxRx(
    portHandler, DXL2_ID, ADDR_TORQUE_ENABLE, 0, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS) {
    ROS_ERROR("Failed to enable torque for Dynamixel ID %d", DXL2_ID);
    return -1;
  }

  dxl_comm_result = packetHandler->write1ByteTxRx(
    portHandler, DXL3_ID, ADDR_TORQUE_ENABLE, 0, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS) {
    ROS_ERROR("Failed to enable torque for Dynamixel ID %d", DXL3_ID);
    return -1;
  }
  // ================================================================

  ros::init(argc, argv, "sync_read_write_winding_node");
  ros::NodeHandle nh;
  ros::ServiceServer sync_get_velocity_srv = nh.advertiseService("/sync_get_winding_velocity", syncGetPresentVelocityCallback);
  ros::Subscriber sync_set_velocity_sub = nh.subscribe("/sync_set_winding_velocity", 10, syncSetVelocityCallback);
  ros::spin();

  portHandler->closePort();
  return 0;
}
