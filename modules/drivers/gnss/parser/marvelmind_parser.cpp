/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/
#include <math.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <memory>
#include <iomanip>

#include <limits>
#include <iostream>
#include <cmath>
#include <vector>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>

#include "ros/include/ros/ros.h"

#include "modules/common/log.h"
#include "modules/drivers/gnss/parser/marvelmind_messages.h"
#include "modules/drivers/gnss/parser/parser.h"
//#include "modules/drivers/gnss/parser/rtcm_decode.h"
#include "modules/drivers/gnss/proto/gnss.pb.h"
#include "modules/drivers/gnss/proto/gnss_best_pose.pb.h"
#include "modules/drivers/gnss/proto/gnss_raw_observation.pb.h"
#include "modules/drivers/gnss/proto/heading.pb.h"
#include "modules/drivers/gnss/proto/imu.pb.h"
#include "modules/drivers/gnss/proto/ins.pb.h"
#include "modules/drivers/gnss/util/time_conversion.h"
#include "modules/drivers/gnss/gnss_gflags.h"

DEFINE_double(marvelmind_axis_coordinate_amp_factor, 1.00, "Ratio of Axis Amplification");

namespace apollo {
namespace drivers {
namespace gnss {
// Anonymous namespace that contains helper constants and functions.
namespace {
    constexpr size_t BUFFER_SIZE = 256;
    constexpr double DEG_TO_RAD = M_PI / 180.0;

    constexpr float FLOAT_NAN = std::numeric_limits<float>::quiet_NaN();
    constexpr double azimuth_deg_to_yaw_rad(double azimuth, double angle) {
      return (angle - azimuth) * DEG_TO_RAD;
    }

}  // namespace

class MarvelmindParser : public Parser {
 public:
    MarvelmindParser();
    explicit MarvelmindParser(const config::Config& config);
    virtual MessageType GetMessage(MessagePtr* message_ptr);

 private:
    Parser::MessageType PrepareMessage(MessagePtr* message_ptr);
//    Parser* CreateMarvelmind(const config::Config& config);
    // -1 is an unused value.
    uint16_t CalcCrcModbus_(uint8_t *buf, int len);
    void Imu100d2Callback(const sensor_msgs::Imu& msg);
    void Imu100d2MagCallback(const sensor_msgs::MagneticField& msg);

    std::vector<uint8_t> buffer_;
    size_t total_length_ = 0;
    //struct MarvelmindHedge *hedge_;
    struct apollo::drivers::gnss::marvelmind::MarvelmindHedge *hedge_;
    ::apollo::drivers::gnss::Ins ins_;
    //::apollo::drivers::gnss::ZhdGps zhdgps_;
    // add ycy for zhd gps
    config::ZhdConfig zhd_config_;

    double FusionRoll;
    double FusionPitch;
    double FusionYaw;
    double velocityX;
    double velocityY;
    double velocityZ;
    std::unique_ptr<ros::NodeHandle> node_handle_;
    ros::Subscriber imu100d2_sub_;
    ros::Subscriber imu100d2mag_sub_;
};

Parser* Parser::CreateMarvelmind(const config::Config& config) {
  return new MarvelmindParser(config);
}

void MarvelmindParser::Imu100d2MagCallback(const sensor_msgs::MagneticField& msg)
{
/*
    double magx = msg.magnetic_field.x * cos(FusionPitch)
       + msg.magnetic_field.y * sin(FusionRoll) * sin(FusionPitch) +
      msg.magnetic_field.z * cos(FusionRoll) * sin(FusionPitch);
    double magy = msg.magnetic_field.y * cos(FusionRoll) -
      msg.magnetic_field.z * sin(FusionRoll);
    FusionYaw = 180 * atan2(-magy,magx)/M_PI;
    ins_.mutable_euler_angles()->set_x(FusionRoll/180*M_PI);
    ins_.mutable_euler_angles()->set_y(FusionPitch/180*M_PI);
    ins_.mutable_euler_angles()->set_z(FusionYaw/180*M_PI);

    ROS_ERROR("roll = %lf,pitch = %lf,yaw = %lf\n",FusionRoll/180*M_PI,FusionPitch/180*M_PI,FusionYaw/180*M_PI);
*/
}

void MarvelmindParser::Imu100d2Callback(const sensor_msgs::Imu& msg)
{
    ins_.mutable_angular_velocity()->set_x(msg.angular_velocity.x);
    ins_.mutable_angular_velocity()->set_y(msg.angular_velocity.x);
    ins_.mutable_angular_velocity()->set_z(msg.angular_velocity.x);

    ins_.mutable_linear_acceleration()->set_x(msg.linear_acceleration.x * FLAGS_marvelmind_axis_coordinate_amp_factor);
    ins_.mutable_linear_acceleration()->set_y(msg.linear_acceleration.y * FLAGS_marvelmind_axis_coordinate_amp_factor);
    ins_.mutable_linear_acceleration()->set_z(msg.linear_acceleration.z * FLAGS_marvelmind_axis_coordinate_amp_factor);
    Eigen::Quaterniond q(msg.orientation.w,msg.orientation.x,msg.orientation.y,msg.orientation.z);
/*
    q.x() = msg.orientation.x;
    q.y() = msg.orientation.y;
    q.z() = msg.orientation.z;
    q.w() = msg.orientation.w;
*/
    Eigen::Vector3d eulerAngle=q.toRotationMatrix().eulerAngles(0,1,2);
    ins_.mutable_euler_angles()->set_x(eulerAngle.x());
    ins_.mutable_euler_angles()->set_y(eulerAngle.y());
    ins_.mutable_euler_angles()->set_z(eulerAngle.z());
//    ROS_ERROR("roll = %lf,pitch = %lf,yaw = %lf\n",eulerAngle.x(),eulerAngle.y(),eulerAngle.z());
/*
    FusionPitch = 180*atan2(msg.linear_acceleration.x,
      sqrt(msg.linear_acceleration.y * msg.linear_acceleration.y
       + msg.linear_acceleration.z * msg.linear_acceleration.z))/M_PI;
    FusionRoll = 180*atan2(msg.linear_acceleration.y,
      sqrt(msg.linear_acceleration.x * msg.linear_acceleration.x
       + msg.linear_acceleration.z * msg.linear_acceleration.z))/M_PI;
    double timeDiff = msg.header.stamp.toSec() - ros::Time::now().toSec();
*/
//    velocityX = velocityX + msg.linear_acceleration.x * timeDiff;
//    velocityY = velocityY + msg.linear_acceleration.y * timeDiff;
//    velocityZ = velocityZ + msg.linear_acceleration.z * timeDiff;
}

static int16_t get_int16(uint8_t *buffer)
{
  int16_t res= buffer[0] |
                 (((uint16_t ) buffer[1])<<8);
                 
    return res;
}

static uint32_t get_uint32(uint8_t *buffer)
{
  uint32_t res= buffer[0] |
            (((uint32_t ) buffer[1])<<8) |
            (((uint32_t ) buffer[2])<<16) |
            (((uint32_t ) buffer[3])<<24);
                 
    return res;
}

static int32_t get_int32(uint8_t *buffer)
{
  int32_t res= buffer[0] |
            (((uint32_t ) buffer[1])<<8) |
            (((uint32_t ) buffer[2])<<16) |
            (((uint32_t ) buffer[3])<<24);
                 
    return res;
}

//////////////////////////////////////////////////////////////////////////

static uint8_t markPositionReady(struct marvelmind::MarvelmindHedge *hedge)
{
  uint8_t ind= hedge->lastValues_next;
  uint8_t indCur= ind;

  hedge->positionBuffer[ind].ready=
      true;
  hedge->positionBuffer[ind].processed=
      false;
  ind++;
  if (ind>=hedge->maxBufferedPositions)
      ind=0;
  if (hedge->lastValuesCount_<hedge->maxBufferedPositions)
      hedge->lastValuesCount_++;
  hedge->haveNewValues_=true;

  hedge->lastValues_next= ind;

  return indCur;
}

static struct marvelmind::PositionValue process_position_datagram(struct marvelmind::MarvelmindHedge * hedge, uint8_t *buffer)
{
  uint8_t ind= hedge->lastValues_next;

  hedge->positionBuffer[ind].address=
    buffer[16];
  hedge->positionBuffer[ind].timestamp=
    buffer[5] |
    (((uint32_t ) buffer[6])<<8) |
    (((uint32_t ) buffer[7])<<16) |
    (((uint32_t ) buffer[8])<<24);

  int16_t vx= buffer[9] |
            (((uint16_t ) buffer[10])<<8);
  hedge->positionBuffer[ind].x= vx*10;// millimeters
//AERROR << "---x"<< hedge->positionBuffer[ind].z;
  int16_t vy= buffer[11] |
            (((uint16_t ) buffer[12])<<8);
  hedge->positionBuffer[ind].y= vy*10;// millimeters
//AERROR << "---y"<< hedge->positionBuffer[ind].y;
  int16_t vz= buffer[13] |
            (((uint16_t ) buffer[14])<<8);
  hedge->positionBuffer[ind].z= vz*10;// millimeters
//AERROR << "---z" << hedge->positionBuffer[ind].z;
  hedge->positionBuffer[ind].flags= buffer[15];

  uint16_t vang= buffer[17] |
               (((uint16_t ) buffer[18])<<8);
  hedge->positionBuffer[ind].angle= ((float) (vang & 0x0fff))/10.0f;

  hedge->positionBuffer[ind].highResolution= false;

  ind= markPositionReady(hedge);

  return hedge->positionBuffer[ind];
}

static struct marvelmind::PositionValue process_position_highres_datagram(struct marvelmind::MarvelmindHedge * hedge, uint8_t *buffer)
{
  uint8_t ind= hedge->lastValues_next;

  hedge->positionBuffer[ind].address=
      buffer[22];
  hedge->positionBuffer[ind].timestamp=
      buffer[5] |
      (((uint32_t ) buffer[6])<<8) |
      (((uint32_t ) buffer[7])<<16) |
      (((uint32_t ) buffer[8])<<24);

  int32_t vx= buffer[9] |
              (((uint32_t ) buffer[10])<<8) |
              (((uint32_t ) buffer[11])<<16) |
              (((uint32_t ) buffer[12])<<24);
  hedge->positionBuffer[ind].x= vx;

//AERROR << "H---x"<<hedge->positionBuffer[ind].x;
  int32_t vy= buffer[13] |
              (((uint32_t ) buffer[14])<<8) |
              (((uint32_t ) buffer[15])<<16) |
              (((uint32_t ) buffer[16])<<24);
  hedge->positionBuffer[ind].y= vy;

//AERROR << "H---y"<< hedge->positionBuffer[ind].y;
  int32_t vz= buffer[17] |
              (((uint32_t ) buffer[18])<<8) |
              (((uint32_t ) buffer[19])<<16) |
              (((uint32_t ) buffer[20])<<24);
  hedge->positionBuffer[ind].z= vz;
  
//AERROR << "H---z"<< hedge->positionBuffer[ind].z;
  hedge->positionBuffer[ind].flags= buffer[21];
  
  uint16_t vang= buffer[23] |
                 (((uint16_t ) buffer[24])<<8);
  hedge->positionBuffer[ind].angle= ((float) (vang & 0x0fff))/10.0f;

  hedge->positionBuffer[ind].highResolution= true;

  ind= markPositionReady(hedge);

  return hedge->positionBuffer[ind];
}

static struct marvelmind::StationaryBeaconPosition *getOrAllocBeacon(struct marvelmind::MarvelmindHedge *hedge,uint8_t address)
{
  uint8_t i;
  uint8_t n_used= hedge->positionsBeacons.numBeacons;

  if (n_used != 0)
      for(i=0;i<n_used;i++)
      {
          if (hedge->positionsBeacons.beacons[i].address == address)
          {
              return &hedge->positionsBeacons.beacons[i];
          }
      }

  if (n_used >= (MAX_STATIONARY_BEACONS-1))
      return NULL;

  hedge->positionsBeacons.numBeacons= (n_used + 1);
  return &hedge->positionsBeacons.beacons[n_used];
}

static void process_beacons_positions_datagram(struct marvelmind::MarvelmindHedge *hedge, uint8_t *buffer)
{
  uint8_t n= buffer[5];// number of beacons in packet
  uint8_t i,ofs;
  uint8_t address;
  int16_t x,y,z;
  struct marvelmind::StationaryBeaconPosition *b;

  if ((1+n*8)!=buffer[4])
    return;// incorrect size

  for(i=0;i<n;i++)
  {
    ofs= 6+i*8;

    address= buffer[ofs+0];
    x=  buffer[ofs+1] |
        (((uint16_t ) buffer[ofs+2])<<8);
    y=  buffer[ofs+3] |
        (((uint16_t ) buffer[ofs+4])<<8);
    z=  buffer[ofs+5] |
        (((uint16_t ) buffer[ofs+6])<<8);

    b= getOrAllocBeacon(hedge, address);
    if (b != NULL)
    {
      b->address= address;
      b->x= x*10;// millimeters
      b->y= y*10;// millimeters
      b->z= z*10;// millimeters

      b->highResolution= false;
      b->updatedForMsg= true;

      hedge->positionsBeacons.updated= true;
    }
  }
}

static void process_beacons_positions_highres_datagram(struct marvelmind::MarvelmindHedge *hedge, uint8_t *buffer)
{
  uint8_t n= buffer[5];// number of beacons in packet
  uint8_t i,ofs;
  uint8_t address;
  int32_t x,y,z;
  struct marvelmind::StationaryBeaconPosition *b;

  if ((1+n*14)!=buffer[4])
    return;// incorrect size

  for(i=0;i<n;i++)
  {
    ofs= 6+i*14;

    address= buffer[ofs+0];
    x=  buffer[ofs+1] |
        (((uint32_t ) buffer[ofs+2])<<8) |
        (((uint32_t ) buffer[ofs+3])<<16) |
        (((uint32_t ) buffer[ofs+4])<<24);
    y=  buffer[ofs+5] |
        (((uint32_t ) buffer[ofs+6])<<8) |
        (((uint32_t ) buffer[ofs+7])<<16) |
        (((uint32_t ) buffer[ofs+8])<<24);
    z=  buffer[ofs+9] |
        (((uint32_t ) buffer[ofs+10])<<8) |
        (((uint32_t ) buffer[ofs+11])<<16) |
        (((uint32_t ) buffer[ofs+12])<<24);

    b= getOrAllocBeacon(hedge, address);
    if (b != NULL)
    {
      b->address= address;
      b->x= x;
      b->y= y;
      b->z= z;

      b->highResolution= true;
      b->updatedForMsg= true;

      hedge->positionsBeacons.updated= true;
    }
  }
}

static void process_imu_raw_datagram(struct marvelmind::MarvelmindHedge *hedge, uint8_t *buffer)
{
  uint8_t *dataBuf= &buffer[5];

  hedge->rawIMU.acc_x= get_int16(&dataBuf[0]);
  hedge->rawIMU.acc_y= get_int16(&dataBuf[2]);
  hedge->rawIMU.acc_z= get_int16(&dataBuf[4]);
  
  //
  hedge->rawIMU.gyro_x= get_int16(&dataBuf[6]);
  hedge->rawIMU.gyro_y= get_int16(&dataBuf[8]);
  hedge->rawIMU.gyro_z= get_int16(&dataBuf[10]);
  
  //
  hedge->rawIMU.compass_x= get_int16(&dataBuf[12]);
  hedge->rawIMU.compass_y= get_int16(&dataBuf[14]);
  hedge->rawIMU.compass_z= get_int16(&dataBuf[16]);

  hedge->rawIMU.timestamp= get_uint32(&dataBuf[24]);
  
  hedge->rawIMU.updated= true;
}

static void process_imu_fusion_datagram(struct marvelmind::MarvelmindHedge *hedge, uint8_t *buffer)
{
  uint8_t *dataBuf= &buffer[5];
  
  hedge->fusionIMU.x= get_int32(&dataBuf[0]);
  hedge->fusionIMU.y= get_int16(&dataBuf[4]);
  hedge->fusionIMU.z= get_int16(&dataBuf[8]);
  
  hedge->fusionIMU.qw= get_int16(&dataBuf[12]);
  hedge->fusionIMU.qx= get_int16(&dataBuf[14]);
  hedge->fusionIMU.qy= get_int16(&dataBuf[16]);
  hedge->fusionIMU.qz= get_int16(&dataBuf[18]);
  
  hedge->fusionIMU.vx= get_int16(&dataBuf[20]);
  hedge->fusionIMU.vy= get_int16(&dataBuf[22]);
  hedge->fusionIMU.vz= get_int16(&dataBuf[24]);
  
  hedge->fusionIMU.ax= get_int16(&dataBuf[26]);
  hedge->fusionIMU.ay= get_int16(&dataBuf[28]);
  hedge->fusionIMU.az= get_int16(&dataBuf[30]);

  hedge->fusionIMU.timestamp= get_uint32(&dataBuf[34]);
  
  hedge->fusionIMU.updated= true;
}

static void process_raw_distances_datagram(struct marvelmind::MarvelmindHedge *hedge, uint8_t *buffer)
{
  uint8_t *dataBuf= &buffer[5];
  uint8_t ofs; 

  hedge->rawDistances.address_hedge= dataBuf[0];

  ofs= 1;
  for(uint8_t i=0;i<4;i++)
  {
    hedge->rawDistances.distances[i].address_beacon= dataBuf[ofs+0]; 
    hedge->rawDistances.distances[i].distance= get_uint32(&dataBuf[ofs+1]);
    ofs+= 6;
  }

  hedge->rawDistances.updated= true;
}

MarvelmindParser::MarvelmindParser() {
  buffer_.reserve(BUFFER_SIZE);
  buffer_.reserve(BUFFER_SIZE);
  buffer_.clear();
  total_length_ = 0;

  ins_.mutable_position_covariance()->Resize(9, FLOAT_NAN);
  ins_.mutable_euler_angles_covariance()->Resize(9, FLOAT_NAN);
  ins_.mutable_linear_velocity_covariance()->Resize(9, FLOAT_NAN);
}

MarvelmindParser::MarvelmindParser(const config::Config& config) {    
  node_handle_.reset(new ros::NodeHandle("ImuRecv"));
  if(!FLAGS_indoor_useposimu)
  {
    imu100d2_sub_ = node_handle_->subscribe("/imu_data", 10,
                        &MarvelmindParser::Imu100d2Callback, static_cast<MarvelmindParser *>(this));
    imu100d2mag_sub_ = node_handle_->subscribe("/imu/mag", 10,
                        &MarvelmindParser::Imu100d2MagCallback, static_cast<MarvelmindParser *>(this));
  }
  hedge_ = (struct marvelmind::MarvelmindHedge *)malloc (sizeof (struct marvelmind::MarvelmindHedge));
  if (hedge_)
  {
      hedge_->ttyFileName=DEFAULT_TTY_FILENAME;
      hedge_->baudRate=9600;
      hedge_->maxBufferedPositions=1;
      hedge_->positionBuffer=NULL;
      hedge_->verbose=false;
      hedge_->receiveDataCallback=NULL;
      hedge_->anyInputPacketCallback= NULL;
      hedge_->lastValuesCount_=0;
      hedge_->lastValues_next= 0;
      hedge_->haveNewValues_=false;
      hedge_->terminationRequired= false;
      
      hedge_->rawIMU.updated= false;
      hedge_->fusionIMU.updated= false;
      hedge_->rawDistances.updated= false;
  }
  else ROS_INFO ("Not enough memory");
  if (hedge_==NULL)
  {
      ROS_INFO ("Error: Unable to create MarvelmindHedge");
      return;
  }
//  hedge_->ttyFileName = ttyFileName;
  hedge_->verbose=true; // show errors and warnings
//  hedge_->anyInputPacketCallback = semCallback;
  uint8_t i;
  hedge_->positionBuffer=
      (struct marvelmind::PositionValue *)malloc(sizeof (struct marvelmind::PositionValue)*hedge_->maxBufferedPositions);
  if (hedge_->positionBuffer==NULL)
  {
      if (hedge_->verbose) puts ("Not enough memory");
      hedge_->terminationRequired=true;
      return;
  }
  for(i=0;i<hedge_->maxBufferedPositions;i++)
  {
      hedge_->positionBuffer[i].ready= false;
      hedge_->positionBuffer[i].processed= false;
  }
  for(i=0;i<MAX_STATIONARY_BEACONS;i++)
  {
      hedge_->positionsBeacons.beacons[i].updatedForMsg= false;
  }
  hedge_->positionsBeacons.numBeacons= 0;
  hedge_->positionsBeacons.updated= false;

//  pthread_create (&hedge_->thread_, NULL, Marvelmind_Thread_, hedge_);
#if 0
  buffer_.reserve(BUFFER_SIZE);
  buffer_.clear();
  total_length_ = 0;

  ins_.mutable_position_covariance()->Resize(9, FLOAT_NAN);
  ins_.mutable_euler_angles_covariance()->Resize(9, FLOAT_NAN);
  ins_.mutable_linear_velocity_covariance()->Resize(9, FLOAT_NAN);

  if (config.has_zhd_config()) {
    zhd_config_ = config.zhd_config();
  } else {
    zhd_config_.set_angle_heading(90);
    AERROR << "angle_heading=" << std::hex << zhd_config_.angle_heading();
  }
#endif
}

Parser::MessageType MarvelmindParser::GetMessage(MessagePtr* message_ptr){
  struct marvelmind::PositionValue curPosition;
  uint8_t input_buffer[256];
  uint8_t recvState = marvelmind::RECV_HDR; // current state of receive data
  uint8_t nBytesInBlockReceived=0; // bytes received
  uint16_t dataId;
  MessageType type = MessageType::NONE;
  if (data_ == nullptr) {
    return MessageType::NONE;
  }
  while (data_ < data_end_)
  {
    uint8_t receivedChar;
    receivedChar = *data_;
    data_++;
    if (data_ != nullptr)
    {
      bool goodByte= false;
      input_buffer[nBytesInBlockReceived]= receivedChar;
      switch (recvState)
      {
        case marvelmind::RECV_HDR:
          switch(nBytesInBlockReceived)
          {
            case 0:
              goodByte= (receivedChar == 0xff);
              break;
            case 1:
              goodByte= (receivedChar == 0x47);
              break;
            case 2:
              goodByte= true;
              break;
            case 3:
              dataId= (((uint16_t) receivedChar)<<8) + input_buffer[2];
              goodByte=   (dataId == POSITION_DATAGRAM_ID) ||
              (dataId == BEACONS_POSITIONS_DATAGRAM_ID) ||
              (dataId == POSITION_DATAGRAM_HIGHRES_ID) ||
              (dataId == BEACONS_POSITIONS_DATAGRAM_HIGHRES_ID) ||
              (dataId == IMU_RAW_DATAGRAM_ID) ||
              (dataId == IMU_FUSION_DATAGRAM_ID) ||
              (dataId == BEACON_RAW_DISTANCE_DATAGRAM_ID);
              break;
            case 4:
              switch(dataId )
              {
                case POSITION_DATAGRAM_ID:
                  goodByte= (receivedChar == 0x10);
                  break;
                case BEACONS_POSITIONS_DATAGRAM_ID:
                case BEACONS_POSITIONS_DATAGRAM_HIGHRES_ID:
                  goodByte= true;
                  break;
                case POSITION_DATAGRAM_HIGHRES_ID:
                  goodByte= (receivedChar == 0x16);
                  break;
                case IMU_RAW_DATAGRAM_ID:
                  goodByte= (receivedChar == 0x20);
                  break;
                case IMU_FUSION_DATAGRAM_ID:
                  goodByte= (receivedChar == 0x2a);
                  break;
                case BEACON_RAW_DISTANCE_DATAGRAM_ID:
                  goodByte= (receivedChar == 0x20);
                  break;
              }
              if (goodByte)
                recvState = marvelmind::RECV_DGRAM;
              break;
          }
          if (goodByte)
          {
            // correct header byte
            nBytesInBlockReceived++;
          }
          else
          {
            // ...or incorrect
            recvState = marvelmind::RECV_HDR;
            nBytesInBlockReceived=0;
          }
          break;
        case marvelmind::RECV_DGRAM:
          nBytesInBlockReceived++;
        if (nBytesInBlockReceived>=7+input_buffer[4])
        {
          // parse dgram
          uint16_t blockCrc= CalcCrcModbus_(input_buffer,nBytesInBlockReceived);
          if (blockCrc==0)
          {
            switch(dataId)
            {
              case POSITION_DATAGRAM_ID:
                // add to positionBuffer
                curPosition= process_position_datagram(hedge_, input_buffer);
                type = PrepareMessage(message_ptr);
                break;
              case BEACONS_POSITIONS_DATAGRAM_ID:
                process_beacons_positions_datagram(hedge_, input_buffer);
                break;
              case POSITION_DATAGRAM_HIGHRES_ID:
                // add to positionBuffer
                curPosition= process_position_highres_datagram(hedge_, input_buffer);
                type = PrepareMessage(message_ptr);
                break;
              case BEACONS_POSITIONS_DATAGRAM_HIGHRES_ID:
                process_beacons_positions_highres_datagram(hedge_, input_buffer);
                break;
              case IMU_RAW_DATAGRAM_ID:
                process_imu_raw_datagram(hedge_, input_buffer);
                break;
              case IMU_FUSION_DATAGRAM_ID:
                process_imu_fusion_datagram(hedge_, input_buffer);
                break;
              case BEACON_RAW_DISTANCE_DATAGRAM_ID:
                process_raw_distances_datagram(hedge_, input_buffer);
                break;
            }
            // callback

            if (hedge_->anyInputPacketCallback)
            {
              //hedge_->anyInputPacketCallback();
            }

            if (dataId == POSITION_DATAGRAM_ID)
            {
              //type = PrepareMessage(message_ptr);
            }
          }
          // and repeat
          recvState = marvelmind::RECV_HDR;
          nBytesInBlockReceived=0;
        }
      }
    }
  }

  return type;
}

Parser::MessageType MarvelmindParser::PrepareMessage(MessagePtr* message_ptr) {
  static double timeLast = 0;
  static double lastPosX = 0 ,lastPosY = 0 ,lastPosZ = 0;
  uint8_t ind = hedge_->lastValues_next;
  double timeDiff = 0;
  static int timeUpdate = 0;

  ins_.mutable_header()->set_timestamp_sec(ros::Time::now().toSec());
  ins_.set_measurement_time(hedge_->positionBuffer[ind].timestamp);
  ins_.mutable_position()->set_lon(hedge_->positionBuffer[ind].x * FLAGS_marvelmind_axis_coordinate_amp_factor / 1000.);
  ins_.mutable_position()->set_lat(hedge_->positionBuffer[ind].y * FLAGS_marvelmind_axis_coordinate_amp_factor / 1000.);
  ins_.mutable_position()->set_height(hedge_->positionBuffer[ind].z / 1000.);
  if(!FLAGS_indoor_useposimu/* && timeUpdate++ > FLAGS_indoor_velocity_update*/)
  {
    timeUpdate = 0;
    timeDiff = (ins_.header().timestamp_sec() - timeLast);
    velocityX = (hedge_->positionBuffer[ind].x * FLAGS_marvelmind_axis_coordinate_amp_factor / 1000. - lastPosX) / timeDiff;
    velocityY = (hedge_->positionBuffer[ind].y * FLAGS_marvelmind_axis_coordinate_amp_factor / 1000. - lastPosY) / timeDiff;
    velocityZ = (hedge_->positionBuffer[ind].z / 1000. - lastPosZ) / timeDiff;
  }
  ins_.mutable_linear_velocity()->set_x(velocityX);
  ins_.mutable_linear_velocity()->set_y(velocityY);
  ins_.mutable_linear_velocity()->set_z(velocityZ);
  if(FLAGS_indoor_useposimu && hedge_->rawIMU.updated)
  {
    ins_.mutable_angular_velocity()->set_x(hedge_->rawIMU.gyro_x);
    ins_.mutable_angular_velocity()->set_y(hedge_->rawIMU.gyro_y);
    ins_.mutable_angular_velocity()->set_z(hedge_->rawIMU.gyro_z);
  }
  if(FLAGS_indoor_useposimu && hedge_->fusionIMU.updated)
  {
    Eigen::Quaterniond q;
    q.x() = hedge_->fusionIMU.qx;
    q.y() = hedge_->fusionIMU.qy;
    q.z() = hedge_->fusionIMU.qz;
    q.w() = hedge_->fusionIMU.qw;
    Eigen::Vector3d eulerAngle=q.matrix().eulerAngles(2,1,0);
    ins_.mutable_euler_angles()->set_x(eulerAngle.x()/(double)1000);
    ins_.mutable_euler_angles()->set_y(eulerAngle.y()/(double)1000);
    ins_.mutable_euler_angles()->set_z(eulerAngle.z()/(double)1000);
    ins_.mutable_linear_velocity()->set_x(hedge_->fusionIMU.vx/(double)1000);
    ins_.mutable_linear_velocity()->set_y(hedge_->fusionIMU.vy/(double)1000);
    ins_.mutable_linear_velocity()->set_z(hedge_->fusionIMU.vz/(double)1000);
    ins_.mutable_linear_acceleration()->set_x(hedge_->fusionIMU.ax/(double)1000);
    ins_.mutable_linear_acceleration()->set_y(hedge_->fusionIMU.ay/(double)1000);
    ins_.mutable_linear_acceleration()->set_z(hedge_->fusionIMU.az/(double)1000);
  }
  lastPosX = hedge_->positionBuffer[ind].x * FLAGS_marvelmind_axis_coordinate_amp_factor / 1000.;
  lastPosY = hedge_->positionBuffer[ind].y * FLAGS_marvelmind_axis_coordinate_amp_factor / 1000.;
  lastPosZ = hedge_->positionBuffer[ind].z / 1000.;
  timeLast = ins_.header().timestamp_sec();
  *message_ptr = &ins_;
  return MessageType::INS;
}

uint16_t MarvelmindParser::CalcCrcModbus_(uint8_t *buf, int len)
{
    uint16_t crc = 0xFFFF;
    int pos;
    for (pos = 0; pos < len; pos++)
    {
        crc ^= (uint16_t)buf[pos]; // XOR byte into least sig. byte of crc
        int i;
        for (i = 8; i != 0; i--) // Loop over each bit
        {
            if ((crc & 0x0001) != 0) // If the LSB is set
            {
                crc >>= 1; // Shift right and XOR 0xA001
                crc ^= 0xA001;
            }
            else  // Else LSB is not set
                crc >>= 1; // Just shift right
        }
    }
    return crc;
}

}  // namespace gnss
}  // namespace drivers
}  // namespace apollo
