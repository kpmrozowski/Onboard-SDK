/** @file dji_broadcast.cpp
 *  @version 4.0.0
 *  @date April 2017
 *
 *  @brief
 *  Broadcast Telemetry API for DJI onboardSDK library
 *
 *  @Copyright (c) 2916-2017 DJI
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 */

#include "dji_broadcast.hpp"
#include "dji_vehicle.hpp"

using namespace DJI;
using namespace DJI::OSDK;

void
DataBroadcast::unpackCallback(Vehicle* vehicle, RecvContainer recvFrame,
                              UserData data)
{
  DataBroadcast* broadcastPtr = (DataBroadcast*)data;

  if (broadcastPtr->getVehicle()->isLegacyM600())
  {
    broadcastPtr->unpackOldM600Data(&recvFrame);
  }
  else if (broadcastPtr->getVehicle()->getFwVersion() != Version::M100_31)
  {
    broadcastPtr->unpackData(&recvFrame);
  }
  else
  {
    broadcastPtr->unpackM100Data(&recvFrame);
  }

  if (broadcastPtr->userCbHandler.callback)
  {
    broadcastPtr->userCbHandler.callback(vehicle, recvFrame,
                                         broadcastPtr->userCbHandler.userData);
  }
}

DataBroadcast::DataBroadcast(Vehicle* vehiclePtr)
{
  unpackHandler.callback = unpackCallback;
  unpackHandler.userData = this;

  userCbHandler.callback = 0;
  userCbHandler.userData = 0;

  Platform::instance().mutexCreate(&m_msgLock);
  if (vehiclePtr)
  {
    setVehicle(vehiclePtr);
  }
}

DataBroadcast::~DataBroadcast()
{
  this->setUserBroadcastCallback(0, NULL);
  unpackHandler.callback = 0;
  unpackHandler.userData = 0;
}

// clang-format off
Telemetry::TimeStamp
DataBroadcast::getTimeStamp()
{
  Telemetry::TimeStamp  data;
  lockMSG();
  if (m_vehicle->isLegacyM600())
  {
    // Supported Broadcast data in Matrice 600 old firmware
    data.time_ms = m_legacyTimeStamp.time;
    data.time_ns = m_legacyTimeStamp.nanoTime;
  }
  else if(m_vehicle->isM100())
  {
    // Supported Broadcast data in Matrice 100
    data.time_ms = m_legacyTimeStamp.time;
    data.time_ns = m_legacyTimeStamp.nanoTime;
  }
  else
  {
    data = m_timeStamp;
  }
  freeMSG();
  return data;
}

Telemetry::SyncStamp
DataBroadcast::getSyncStamp()
{
  Telemetry::SyncStamp data = {0};
  lockMSG();
  if (m_vehicle->isLegacyM600())
  {
    // Supported Broadcast data in Matrice 600 old firmware
    data.flag = m_legacyTimeStamp.syncFlag;
  }
  else if(m_vehicle->isM100())
  {
    // Supported Broadcast data in Matrice 100
    data.flag = m_legacyTimeStamp.syncFlag;
  }
  else
  {
    data = m_syncStamp;
  }
  freeMSG();
  return data;
}

Telemetry::Quaternion
DataBroadcast::getQuaternion()
{
  Telemetry::Quaternion data;
  lockMSG();
  data = m_q;
  freeMSG();
  return data;
}

Telemetry::Vector3f
DataBroadcast::getAcceleration()
{
  Telemetry::Vector3f data;
  lockMSG();
  data = m_a;
  freeMSG();
  return data;
}

Telemetry::Vector3f
DataBroadcast::getVelocity()
{
  Telemetry::Vector3f data;
  lockMSG();
  if (m_vehicle->isLegacyM600())
  {
    // Supported Broadcast data in Matrice 600 old firmware
    data.x = m_legacyVelocity.x;
    data.y = m_legacyVelocity.y;
    data.z = m_legacyVelocity.z;
  }
  else if(m_vehicle->isM100())
  {
    // Supported Broadcast data in Matrice 100
    data.x = m_legacyVelocity.x;
    data.y = m_legacyVelocity.y;
    data.z = m_legacyVelocity.z;
  }
  else
  {
    data = m_v;
  }
  freeMSG();
  return data;
}

Telemetry::VelocityInfo
DataBroadcast::getVelocityInfo()
{
  Telemetry::VelocityInfo data;
  lockMSG();
  if (m_vehicle->isLegacyM600())
  {
    // Supported Broadcast data in Matrice 600 old firmware
    data.health = m_legacyVelocity.health;
    data.reserve = m_legacyVelocity.reserve;
  }
  else if(m_vehicle->isM100())
  {
    // Supported Broadcast data in Matrice 100
    data.health = m_legacyVelocity.health;
    data.reserve = m_legacyVelocity.reserve;
    // TODO add sensorID (only M100)
  }
  else
  {
    data = m_vi;
  }
  freeMSG();
  return data;
}

Telemetry::Vector3f
DataBroadcast::getAngularRate()
{
  Telemetry::Vector3f data;
  lockMSG();
  data = m_w;
  freeMSG();
  return data;
}

Telemetry::GlobalPosition
DataBroadcast::getGlobalPosition()
{
  Telemetry::GlobalPosition data;
  lockMSG();
  data = m_gp;
  freeMSG();
  return data;
}

// Not supported on Matrice 100
Telemetry::RelativePosition
DataBroadcast::getRelativePosition()
{
  Telemetry::RelativePosition data;
  lockMSG();
  data = m_rp;
  freeMSG();
  return data;
}

// Not supported on Matrice 100
Telemetry::GPSInfo
DataBroadcast::getGPSInfo()
{
  Telemetry::GPSInfo data;
  lockMSG();
  if (m_vehicle->isLegacyM600())
  {
    // Supported Broadcast data in Matrice 600 old firmware
    data.latitude = m_legacyGPSInfo.latitude;
    data.longitude = m_legacyGPSInfo.longitude;
    data.HFSL = m_legacyGPSInfo.HFSL;
    data.velocityNED = m_legacyGPSInfo.velocityNED;
    data.time = m_legacyGPSInfo.time;
    //GPS details not supported.
  }
  else
  {
    data = m_gps;
  }
  freeMSG();
  return data;
}

// Not supported on Matrice 100
Telemetry::RTK
DataBroadcast::getRTKInfo()
{
  Telemetry::RTK data;
  lockMSG();
  data = m_rtk;
  freeMSG();
  return data;
}

Telemetry::Mag
DataBroadcast::getMag()
{
  Telemetry::Mag data;
  lockMSG();
  data = m_mag;
  freeMSG();
  return data;
}

Telemetry::RC
DataBroadcast::getRC()
{
  Telemetry::RC data;
  lockMSG();
  data = m_rc;
  freeMSG();
  return data;
}

Telemetry::Gimbal
DataBroadcast::getGimbal()
{
  Telemetry::Gimbal data;
  lockMSG();
  data = m_gimbal;
  freeMSG();
  return data;
}

Telemetry::Status
DataBroadcast::getStatus()
{
  Telemetry::Status data = {0};
  lockMSG();
  if (m_vehicle->isLegacyM600())
  {
    // Broadcast data on M600 old firmware. Only flight status is available.
    data.flight = m_legacyStatus;
  }
  else if(m_vehicle->isM100())
  {
    // Supported Broadcast data in Matrice 100
    data.flight = m_legacyStatus;
  }
  else
  {
    data = m_status;
  }
  freeMSG();
  return data;
}

Telemetry::Battery
DataBroadcast::getBatteryInfo()
{
  Telemetry::Battery data = {0};
  lockMSG();
  if (m_vehicle->isLegacyM600())
  {
    // Only capacity is supported on old M600 FW
    data.percentage = m_legacyBattery;
  }
  else if (m_vehicle->isM100())
  {
    // Supported Broadcast data in Matrice 100
    data.percentage = m_legacyBattery;
  }
  else
  {
    data = m_battery;
  }
  freeMSG();
  return data;
}

Telemetry::SDKInfo
DataBroadcast::getSDKInfo()
{
  Telemetry::SDKInfo data;
  lockMSG();
  data = m_info;
  freeMSG();
  return data;
}

Telemetry::Compass
DataBroadcast::getCompassData()
{
    Telemetry::Compass data;
    lockMSG();
    data = m_compass;
    freeMSG();
    return data;
}
// clang-format on

Vehicle*
DataBroadcast::getVehicle() const
{
  return m_vehicle;
}

void
DataBroadcast::setVehicle(Vehicle* vehiclePtr)
{
  m_vehicle = vehiclePtr;
}

void
DataBroadcast::setBroadcastFreq(uint8_t* dataLenIs16, VehicleCallBack callback,
                                UserData userData)
{
  for (int i = 0; i < 16; ++i)
  {
    dataLenIs16[i] = (dataLenIs16[i] > 7 ? 5 : dataLenIs16[i]);
  }

  uint32_t cmd_timeout = 100; // unit is ms
  uint32_t retry_time  = 1;
  VehicleCallBack cb = NULL;
  UserData udata = NULL;

  if (callback)
  {
    cb = callback;
    udata = userData;
  }
  else
  {
    cb = setFrequencyCallback;
    udata = NULL;
  }
  m_vehicle->legacyLinker->sendAsync(
      OpenProtocolCMD::CMDSet::Activation::frequency, (uint8_t *) dataLenIs16,
      16, cmd_timeout, retry_time, cb, udata);
}

ACK::ErrorCode
DataBroadcast::setBroadcastFreq(uint8_t* dataLenIs16, int timeout)
{
  for (int i = 0; i < 16; ++i)
  {
    dataLenIs16[i] = (dataLenIs16[i] > 7 ? 5 : dataLenIs16[i]);
  }

  return *(ACK::ErrorCode*)m_vehicle->legacyLinker->sendSync(
      OpenProtocolCMD::CMDSet::Activation::frequency, dataLenIs16, 16, 100, 1);
}

void
DataBroadcast::unpackData(RecvContainer* pRecvFrame)
{
  uint8_t* pdata = pRecvFrame->recvData.raw_ack_array;
  lockMSG();
  m_passFlag = *(uint16_t*)pdata;
  pdata += sizeof(uint16_t);
  // clang-format off
  unpackOne(FLAG_TIME        ,&m_timeStamp ,pdata,sizeof(m_timeStamp ));
  unpackOne(FLAG_TIME        ,&m_syncStamp ,pdata,sizeof(m_syncStamp ));
  unpackOne(FLAG_QUATERNION  ,&m_q         ,pdata,sizeof(m_q         ));
  unpackOne(FLAG_ACCELERATION,&m_a         ,pdata,sizeof(m_a         ));
  unpackOne(FLAG_VELOCITY    ,&m_v         ,pdata,sizeof(m_v         ));
  unpackOne(FLAG_ANGULAR_RATE,&m_w         ,pdata,sizeof(m_w         ));
  unpackOne(FLAG_VELOCITY    ,&m_vi        ,pdata,sizeof(m_vi        ));
  unpackOne(FLAG_POSITION    ,&m_gp        ,pdata,sizeof(m_gp        ));
  unpackOne(FLAG_POSITION    ,&m_rp        ,pdata,sizeof(m_rp        ));
  unpackOne(FLAG_GPSINFO     ,&m_gps       ,pdata,sizeof(m_gps       ));
  unpackOne(FLAG_RTKINFO     ,&m_rtk       ,pdata,sizeof(m_rtk       ));
  unpackOne(FLAG_MAG         ,&m_mag       ,pdata,sizeof(m_mag       ));
  unpackOne(FLAG_RC          ,&m_rc        ,pdata,sizeof(m_rc        ));
  unpackOne(FLAG_GIMBAL      ,&m_gimbal    ,pdata,sizeof(m_gimbal    ));
  unpackOne(FLAG_STATUS      ,&m_status    ,pdata,sizeof(m_status    ));
  unpackOne(FLAG_BATTERY     ,&m_battery   ,pdata,sizeof(m_battery   ));
  unpackOne(FLAG_DEVICE      ,&m_info      ,pdata,sizeof(m_info      ));
  unpackOne(FLAG_COMPASS     ,&m_compass   ,pdata,sizeof(m_compass   ));
  // clang-format on
  freeMSG();
}

void
DataBroadcast::unpackM100Data(RecvContainer* pRecvFrame)
{
  uint8_t* pdata = pRecvFrame->recvData.raw_ack_array;
  lockMSG();
  m_passFlag = *(uint16_t*)pdata;
  pdata += sizeof(uint16_t);
  // clang-format off
  unpackOne(FLAG_TIME        ,&m_legacyTimeStamp   ,pdata,sizeof(m_legacyTimeStamp ));
  unpackOne(FLAG_QUATERNION  ,&m_q                 ,pdata,sizeof(m_q               ));
  unpackOne(FLAG_ACCELERATION,&m_a                 ,pdata,sizeof(m_a               ));
  unpackOne(FLAG_VELOCITY    ,&m_legacyVelocity    ,pdata,sizeof(m_legacyVelocity  ));
  unpackOne(FLAG_ANGULAR_RATE,&m_w                 ,pdata,sizeof(m_w               ));
  unpackOne(FLAG_POSITION    ,&m_gp                ,pdata,sizeof(m_gp              ));
  unpackOne(FLAG_M100_MAG    ,&m_mag               ,pdata,sizeof(m_mag             ));
  unpackOne(FLAG_M100_RC     ,&m_rc                ,pdata,sizeof(m_rc              ));
  unpackOne(FLAG_M100_GIMBAL ,&m_gimbal            ,pdata,sizeof(m_gimbal          ));
  unpackOne(FLAG_M100_STATUS ,&m_legacyStatus      ,pdata,sizeof(m_legacyStatus    ));
  unpackOne(FLAG_M100_BATTERY,&m_legacyBattery     ,pdata,sizeof(m_legacyBattery   ));
  unpackOne(FLAG_M100_DEVICE ,&m_info              ,pdata,sizeof(m_info            ));
  // clang-format on
  freeMSG();
}

void
DataBroadcast::unpackOldM600Data(RecvContainer* pRecvFrame)
{
  uint8_t* pdata = pRecvFrame->recvData.raw_ack_array;
  lockMSG();
  m_passFlag = *(uint16_t*)pdata;
  pdata += sizeof(uint16_t);
  // clang-format off
  unpackOne(FLAG_TIME        ,&m_legacyTimeStamp   ,pdata,sizeof(m_legacyTimeStamp ));
  unpackOne(FLAG_QUATERNION  ,&m_q                 ,pdata,sizeof(m_q               ));
  unpackOne(FLAG_ACCELERATION,&m_a                 ,pdata,sizeof(m_a               ));
  unpackOne(FLAG_VELOCITY    ,&m_legacyVelocity    ,pdata,sizeof(m_legacyVelocity  ));
  unpackOne(FLAG_ANGULAR_RATE,&m_w                 ,pdata,sizeof(m_w               ));
  unpackOne(FLAG_POSITION    ,&m_gp                ,pdata,sizeof(m_gp              ));
  unpackOne(FLAG_GPSINFO     ,&m_legacyGPSInfo     ,pdata,sizeof(m_legacyGPSInfo   ));
  unpackOne(FLAG_RTKINFO     ,&m_rtk               ,pdata,sizeof(m_rtk             ));
  unpackOne(FLAG_MAG         ,&m_mag               ,pdata,sizeof(m_mag             ));
  unpackOne(FLAG_RC          ,&m_rc                ,pdata,sizeof(m_rc              ));
  unpackOne(FLAG_GIMBAL      ,&m_gimbal            ,pdata,sizeof(m_gimbal          ));
  unpackOne(FLAG_STATUS      ,&m_legacyStatus      ,pdata,sizeof(m_legacyStatus    ));
  unpackOne(FLAG_BATTERY     ,&m_legacyBattery     ,pdata,sizeof(m_legacyBattery   ));
  unpackOne(FLAG_DEVICE      ,&m_info              ,pdata,sizeof(m_info            ));
  // clang-format on
  freeMSG();
}

void
DataBroadcast::unpackOne(DataBroadcast::FLAG flag, void* data, uint8_t*& buf,
                         size_t size)
{
  if (flag & m_passFlag)
  {
    memcpy((uint8_t*)data, (uint8_t*)buf, size);
    buf += size;
  }
}

void
DataBroadcast::setVersionDefaults(uint8_t* frequencyBuffer)
{
  if (!m_vehicle->isM100())
  {
    setFreqDefaults(frequencyBuffer);
  }
  else
  {
    setFreqDefaultsM100_31(frequencyBuffer);
  }
}

void
DataBroadcast::setBroadcastFreqDefaults()
{
  uint8_t frequencyBuffer[16];
  setVersionDefaults(frequencyBuffer);
  setBroadcastFreq(frequencyBuffer);
}

ACK::ErrorCode
DataBroadcast::setBroadcastFreqDefaults(int timeout)
{
  uint8_t frequencyBuffer[16];
  setVersionDefaults(frequencyBuffer);
  return setBroadcastFreq(frequencyBuffer, timeout);
}

void
DataBroadcast::setFreqDefaultsM100_31(uint8_t* freq)
{
  /* Channels definition for M100
   * 0 - Timestamp
   * 1 - Attitude Quaterniouns
   * 2 - Acceleration
   * 3 - Velocity (Ground Frame)
   * 4 - Angular Velocity (Body Frame)
   * 5 - Position
   * 6 - Magnetometer
   * 7 - RC Channels Data
   * 8 - Gimbal Data
   * 9 - Flight Status
   * 10 - Battery Level
   * 11 - Control Information
   */
  freq[0]  = FREQ_1HZ;
  freq[1]  = FREQ_10HZ;
  freq[2]  = FREQ_50HZ;
  freq[3]  = FREQ_100HZ;
  freq[4]  = FREQ_50HZ;
  freq[5]  = FREQ_10HZ;
  freq[6]  = FREQ_1HZ;
  freq[7]  = FREQ_10HZ;
  freq[8]  = FREQ_50HZ;
  freq[9]  = FREQ_100HZ;
  freq[10] = FREQ_50HZ;
  freq[11] = FREQ_10HZ;
}

void
DataBroadcast::setFreqDefaults(uint8_t* freq)
{
  /* Channels definition for A3/N3/M600
   * 0 - Timestamp
   * 1 - Attitude Quaterniouns
   * 2 - Acceleration
   * 3 - Velocity (Ground Frame)
   * 4 - Angular Velocity (Body Frame)
   * 5 - Position
   * 6 - GPS Detailed Information
   * 7 - RTK Detailed Information
   * 8 - Magnetometer
   * 9 - RC Channels Data
   * 10 - Gimbal Data
   * 11 - Flight Statusack
   * 12 - Battery Level
   * 13 - Control Information
   * 14 - Compass Data
   */
  freq[0]  = FREQ_50HZ;
  freq[1]  = FREQ_50HZ;
  freq[2]  = FREQ_50HZ;
  freq[3]  = FREQ_50HZ;
  freq[4]  = FREQ_50HZ;
  freq[5]  = FREQ_50HZ;
  freq[6]  = FREQ_0HZ; // Don't send GPS details
  freq[7]  = FREQ_0HZ; // Don't send RTK
  freq[8]  = FREQ_0HZ; // Don't send Mag
  freq[9]  = FREQ_50HZ;
  freq[10] = FREQ_50HZ;
  freq[11] = FREQ_10HZ;
  freq[12] = FREQ_1HZ;
  freq[13] = FREQ_1HZ;
  freq[14] = FREQ_1HZ;
}

void
DataBroadcast::setBroadcastFreqToZero()
{
  uint8_t freq[16];

  freq[0]  = FREQ_0HZ;
  freq[1]  = FREQ_0HZ;
  freq[2]  = FREQ_0HZ;
  freq[3]  = FREQ_0HZ;
  freq[4]  = FREQ_0HZ;
  freq[5]  = FREQ_0HZ;
  freq[6]  = FREQ_0HZ;
  freq[7]  = FREQ_0HZ;
  freq[8]  = FREQ_0HZ;
  freq[9]  = FREQ_0HZ;
  freq[10] = FREQ_0HZ;
  freq[11] = FREQ_0HZ;
  freq[12] = FREQ_0HZ;
  freq[13] = FREQ_0HZ;
  freq[14] = FREQ_0HZ;
  setBroadcastFreq(freq);
}

void
DataBroadcast::setFrequencyCallback(Vehicle* vehicle, RecvContainer recvFrame,
                                    UserData userData)
{

  ACK::ErrorCode ackErrorCode;
  ackErrorCode.data = OpenProtocolCMD::ErrorCode::CommonACK::NO_RESPONSE_ERROR;
  ackErrorCode.info = recvFrame.recvInfo;

  if (recvFrame.recvInfo.len - OpenProtocol::PackageMin <= 2)
  {
    // Two-byte ACK
    ackErrorCode.data = recvFrame.recvData.ack;
  }

  if (!ACK::getError(ackErrorCode))
  {
    ACK::getErrorCodeMessage(ackErrorCode, __func__);
  }
}

void
DataBroadcast::setUserBroadcastCallback(VehicleCallBack callback,
                                        UserData        userData)
{
  userCbHandler.callback = callback;
  userCbHandler.userData = userData;
}

uint16_t
DataBroadcast::getPassFlag()
{
  return m_passFlag;
}

uint16_t
DataBroadcast::getBroadcastLength()
{
  return this->m_broadcastLength;
}

void
DataBroadcast::setBroadcastLength(uint16_t length)
{
  this->m_broadcastLength = length;
}

void
DataBroadcast::lockMSG() {
  Platform::instance().mutexLock(m_msgLock);
}

void
DataBroadcast::freeMSG() {
  Platform::instance().mutexUnlock(m_msgLock);
}

