#include<stdlib.h>
#include<iostream>
#include"XsensDriver.hpp"


XsensDriver::XsensDriver() {
  packet = 0;
}


int XsensDriver::connectDevice(std::string dev, enum XsensDriver::imuMode imuMode) {
  CmtDeviceId deviceIds[256];

  //hardcoded Baudrate for now
  int baudrate = B115200;
  
  XsensResultValue ret = cmt3.openPort(dev.c_str(), baudrate);
  if(ret != XRV_OK) {
    perror("Failed to open Xsens MTi");
    return -1;
  }
  
  CmtOutputMode mode = 0;
  CmtOutputSettings settings = 0;


  switch (imuMode) {
  case ONLY_CAL_DATA:
    //configure IMU in calibrated data mode
    mode = CMT_OUTPUTMODE_CALIB;
    break;
  case ONLY_ORI_DATA:
    //configure IMU in orientation mode
    mode = CMT_OUTPUTMODE_ORIENT;
    
    //tell IMU to give us orientation as quaternion
    settings = CMT_OUTPUTSETTINGS_ORIENTMODE_QUATERNION;
    break;
  case CAL_AND_ORI_DATA:
    //configure IMU in calibrated data and orientation mode
    mode = CMT_OUTPUTMODE_CALIB | CMT_OUTPUTMODE_ORIENT;
    
    //tell IMU to give us orientation as quaternion
    settings = CMT_OUTPUTSETTINGS_ORIENTMODE_QUATERNION;
    break;
  }

  //add timestamps to data
  settings |= CMT_OUTPUTSETTINGS_TIMESTAMP_SAMPLECNT;
  
  unsigned int mtCount = cmt3.getMtCount();
	
  // set sensor to config sate
  ret = cmt3.gotoConfig();
  if(ret != XRV_OK) {
    perror("Failed to set Xsens MTi into Config mode");
    return -2;
  }

  //get device ids
  for(unsigned int j = 0; j < mtCount; j++){
    ret = cmt3.getDeviceId((unsigned char)(j+1), deviceIds[j]);
    if(ret != XRV_OK) {
      perror("Failed to get Xsens MTi device ID");
      return -3;
    }
    
    printf("Device ID at busId %i: %08lx\n\n",j+1,(long) deviceIds[j]);
  }

  unsigned short sampleFreq;
  sampleFreq = cmt3.getSampleFrequency();

  for (unsigned int i = 0; i < mtCount; i++) {
    CmtDeviceMode deviceMode(mode, settings, sampleFreq);
    if ((deviceIds[i] & 0xFFF00000) != 0x00500000) {
      // not an MTi-G, remove all GPS related stuff
      deviceMode.m_outputMode &= 0xFF0F;
    }
    ret = cmt3.setDeviceMode(deviceMode, true, deviceIds[i]);
    if(ret != XRV_OK) {
      perror("Failed to set Xsens MTi into devicemode");
      return -4;
    }
  }

  // tell IMU to send us data
  ret = cmt3.gotoMeasurement();
  if(ret != XRV_OK) {
      perror("Failed to get Xsens MTi Measurements");
      return -5;
  }

  if(packet) {
    delete packet;
    packet = 0;
  }
  
  packet = new xsens::Packet((unsigned short) mtCount, cmt3.isXm());

  return 0;
}

bool XsensDriver::setTimeout(const uint32_t timeout) {
  return !cmt3.setTimeoutMeasurement(timeout);
}



enum XsensDriver::errorCodes XsensDriver::getReading() {

  XsensResultValue ret = cmt3.waitForDataMessage(packet);
  
  switch(ret) {
    case XRV_TIMEOUT:
    case XRV_TIMEOUTNODATA:
      //timeout, no data available
      return ERROR_TIMEOUT;
      break;

    default:
      //error occured
      return ERROR_OTHER;
      break;
      
    case XRV_OK:
      caldata = packet->getCalData(0);
      qat_data = packet->getOriQuat(0);

      return NO_ERROR;
      break;
  }
  
  return ERROR_OTHER;
}

int XsensDriver::getPacketCounter() {
  return packet->getSampleCounter();
}

Eigen::Quaternion<double> XsensDriver::getOrientation() const {
  return Eigen::Quaternion<double>(qat_data.m_data[0], qat_data.m_data[1], qat_data.m_data[2], qat_data.m_data[3]);
}

Eigen::Vector3d XsensDriver::getCalibratedAccData() const {
  return Eigen::Vector3d(caldata.m_acc.m_data[0], caldata.m_acc.m_data[1], caldata.m_acc.m_data[2]);
}

Eigen::Vector3d XsensDriver::getCalibratedGyroData() const {
  return Eigen::Vector3d(caldata.m_gyr.m_data[0], caldata.m_gyr.m_data[1], caldata.m_gyr.m_data[2]);
}

Eigen::Vector3d XsensDriver::getCalibratedMagData() const {
  return Eigen::Vector3d(caldata.m_mag.m_data[0], caldata.m_mag.m_data[1], caldata.m_mag.m_data[2]);
}



