#include <stdlib.h>
#include <iostream>
#include "XsensData.hpp"
#include "XsensDriver.hpp"
#include <iostream>

using namespace std;
using namespace xsens_imu;

XsensDriver::XsensDriver() {
    _data = new XsensData();
    _data->packet = NULL;
}

XsensDriver::~XsensDriver() {
    delete _data;
}

bool XsensDriver::open(std::string const& dev) {
    // Hardcoded Baudrate for now
    int baudrate = CMT_BAUD_RATE_115K2;
    XsensResultValue ret = _data->cmt3.openPort(dev.c_str(), baudrate);
    if(ret != XRV_OK)
    {
        cerr << "xsens: failed to find an MTi on port " << dev << " (" << xsensResultText(ret) << ")" << endl;
        return false;
    }

    // Reset alignment matrix, "just in case"
    CmtMatrix mat;
    mat.m_data[0][0] = 1.0;
    mat.m_data[0][1] = 0.0;
    mat.m_data[0][2] = 0.0;

    mat.m_data[1][0] = 0.0;
    mat.m_data[1][1] = 1.0;
    mat.m_data[1][2] = 0.0;

    mat.m_data[2][0] = 0.0;
    mat.m_data[2][1] = 0.0;
    mat.m_data[2][2] = 1.0;
    ret = _data->cmt3.setObjectAlignmentMatrix(mat);
    if (ret != XRV_OK)
    {
        cerr << "xsens: failed to set object alignment " << xsensResultText(ret) << endl;
        return false;
    }

    return true;
}

bool XsensDriver::setCalibrationMode()
{
    int cmt_output  = CMT_OUTPUTMODE_RAW;
    int cmt_options = CMT_OUTPUTSETTINGS_TIMESTAMP_SAMPLECNT;
    int sample_freq = 100;

    // Set sensor to config state
    XsensResultValue ret = _data->cmt3.gotoConfig();
    if(ret != XRV_OK)
    {
        std::cerr << "xsens: failed to go into config mode " << xsensResultText(ret) << std::endl;
        return false;
    }

    CmtDeviceMode deviceMode(cmt_output, cmt_options, sample_freq);
    deviceMode.m_outputMode &= 0xFF0F;
    ret = _data->cmt3.setDeviceMode(deviceMode, true);
    if(ret != XRV_OK)
    {
        std::cerr << "xsens: failed to set device mode " << xsensResultText(ret) << std::endl;
        return false;
    }

    // Set logging
    ret = _data->cmt3.createLogFile("xsens-imu-calib.bin", true);
    if (ret != XRV_OK)
    {
        std::cerr << "xsens: failed to enable logging " << xsensResultText(ret) << std::endl;
        return false;
    }

    char logfile_name[PATH_MAX];
    _data->cmt3.getLogFileName(logfile_name);
    std::cout << "xsens: driver is now in calibration mode\n";
    std::cout << "  output file: " << logfile_name << std::endl;

    // Go into measurement mode
    ret = _data->cmt3.gotoMeasurement();
    if (ret != XRV_OK)
    {
        std::cerr << "xsens: failed to go into measurement mode" << xsensResultText(ret) << std::endl;
        return false;
    }

    unsigned int mtCount = _data->cmt3.getMtCount();
    delete _data->packet;
    _data->packet = new xsens::Packet(mtCount, _data->cmt3.isXm());
    return true;
}

bool XsensDriver::setReadingMode(imuMode output_mode)
{
    CmtOutputMode     mode = 0;
    CmtOutputSettings settings = 0;

    switch (output_mode) {
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

    unsigned int mtCount = _data->cmt3.getMtCount();

    // set sensor to config sate
    XsensResultValue ret = _data->cmt3.gotoConfig();
    if(ret != XRV_OK) {
        std::cerr << "xsens: failed to go into config mode " << xsensResultText(ret) << std::endl;
        return false;
    }

    unsigned short sampleFreq = _data->cmt3.getSampleFrequency();

    CmtDeviceMode deviceMode(mode, settings, sampleFreq);
    deviceMode.m_outputMode &= 0xFF0F;
    ret = _data->cmt3.setDeviceMode(deviceMode, true);
    if(ret != XRV_OK) {
        std::cerr << "xsens: failed to set device mode " << xsensResultText(ret) << std::endl;
        return false;
    }

   // Go into measurement mode
    ret = _data->cmt3.gotoMeasurement();
    if (ret != XRV_OK)
    {
        std::cerr << "xsens: failed to go into measurement mode " << xsensResultText(ret) << std::endl;
        return false;
    }

    delete _data->packet;
    _data->packet = new xsens::Packet((unsigned short) mtCount, _data->cmt3.isXm());

    return true;
}

bool XsensDriver::setTimeout(const uint32_t timeout) {
  return !_data->cmt3.setTimeoutMeasurement(timeout);
}



enum xsens_imu::errorCodes XsensDriver::getReading() {

  XsensResultValue ret = _data->cmt3.waitForDataMessage(_data->packet);
  
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
      _data->caldata  = _data->packet->getCalData(0);
      _data->qat_data = _data->packet->getOriQuat(0);

      return NO_ERROR;
      break;
  }
  
  return ERROR_OTHER;
}

int XsensDriver::getPacketCounter() {
  return _data->packet->getSampleCounter();
}

Eigen::Quaternion<double> XsensDriver::getOrientation() const {
  return Eigen::Quaternion<double>(_data->qat_data.m_data[0], _data->qat_data.m_data[1], _data->qat_data.m_data[2], _data->qat_data.m_data[3]);
}

Eigen::Vector3d XsensDriver::getCalibratedAccData() const {
  return Eigen::Vector3d(_data->caldata.m_acc.m_data[0], _data->caldata.m_acc.m_data[1], _data->caldata.m_acc.m_data[2]);
}

Eigen::Vector3d XsensDriver::getCalibratedGyroData() const {
  return Eigen::Vector3d(_data->caldata.m_gyr.m_data[0], _data->caldata.m_gyr.m_data[1], _data->caldata.m_gyr.m_data[2]);
}

Eigen::Vector3d XsensDriver::getCalibratedMagData() const {
  return Eigen::Vector3d(_data->caldata.m_mag.m_data[0], _data->caldata.m_mag.m_data[1], _data->caldata.m_mag.m_data[2]);
}

