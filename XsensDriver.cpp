#include <stdlib.h>
#include <iostream>
#include "XsensData.hpp"
#include "XsensDriver.hpp"
#include <iostream>
#include <stdexcept>

using namespace std;
using namespace xsens_imu;

XsensDriver::XsensDriver()
	: last_samplectr(0)
	, samplectr_offset(0)
{
    _data = new XsensData();
    _data->packet = NULL;

    _sample_freq = DEFAULT_SAMPLE_FREQUENCY; // initialise with default frequency
}

XsensDriver::~XsensDriver() {
    delete _data;
}

bool XsensDriver::close() {
    XsensResultValue ret = _data->cmt3.closePort(true);
    return ret == XRV_OK;
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

int XsensDriver::getFileHandle()
{
    int fd = -1;
    if( _data && _data->cmt3.getCmt2s() && _data->cmt3.getCmt2s()->getCmt1s() )
        fd = _data->cmt3.getCmt2s()->getCmt1s()->getHandle();
    return fd;
}

bool XsensDriver::setCalibrationMode()
{
    int cmt_output  = CMT_OUTPUTMODE_RAW;
    int cmt_options = CMT_OUTPUTSETTINGS_TIMESTAMP_SAMPLECNT;
    int sample_freq = XsensDriver::_sample_freq;

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

        case ORI_EULER_DATA:
            //configure IMU in orientation mode
            mode = CMT_OUTPUTMODE_ORIENT;
            //tell IMU to give us orientation as Euler angels
            settings = CMT_OUTPUTSETTINGS_ORIENTMODE_EULER;
            break;

        case CAL_AND_ORI_DATA:
            //configure IMU in calibrated data and orientation mode
            mode = CMT_OUTPUTMODE_CALIB | CMT_OUTPUTMODE_ORIENT;

            //tell IMU to give us orientation as quaternion
            settings = CMT_OUTPUTSETTINGS_ORIENTMODE_QUATERNION;
            break;
        case RAW_DATA:
            //configure IMU in raw data mode
            //cannot be combined with calibrated data mode
            mode = CMT_OUTPUTMODE_RAW;

            //tell IMU to give us orientation as quaternion
            settings = 0;
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

    // use configurable "_sample_freq" instead of static frequency
    CmtDeviceMode deviceMode(mode, settings, _sample_freq);
    deviceMode.m_outputMode &= 0xFF0F;
    ret = _data->cmt3.setDeviceMode(deviceMode, true, CMT_DID_BROADCAST); //

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
    m_timeout = timeout;
    return !_data->cmt3.setTimeoutMeasurement(timeout);
}

void XsensDriver::setFrequency(const uint16_t new_sample_frequency) {
	_sample_freq = new_sample_frequency;
}

uint16_t XsensDriver::getFrequency(void) {
	return _data->cmt3.getSampleFrequency();
}

std::list<std::string> XsensDriver::getAvailableScenarios() {
  std::list<std::string> result;
  CmtScenario available_scenarios[CMT_MAX_SCENARIOS + 1];

  XsensResultValue ret = _data->cmt3.gotoConfig();
  if (ret != XRV_OK)
    throw std::runtime_error("cannot go into config");
  _data->cmt3.getAvailableScenarios(available_scenarios);
  ret = _data->cmt3.gotoMeasurement();
  if (ret != XRV_OK)
    throw std::runtime_error("cannot go into measurement");
  
  m_scenarios.clear();
  for (int i = 0; i < CMT_MAX_SCENARIOS; ++i)
  {
    if (available_scenarios[i].m_type == 0) break;
    std::string name(available_scenarios[i].m_label);
    int space = name.find_first_of(" ");
    name.erase(space);
    m_scenarios[name] = available_scenarios[i].m_type;
    result.push_back(name);

    std::cerr << name << "=" << (int)available_scenarios[i].m_type << std::endl;
  }
  return result;
}

bool XsensDriver::setScenario(std::string const& name) {
  if (m_scenarios.empty())
    getAvailableScenarios();

  XsensResultValue ret = _data->cmt3.gotoConfig();
  if (ret != XRV_OK)
    return false;

  map<string, int>::const_iterator type = m_scenarios.find(name);
  if (type == m_scenarios.end())
  {
    std::cerr << "this Xsens has no scenario called '" << name << "'" << std::endl;
    return false;
  }

  std::cerr << "setting ID: " << type->second << std::endl;
  if (_data->cmt3.setScenario(type->second) != XRV_OK);
  {
    std::cerr << "failed to switch the scenario to " << name << std::endl;
    return false;
  }

  if (_data->cmt3.gotoMeasurement() != XRV_OK)
  {
    std::cerr << "failed to switch back to measurement mode" << std::endl;
    return false;
  }
  return true;
}

bool XsensDriver::setSyncOut(bool enable, bool pulse, bool positive) {
    CmtSyncOutSettings syncOutSettings;
    if (enable) {
	if (pulse) {
	    if (positive)
		syncOutSettings = CMT_SYNCOUT_TYPE_PULSE
		    | CMT_SYNCOUT_POL_POS;
	    else
		syncOutSettings = CMT_SYNCOUT_TYPE_PULSE
		    | CMT_SYNCOUT_POL_NEG;
	} else
	    syncOutSettings = CMT_SYNCOUT_TYPE_TOGGLE;
    } else
	syncOutSettings = CMT_SYNCOUT_DISABLED;
    XsensResultValue ret = _data->cmt3.setSyncOutSettings(syncOutSettings);
    if(ret != XRV_OK) {
        std::cerr << "xsens: failed to set sync out settings " << xsensResultText(ret) << std::endl;
        return false;
    }
    return true;
}

enum xsens_imu::errorCodes XsensDriver::getReading() {
    
    timeval timeout_spec = { m_timeout / 1000, (m_timeout % 1000) * 1000 };
    int fd = getFileHandle();
    
    while(1)
    {
        enum xsens_imu::errorCodes status = getReadingNonBlocking();

        if(status != ERROR_AGAIN)
            return status;

        fd_set set;
        FD_ZERO(&set);
        FD_SET(fd, &set);

        int ret = select(fd + 1, &set, NULL, NULL, &timeout_spec);
        if (ret < 0)
            return ERROR_OTHER;
        else if (ret == 0)
            return ERROR_TIMEOUT;        
    }

    return ERROR_OTHER;
}

enum xsens_imu::errorCodes XsensDriver::getReadingNonBlocking() {

  XsensResultValue ret = _data->cmt3.readDataPacket(_data->packet, true);
  
  switch(ret) {
    case XRV_TIMEOUT:
    case XRV_TIMEOUTNODATA:
      //timeout, no data available
      return ERROR_AGAIN;
      break;

    default:
      //error occured
      return ERROR_OTHER;
      break;
      
    case XRV_OK:
      _data->caldata  = _data->packet->getCalData(0);
      _data->rawdata  = _data->packet->getRawData(0);
      _data->qat_data = _data->packet->getOriQuat(0);
      _data->eulerdata = _data->packet->getOriEuler(0);

      return NO_ERROR;
      break;
  }
  
  return ERROR_OTHER;
}

int XsensDriver::getPacketCounter() {
  uint16_t ctr = _data->packet->getSampleCounter();
  if (ctr < last_samplectr) {
    samplectr_offset += 65536;
  }
  last_samplectr = ctr;
  return samplectr_offset + ctr;
}

Eigen::Quaternion<double> XsensDriver::getOrientation() const {
  // Eigen Quaternion format: w, x, y, z
  return Eigen::Quaternion<double>(_data->qat_data.m_data[0], _data->qat_data.m_data[1], _data->qat_data.m_data[2], _data->qat_data.m_data[3]);
}

Eigen::Vector3d XsensDriver::getEulerAngles() const {
  return Eigen::Vector3d(_data->eulerdata.m_roll, _data->eulerdata.m_pitch, _data->eulerdata.m_yaw);
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

Eigen::Vector3d XsensDriver::getRawAccData() const {
  return Eigen::Vector3d(_data->rawdata.m_acc.m_data[0], _data->rawdata.m_acc.m_data[1], _data->rawdata.m_acc.m_data[2]);
}

Eigen::Vector3d XsensDriver::getRawGyroData() const {
  return Eigen::Vector3d(_data->rawdata.m_gyr.m_data[0], _data->rawdata.m_gyr.m_data[1], _data->rawdata.m_gyr.m_data[2]);
}

Eigen::Vector3d XsensDriver::getRawMagData() const {
  return Eigen::Vector3d(_data->rawdata.m_mag.m_data[0], _data->rawdata.m_mag.m_data[1], _data->rawdata.m_mag.m_data[2]);
}


