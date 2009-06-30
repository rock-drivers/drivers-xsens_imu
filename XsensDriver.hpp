#include "cmtdef.h"
#include "xsens_time.h"
#include "xsens_list.h"
#include "cmtscan.h"
#include "cmt3.h"
#include<string>
#include <Eigen/Geometry> 

#include "XsensTypes.hpp"

namespace xsens_imu {

    class XsensDriver {
        public:

            XsensDriver();

            bool open(std::string const& dev);

            bool setReadingMode(enum xsens_imu::imuMode = ONLY_ORI_DATA);
            bool setCalibrationMode();

            /**
             * This function acquires a new sensor reading 
             * from the Xsens IMU.
             *
             * Return Values
             *  0 Data Aquired
             * -1 Timeout
             * -2 Error fetching data from IMU
             **/
            enum xsens_imu::errorCodes getReading();

            /**
             * Sets the timout for reading measurements
             * from the IMU. 
             * Parameter is in ms
             * returns true on success
             **/
            bool setTimeout(const uint32_t timeout);

            /**
             * Returns the packt count of the last received
             * packet from the IMU
             **/
            int getPacketCounter();


            /**
             * Returns the current orientation as quaternion.
             * Note getReading() should be called before calling this function
             **/
            Eigen::Quaternion<double> getOrientation() const;

            /**
             * Returns the calibrated accelerometer data
             * Note getReading() should be called before calling this function
             **/
            Eigen::Vector3d getCalibratedAccData() const;


            /**
             * Returns the calibrated gyroscope data
             * Note getReading() should be called before calling this function
             **/
            Eigen::Vector3d getCalibratedGyroData() const;


            /**
             * Returns the calibrated magnetometer data
             * Note getReading() should be called before calling this function
             **/
            Eigen::Vector3d getCalibratedMagData() const;


        private:
            xsens::Cmt3 cmt3;
            xsens::Packet* packet;
            CmtCalData caldata;
            CmtQuat qat_data;
    };
}
