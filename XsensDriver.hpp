#ifndef __XSENSDRIVER_H__
#define __XSENSDRIVER_H__

#include <string>
#include <Eigen/Geometry> 
#include <stdint.h>
#include <list>
#include <map>

#include "XsensTypes.hpp"

namespace xsens_imu {

    class XsensData;

    class XsensDriver {
        public:
            static const int DEFAULT_SAMPLE_FREQUENCY = 100;

            XsensDriver();
            ~XsensDriver();

            bool open(std::string const& dev);
            bool close();

            bool setReadingMode(enum xsens_imu::imuMode = ONLY_ORI_DATA);
            bool setCalibrationMode();
            int getFileHandle();

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
             * This function acquires a new sensor reading 
             * from the Xsens IMU.
             *
             * Return Values
             *  0 Data Aquired
             * -1 Error_Again e.g. no reading there yet
             * -2 Error fetching data from IMU
             **/
            enum xsens_imu::errorCodes getReadingNonBlocking();

            /**
             * Sets the timout for reading measurements
             * from the IMU. 
             * Parameter is in ms
             * returns true on success
             **/
            bool setTimeout(const uint32_t timeout);

            /**
             * Sets the sampling frequency for reading measurements
             * from the IMU.
             * Parameter is in Hz
             * Note: Method has to be called BEFORE setReadingMode() to apply new frequency value.
             **/
            void setFrequency(const uint16_t new_sample_frequency);

            /**
             * Gets the sampling frequency of IMU.
             * Returns frequency in Hz.
             * Note: Method has to be called BEFORE setReadingMode() to apply new frequency value.
             **/
            uint16_t getFrequency(void);

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
             * Returns the current orientation as Euler Angels (aerospace convention: Roll, Pitch, Yaw).
             * Returned angles given as Tait–Bryan convention with x-Axis as heading direction:
             *   Rotation around x-Axis in YZ-Plane: Roll (Angle Phi)
             *   Rotation around y-Axis in XZ-Plane: Pitch (Angle Theta)
             *   Rotation around z-Axis in XY Plane: Yaw (Angle Psi)
             *
             * Note getReading() should be called before calling this function
             **/
            Eigen::Vector3d getEulerAngles() const;

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

            /**
             * Returns the raw accelerometer data
             * Note getReading() should be called before calling this function
             **/
            Eigen::Vector3d getRawAccData() const;

            /**
             * Returns the raw gyroscope data
             * Note getReading() should be called before calling this function
             **/
            Eigen::Vector3d getRawGyroData() const;

            /**
             * Returns the raw magnetometer data
             * Note getReading() should be called before calling this function
             **/
            Eigen::Vector3d getRawMagData() const;

	    /**
	     * Returns the list of scenario names that are available on this device
	     */
	    std::list<std::string> getScenarios() const;

	    std::list<std::string> getAvailableScenarios();
	    bool setScenario(std::string const& name);

	    /**
	     * Configures the SyncOut pin. Must be done before setReadingMode().
	     * enable: selects enabled or disabled
	     * pulse: when true, a pulse is emitted on sensor acquisition,
	     * otherwise the level of the line is toggled.
	     * positive: direction of the pulse
	     */
	    bool setSyncOut(bool enable, bool pulse, bool positive);

        private:
            XsensData* _data;
            std::map< std::string, int > m_scenarios;
            uint16_t last_samplectr;
            int m_timeout;
            int samplectr_offset;
            unsigned short _sample_freq;
   };
}

#endif
