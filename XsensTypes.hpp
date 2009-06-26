#ifndef __XSENSTYPES_H__
#define __XSENSTYPES_H__

namespace xsens_imu {

    enum errorCodes {
        NO_ERROR = 0,
        ERROR_TIMEOUT = 1,
        ERROR_OTHER = 2
    };

    enum imuMode {
        ONLY_CAL_DATA,
        ONLY_ORI_DATA,
        CAL_AND_ORI_DATA
    };
}

#endif
