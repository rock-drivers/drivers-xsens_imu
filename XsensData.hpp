#ifndef __XSENSDATA_H__
#define __XSENSDATA_H__

#include "cmtdef.h"
#include "xsens_time.h"
#include "xsens_list.h"
#include "cmtscan.h"
#include "cmt3.h"

namespace xsens_imu {

    struct XsensData {
            xsens::Cmt3 cmt3;
            xsens::Packet* packet;
            CmtCalData caldata;
            CmtRawData rawdata;
            CmtQuat qat_data;
            CmtEuler eulerdata;
    };
}

#endif
