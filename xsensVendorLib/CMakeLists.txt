
include_directories(${CMAKE_CURRENT_SOURCE_DIR})


ADD_LIBRARY(xsensVendorLib SHARED cmt1.cpp  
  cmt3.cpp cmtpacket.cpp xsens_list.cpp xsens_time.cpp
  cmt2.cpp cmtmessage.cpp cmtscan.cpp xsens_std.cpp)

TARGET_LINK_LIBRARIES(xsensVendorLib rt)
