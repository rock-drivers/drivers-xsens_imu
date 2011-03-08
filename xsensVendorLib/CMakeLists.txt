
include_directories(${CMAKE_CURRENT_SOURCE_DIR})


ADD_LIBRARY(xsensVendorLib cmt1.cpp  
  cmt3.cpp cmtpacket.cpp xsens_exception.cpp xsens_time.cpp
  cmt2.cpp cmtmessage.cpp cmtscan.cpp xsens_std.cpp)

SET_TARGET_PROPERTIES(xsensVendorLib PROPERTIES COMPILE_FLAGS -fPIC)

TARGET_LINK_LIBRARIES(xsensVendorLib rt)
