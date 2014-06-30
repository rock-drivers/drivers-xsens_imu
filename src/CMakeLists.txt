include_directories(BEFORE ${PROJECT_SOURCE_DIR}/external/xsensVendorLib)

rock_library(xsens_imu
    SOURCES XsensDriver.cpp
    HEADERS XsensData.hpp XsensTypes.hpp XsensDriver.hpp
    DEPS_PKGCONFIG base-types eigen3 )

target_link_libraries(xsens_imu xsensVendorLib) 

rock_executable(test_xsen_imu XsensTest.cpp
    DEPS xsens_imu
    DEPS_PKGCONFIG base-types
    DEPS_CMAKE Boost)  