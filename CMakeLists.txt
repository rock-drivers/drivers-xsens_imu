
PROJECT(xsense)

cmake_minimum_required(VERSION 2.6)

#add definitions, compiler switches, etc.
ADD_DEFINITIONS(-Wall)

#list all source files here
#ADD_EXECUTABLE(driver main.cpp interface.cpp)
find_package(PkgConfig)

pkg_check_modules(DFKI_TYPES REQUIRED "dfki-types")
INCLUDE_DIRECTORIES(${DFKI_TYPES_INCLUDE_DIRS})
LINK_DIRECTORIES(${DFKI_TYPES_LIBRARY_DIRS})

ADD_LIBRARY(xsens SHARED XsensDriver.cpp)
TARGET_LINK_LIBRARIES(xsens ${DFKI_TYPES_LIBS})

#need to link to some other libraries ? just add them here
#TARGET_LINK_LIBRARIES(driver png jpeg)
 
CONFIGURE_FILE(Doxyfile.in Doxyfile @ONLY)
ADD_CUSTOM_TARGET(doc doxygen Doxyfile)
INSTALL(TARGETS xsens
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib)
INSTALL(FILES XsensDriver.hpp DESTINATION include)

CONFIGURE_FILE(xsens.pc.in xsens.pc @ONLY)
INSTALL(FILES ${CMAKE_BINARY_DIR}/xsens.pc DESTINATION lib/pkgconfig)



