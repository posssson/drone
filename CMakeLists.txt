# %Tag(FULLTEXT)%
cmake_minimum_required(VERSION 2.8.3)
project(drone)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS
 roscpp
 std_msgs
 message_generation
 genmsg
 roslaunch
 )

## Declare ROS messages and services
add_message_files(
FILES
Capteurs_msg.msg
Consigne_msg.msg
Clavier_msg.msg
Optim_msg.msg
Erreur_angle_msg.msg
Altitude_msg.msg
Gaz_msg.msg
Data_msg.msg
)
#add_service_files(FILES AddTwoInts.srv)

## Generate added messages and services
generate_messages(
 DEPENDENCIES
 std_msgs
)
## Declare a catkin package
catkin_package()

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})
include_directories(/home/pi/wiringPi)

add_executable(Servos src/Servos/Servos.cpp src/Common/Realtime.cpp)
target_link_libraries(Servos ${catkin_LIBRARIES})
add_dependencies(Servos drone_generate_messages_cpp)


add_executable(Clavier src/Clavier/Clavier.cpp)
target_link_libraries(Clavier ${catkin_LIBRARIES})
add_dependencies(Clavier drone_generate_messages_cpp)

add_executable(Init src/Init/Init.cpp)
target_link_libraries(Init ${catkin_LIBRARIES})
add_dependencies(Init drone_generate_messages_cpp)

add_executable(Ecrit src/Ecriture/Ecrit.cpp)
target_link_libraries(Ecrit ${catkin_LIBRARIES})
add_dependencies(Ecrit drone_generate_messages_cpp)

add_executable(Capteurs src/Capteurs/Capteurs.cpp src/Common/Biquad.cpp src/Common/Realtime.cpp src/Capteurs/I2Cdev.cpp src/Common/Biquad.cpp src/Common/LowPassFilter2p.cpp src/Capteurs/MPU9250.cpp src/Capteurs/MPU9250pio.cpp src/Capteurs/MadgwickAHRS.cpp)
target_link_libraries(Capteurs ${catkin_LIBRARIES})
add_dependencies(Capteurs drone_generate_messages_cpp)

add_executable(Optim src/Optim/Optimisation.cpp)
target_link_libraries(Optim ${catkin_LIBRARIES})
add_dependencies(Optim drone_generate_messages_cpp)

add_executable(Altitude src/Altitude/Altitude.cpp src/Altitude/bme280.cpp src/Altitude/I2Cdev.cpp src/Altitude/libSonar.cpp)
target_link_libraries(Altitude ${catkin_LIBRARIES})
add_dependencies(Altitude drone_generate_messages_cpp)

add_executable(Gps src/Gps/Gps.cpp)
target_link_libraries(Gps ${catkin_LIBRARIES})
add_dependencies(Gps drone_generate_messages_cpp)

target_link_libraries (Servos wiringPi pigpio)
target_link_libraries (Capteurs wiringPi pigpio)
target_link_libraries (Optim nlopt)
target_link_libraries (Gps wiringPi)
target_link_libraries (Altitude wiringPi)



include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std = c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std = c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
 set(CMAKE_CXX_FLAGS "-std = c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
 set(CMAKE_CXX_FLAGS "-std = c++0x")
else()
 message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler. Suggested solution: update the pkg build-essential ")
endif()



# %EndTag(FULLTEXT)%
