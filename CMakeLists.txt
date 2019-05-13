cmake_minimum_required(VERSION 2.8.3)
project(data_capture)

find_package(catkin REQUIRED COMPONENTS actionlib std_msgs message_generation)
add_service_files(FILES RequestSaveData.srv SetBatteryLevel.srv)
generate_messages(DEPENDENCIES std_msgs actionlib_msgs)