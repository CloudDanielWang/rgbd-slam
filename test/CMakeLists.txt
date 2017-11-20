#add_executable( run_vo run_vo.cpp )
#target_link_libraries( run_vo myslam )

#add_executable( wifi_test wifitest.cpp )
#target_link_libraries( wifi_test wifi  )

add_executable( slam_front slam_front.cpp )
target_link_libraries( slam_front wifi )

add_executable( wifi_receive wifi_receive.cpp )
target_link_libraries( wifi_receive wifi )

#add_executable( uav_position uav_position.cpp )
#target_link_libraries( uav_position front_end )
