#add_executable( run_vo run_vo.cpp )
#target_link_libraries( run_vo myslam )

add_executable( wifi_test wifitest.cpp )
target_link_libraries( wifi_test wifi  )
