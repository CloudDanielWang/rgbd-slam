#add_library( myslam SHARED
#    frame.cpp
#    mappoint.cpp
#    map.cpp
#    camera.cpp
#    config.cpp
#    g2o_types.cpp
#    visual_odometry.cpp
#    cloudmap.cpp
#)

#target_link_libraries( myslam
#    ${THIRD_PARTY_LIBS}
#)

#之后此处需要根据用途分类进行链接，整理
#add_library(wifi wifi.cpp
#    frame.cpp
#    mappoint.cpp
#    map.cpp
#    camera.cpp
#    config.cpp
#    g2o_types.cpp
#    visual_odometry.cpp
#    cloudmap.cpp
#    converter.cpp
#    data.cpp
#    viz.cpp
#)


add_library(front_end 
    frame.cpp
    mappoint.cpp
    map.cpp
    camera.cpp
    config.cpp
    g2o_types.cpp
    visual_odometry.cpp
    converter.cpp
    data.cpp
    timer.cpp
    gps.cpp
    imu.cpp
)

target_link_libraries( front_end 
    ${THIRD_PARTY_LIBS}
    rt
)
