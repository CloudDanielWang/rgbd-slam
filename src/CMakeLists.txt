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
add_library(wifi wifi.cpp
    frame.cpp
    mappoint.cpp
    map.cpp
    camera.cpp
    config.cpp
    g2o_types.cpp
    visual_odometry.cpp
    cloudmap.cpp
    converter.cpp
    data.cpp
)

target_link_libraries( wifi
    ${THIRD_PARTY_LIBS}
)
