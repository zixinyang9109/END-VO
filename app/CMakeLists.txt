#add_executable(run_kitti_stereo run_kitti_stereo.cpp)
#target_link_libraries(run_kitti_stereo myslam ${THIRD_PARTY_LIBS})

add_executable(run_end_stereo run_end_stereo.cpp)
target_link_libraries(run_end_stereo myslam ${THIRD_PARTY_LIBS})
