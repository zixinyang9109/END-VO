SET(TEST_SOURCES test_triangulation)

FOREACH (test_src ${TEST_SOURCES})
    ADD_EXECUTABLE(${test_src} ${test_src}.cpp test_Hello.cpp)
    TARGET_LINK_LIBRARIES(${test_src} ${THIRD_PARTY_LIBS} myslam)
    ADD_TEST(${test_src} ${test_src})
ENDFOREACH (test_src)

## test detector and cv functions
add_executable(test_Hello test_Hello.cpp)
# link executable to libs
target_link_libraries(test_Hello myslam)


add_executable(test_depth test_depth.cpp)
target_link_libraries(test_depth myslam)

add_executable(test_photo photometric_checking.cpp)
target_link_libraries(test_photo myslam)

add_executable(test_prior optical_flow_with_depth_prior.cpp)
target_link_libraries(test_prior myslam)

add_executable(filter filter_points.cpp)
target_link_libraries(filter myslam)

add_executable(first first_module.cpp)
target_link_libraries(first myslam)

add_executable(test_read test_read_depth.cpp)
target_link_libraries(test_read myslam)

add_executable(clean_prior clean_prior.cpp)
target_link_libraries(clean_prior myslam)

add_executable(test_dense_optical_flow test_dense_optical_flow.cpp)
target_link_libraries(test_dense_optical_flow myslam)

add_executable(test_keyframe test_keyframe.cpp)
target_link_libraries(test_keyframe myslam)

add_executable(test_visualization test_visualization.cpp)
target_link_libraries(test_visualization myslam)