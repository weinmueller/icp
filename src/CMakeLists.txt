add_library(icp icp.cpp pointcloud_io.cpp)
target_include_directories(icp PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(icp PUBLIC Eigen3::Eigen)
