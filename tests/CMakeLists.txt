set(ICP_TEST_DATA_DIR "${CMAKE_CURRENT_SOURCE_DIR}/data")

# --- Helper: register one test case from an executable ---
function(icp_add_test TEST_NAME EXECUTABLE)
    add_test(NAME ${TEST_NAME} COMMAND ${EXECUTABLE} ${TEST_NAME})
    set_tests_properties(${TEST_NAME} PROPERTIES
        ENVIRONMENT "ICP_TEST_DATA_DIR=${ICP_TEST_DATA_DIR}"
    )
    # Forward extra args as labels
    if(ARGN)
        set_tests_properties(${TEST_NAME} PROPERTIES LABELS "${ARGN}")
    endif()
endfunction()

# ─── test_icp: core algorithm tests ─────────────────────────────────────
add_executable(test_icp test_icp.cpp)
target_link_libraries(test_icp icp)

icp_add_test(identity                   test_icp  "rigid")
icp_add_test(pure_translation           test_icp  "rigid")
icp_add_test(pure_rotation              test_icp  "rigid")
icp_add_test(rotation_and_translation   test_icp  "rigid")
icp_add_test(convergence_error          test_icp  "rigid")
icp_add_test(translation_only           test_icp  "settings")
icp_add_test(with_scaling               test_icp  "settings")
icp_add_test(no_rotation_no_translation test_icp  "settings")

# --- method tests ---
icp_add_test(point_to_plane                   test_icp  "method")
icp_add_test(plane_to_plane                   test_icp  "method")
icp_add_test(point_to_plane_vs_point_to_point test_icp  "method")

# --- kdtree tests ---
icp_add_test(kdtree_translation         test_icp  "kdtree")
icp_add_test(kdtree_rotation            test_icp  "kdtree")
icp_add_test(kdtree_vs_bruteforce       test_icp  "kdtree")

# ─── test_pointcloud_io: file I/O tests ─────────────────────────────────
add_executable(test_pointcloud_io test_pointcloud_io.cpp)
target_link_libraries(test_pointcloud_io icp)

icp_add_test(load              test_pointcloud_io  "fileio")
icp_add_test(load_translated   test_pointcloud_io  "fileio")
icp_add_test(roundtrip         test_pointcloud_io  "fileio")
icp_add_test(load_missing_file test_pointcloud_io  "fileio")

# ─── test_file_icp: end-to-end with point cloud files ───────────────────
add_executable(test_file_icp test_file_icp.cpp)
target_link_libraries(test_file_icp icp)

icp_add_test(file_translation test_file_icp "integration")
