# CMake generated Testfile for 
# Source directory: /home/pascal/Coding/claude/icp/tests
# Build directory: /home/pascal/Coding/claude/icp/build/tests
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(identity "/home/pascal/Coding/claude/icp/build/tests/test_icp" "identity")
set_tests_properties(identity PROPERTIES  ENVIRONMENT "ICP_TEST_DATA_DIR=/home/pascal/Coding/claude/icp/tests/data" LABELS "rigid" _BACKTRACE_TRIPLES "/home/pascal/Coding/claude/icp/tests/CMakeLists.txt;5;add_test;/home/pascal/Coding/claude/icp/tests/CMakeLists.txt;19;icp_add_test;/home/pascal/Coding/claude/icp/tests/CMakeLists.txt;0;")
add_test(pure_translation "/home/pascal/Coding/claude/icp/build/tests/test_icp" "pure_translation")
set_tests_properties(pure_translation PROPERTIES  ENVIRONMENT "ICP_TEST_DATA_DIR=/home/pascal/Coding/claude/icp/tests/data" LABELS "rigid" _BACKTRACE_TRIPLES "/home/pascal/Coding/claude/icp/tests/CMakeLists.txt;5;add_test;/home/pascal/Coding/claude/icp/tests/CMakeLists.txt;20;icp_add_test;/home/pascal/Coding/claude/icp/tests/CMakeLists.txt;0;")
add_test(pure_rotation "/home/pascal/Coding/claude/icp/build/tests/test_icp" "pure_rotation")
set_tests_properties(pure_rotation PROPERTIES  ENVIRONMENT "ICP_TEST_DATA_DIR=/home/pascal/Coding/claude/icp/tests/data" LABELS "rigid" _BACKTRACE_TRIPLES "/home/pascal/Coding/claude/icp/tests/CMakeLists.txt;5;add_test;/home/pascal/Coding/claude/icp/tests/CMakeLists.txt;21;icp_add_test;/home/pascal/Coding/claude/icp/tests/CMakeLists.txt;0;")
add_test(rotation_and_translation "/home/pascal/Coding/claude/icp/build/tests/test_icp" "rotation_and_translation")
set_tests_properties(rotation_and_translation PROPERTIES  ENVIRONMENT "ICP_TEST_DATA_DIR=/home/pascal/Coding/claude/icp/tests/data" LABELS "rigid" _BACKTRACE_TRIPLES "/home/pascal/Coding/claude/icp/tests/CMakeLists.txt;5;add_test;/home/pascal/Coding/claude/icp/tests/CMakeLists.txt;22;icp_add_test;/home/pascal/Coding/claude/icp/tests/CMakeLists.txt;0;")
add_test(convergence_error "/home/pascal/Coding/claude/icp/build/tests/test_icp" "convergence_error")
set_tests_properties(convergence_error PROPERTIES  ENVIRONMENT "ICP_TEST_DATA_DIR=/home/pascal/Coding/claude/icp/tests/data" LABELS "rigid" _BACKTRACE_TRIPLES "/home/pascal/Coding/claude/icp/tests/CMakeLists.txt;5;add_test;/home/pascal/Coding/claude/icp/tests/CMakeLists.txt;23;icp_add_test;/home/pascal/Coding/claude/icp/tests/CMakeLists.txt;0;")
add_test(translation_only "/home/pascal/Coding/claude/icp/build/tests/test_icp" "translation_only")
set_tests_properties(translation_only PROPERTIES  ENVIRONMENT "ICP_TEST_DATA_DIR=/home/pascal/Coding/claude/icp/tests/data" LABELS "settings" _BACKTRACE_TRIPLES "/home/pascal/Coding/claude/icp/tests/CMakeLists.txt;5;add_test;/home/pascal/Coding/claude/icp/tests/CMakeLists.txt;24;icp_add_test;/home/pascal/Coding/claude/icp/tests/CMakeLists.txt;0;")
add_test(with_scaling "/home/pascal/Coding/claude/icp/build/tests/test_icp" "with_scaling")
set_tests_properties(with_scaling PROPERTIES  ENVIRONMENT "ICP_TEST_DATA_DIR=/home/pascal/Coding/claude/icp/tests/data" LABELS "settings" _BACKTRACE_TRIPLES "/home/pascal/Coding/claude/icp/tests/CMakeLists.txt;5;add_test;/home/pascal/Coding/claude/icp/tests/CMakeLists.txt;25;icp_add_test;/home/pascal/Coding/claude/icp/tests/CMakeLists.txt;0;")
add_test(no_rotation_no_translation "/home/pascal/Coding/claude/icp/build/tests/test_icp" "no_rotation_no_translation")
set_tests_properties(no_rotation_no_translation PROPERTIES  ENVIRONMENT "ICP_TEST_DATA_DIR=/home/pascal/Coding/claude/icp/tests/data" LABELS "settings" _BACKTRACE_TRIPLES "/home/pascal/Coding/claude/icp/tests/CMakeLists.txt;5;add_test;/home/pascal/Coding/claude/icp/tests/CMakeLists.txt;26;icp_add_test;/home/pascal/Coding/claude/icp/tests/CMakeLists.txt;0;")
add_test(load "/home/pascal/Coding/claude/icp/build/tests/test_pointcloud_io" "load")
set_tests_properties(load PROPERTIES  ENVIRONMENT "ICP_TEST_DATA_DIR=/home/pascal/Coding/claude/icp/tests/data" LABELS "fileio" _BACKTRACE_TRIPLES "/home/pascal/Coding/claude/icp/tests/CMakeLists.txt;5;add_test;/home/pascal/Coding/claude/icp/tests/CMakeLists.txt;32;icp_add_test;/home/pascal/Coding/claude/icp/tests/CMakeLists.txt;0;")
add_test(load_translated "/home/pascal/Coding/claude/icp/build/tests/test_pointcloud_io" "load_translated")
set_tests_properties(load_translated PROPERTIES  ENVIRONMENT "ICP_TEST_DATA_DIR=/home/pascal/Coding/claude/icp/tests/data" LABELS "fileio" _BACKTRACE_TRIPLES "/home/pascal/Coding/claude/icp/tests/CMakeLists.txt;5;add_test;/home/pascal/Coding/claude/icp/tests/CMakeLists.txt;33;icp_add_test;/home/pascal/Coding/claude/icp/tests/CMakeLists.txt;0;")
add_test(roundtrip "/home/pascal/Coding/claude/icp/build/tests/test_pointcloud_io" "roundtrip")
set_tests_properties(roundtrip PROPERTIES  ENVIRONMENT "ICP_TEST_DATA_DIR=/home/pascal/Coding/claude/icp/tests/data" LABELS "fileio" _BACKTRACE_TRIPLES "/home/pascal/Coding/claude/icp/tests/CMakeLists.txt;5;add_test;/home/pascal/Coding/claude/icp/tests/CMakeLists.txt;34;icp_add_test;/home/pascal/Coding/claude/icp/tests/CMakeLists.txt;0;")
add_test(load_missing_file "/home/pascal/Coding/claude/icp/build/tests/test_pointcloud_io" "load_missing_file")
set_tests_properties(load_missing_file PROPERTIES  ENVIRONMENT "ICP_TEST_DATA_DIR=/home/pascal/Coding/claude/icp/tests/data" LABELS "fileio" _BACKTRACE_TRIPLES "/home/pascal/Coding/claude/icp/tests/CMakeLists.txt;5;add_test;/home/pascal/Coding/claude/icp/tests/CMakeLists.txt;35;icp_add_test;/home/pascal/Coding/claude/icp/tests/CMakeLists.txt;0;")
add_test(file_translation "/home/pascal/Coding/claude/icp/build/tests/test_file_icp" "file_translation")
set_tests_properties(file_translation PROPERTIES  ENVIRONMENT "ICP_TEST_DATA_DIR=/home/pascal/Coding/claude/icp/tests/data" LABELS "integration" _BACKTRACE_TRIPLES "/home/pascal/Coding/claude/icp/tests/CMakeLists.txt;5;add_test;/home/pascal/Coding/claude/icp/tests/CMakeLists.txt;41;icp_add_test;/home/pascal/Coding/claude/icp/tests/CMakeLists.txt;0;")
