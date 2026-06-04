cmake_minimum_required(VERSION 3.14)

# Keep test registration in one place. Module-owned unit tests remain beside the
# code they cover, while cross-module scenarios live under this tests/ tree.
# Per-module test subdirs are gated through cmake/WKOpenVRModules.cmake so that
# a release build with disabled-in-release modules doesn't try to link tests
# against feature libs that were skipped.
add_subdirectory(driver)
add_subdirectory(ui)
wkopenvr_add_module_tests(calibration)
wkopenvr_add_module_tests(facetracking)
wkopenvr_add_module_tests(inputhealth)
wkopenvr_add_module_tests(smoothing)
wkopenvr_add_module_tests(questapp)
wkopenvr_add_module_tests(oscrouter)
wkopenvr_add_module_tests(captions)
wkopenvr_add_module_tests(phantom)
add_subdirectory(e2e)
