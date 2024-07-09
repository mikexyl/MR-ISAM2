cmake_minimum_required(VERSION 3.0)
enable_testing()
project(mr_isam2 VERSION 0.1.0 LANGUAGES CXX C)

find_package(GTSAM REQUIRED) # Uses installed package
include_directories(${GTSAM_INCLUDE_DIR})
set(GTSAM_LIBS gtsam)

find_package(GTSAMCMakeTools)
include(GtsamMakeConfigFile)
include(GtsamBuildTypes)
include(GtsamTesting)

find_package(GTest REQUIRED)
include_directories(${GTEST_INCLUDE_DIR})

file(GLOB sources "*.cpp")
file(GLOB headers "*.h")

include_directories(".")

add_library(mr_isam2 SHARED ${sources} ${headers})
target_link_libraries(mr_isam2 ${GTSAM_LIBS})
set_target_properties(mr_isam2 PROPERTIES LINKER_LANGUAGE CXX)


add_subdirectory(tests)
add_subdirectory(scripts)

# Install targets
install(TARGETS mr_isam2
        EXPORT mr_isam2Targets
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION include)

# Install header files
install(FILES ${headers} DESTINATION include/mr_isam2)


# Export targets
export(EXPORT mr_isam2Targets
       FILE "${CMAKE_CURRENT_BINARY_DIR}/mr_isam2Targets.cmake"
       NAMESPACE mr_isam2::)

# Install export targets
install(EXPORT mr_isam2Targets
        FILE mr_isam2Targets.cmake
        NAMESPACE mr_isam2::
        DESTINATION lib/cmake/mr_isam2)

# Create and install the config file for find_package
include(CMakePackageConfigHelpers)

configure_package_config_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/cmake/mr_isam2Config.cmake.in"
    "${CMAKE_CURRENT_BINARY_DIR}/mr_isam2Config.cmake"
    INSTALL_DESTINATION lib/cmake/mr_isam2
)

install(FILES
    "${CMAKE_CURRENT_BINARY_DIR}/mr_isam2Config.cmake"
    DESTINATION lib/cmake/mr_isam2
)

# Version file for compatibility checking
write_basic_package_version_file(
    "${CMAKE_CURRENT_BINARY_DIR}/mr_isam2ConfigVersion.cmake"
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY AnyNewerVersion
)

install(FILES
    "${CMAKE_CURRENT_BINARY_DIR}/mr_isam2ConfigVersion.cmake"
    DESTINATION lib/cmake/mr_isam2
)