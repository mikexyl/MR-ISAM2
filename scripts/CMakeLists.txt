find_package(TBB REQUIRED COMPONENTS tbb tbbmalloc)
message(STATUS "TBB_INCLUDE_DIRS: ${TBB_INCLUDE_DIRS}")
message(STATUS "TBB_LIBRARIES: ${TBB_LIBRARIES}")
message(STATUS "TBB_VERSION: ${TBB_VERSION}")

find_package(aria_common REQUIRED)
find_package(aria_viz REQUIRED)
# find absl
find_package(absl REQUIRED)

add_executable(run_toy_graph run_toy_graph.cpp)
target_link_libraries(run_toy_graph
    PRIVATE
    ${PROJECT_NAME}
    aria_viz
    aria_common
    ${TBB_LIBRARIES}
    absl::flags
    absl::flags_parse
)
target_include_directories(run_toy_graph
    PRIVATE
    ${TBB_INCLUDE_DIRS}
)