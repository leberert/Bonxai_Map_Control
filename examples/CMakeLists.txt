
add_executable(tutorial tutorial.cpp)
target_link_libraries(tutorial PUBLIC bonxai_core)

###############################################
add_executable(test_serialization test_serialization.cpp)
target_link_libraries(test_serialization PUBLIC bonxai_core)

###############################################

if(octomap_FOUND)
    add_executable(bonxai_map_playground bonxai_map_playground.cpp)
    target_include_directories(bonxai_map_playground PUBLIC ${octomap_INCLUDE_DIRS})
    target_link_libraries(bonxai_map_playground PUBLIC
        bonxai_core
        bonxai_map
        ${OCTOMAP_LIBRARIES}
    )
endif()
