add_executable(multicam
    multicam.cpp
)

target_link_libraries(multicam
    PRIVATE
        realsense2::realsense2
        ${OpenCV_LIBS}
)