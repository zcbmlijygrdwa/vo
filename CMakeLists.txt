find_package( OpenCV REQUIRED )

add_executable(main_epipolar_only main_epipolar_only.cpp)
target_link_libraries( main_epipolar_only
    ${OpenCV_LIBS}
    )
