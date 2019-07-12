find_package( OpenCV REQUIRED )
find_package( Pangolin REQUIRED )

add_executable(main_epipolar_only main_epipolar_only.cpp)
target_link_libraries( main_epipolar_only
    ${OpenCV_LIBS}
    ${Pangolin_LIBRARIES}
    )

add_executable(test_MapViewer test_MapViewer.cpp)
target_link_libraries(test_MapViewer
    ${OpenCV_LIBS}
    ${Pangolin_LIBRARIES}
    )
