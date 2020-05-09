add_library(slambase
camera.cpp
)

target_link_libraries(slambase
${OpenCV_LIBS}
${PCL_LIBRARIES}
${CSPARSE_LIBRARY} 
g2o_core g2o_types_slam3d g2o_solver_csparse 
g2o_stuff g2o_csparse_extension
)