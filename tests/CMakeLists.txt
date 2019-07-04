add_library( estimatorFixture EstimatorFixture.cpp )
target_include_directories( estimatorFixture PRIVATE
  .
  ..
  ${EIGEN3_INCLUDE_DIR}
)
target_link_libraries( estimatorFixture
  commonFixture
)

add_executable( visualizePoses visualizePoses.cpp )
target_include_directories( visualizePoses PRIVATE
  .
  ..
  ${EIGEN3_INCLUDE_DIR}
)
target_link_libraries( visualizePoses
  estimatorFixture
  pointVisualizer
)
