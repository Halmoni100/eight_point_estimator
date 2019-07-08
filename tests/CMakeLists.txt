add_library( estimatorFixture SHARED EstimatorFixture.cpp )
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

add_executable( checkEstimator checkEstimator.cpp )
target_include_directories( checkEstimator PRIVATE
  .
  ..
  ${EIGEN3_INCLUDE_DIR}
  ${Boost_INCLUDE_DIRS}
)
target_link_libraries( checkEstimator
  estimatorFixture
  helperFunctions
  eightPointEstimator
  ${Boost_LIBRARIES}
)
