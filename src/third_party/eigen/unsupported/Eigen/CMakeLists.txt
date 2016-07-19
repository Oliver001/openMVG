set(Eigen_HEADERS AdolcForward AlignedVector3 ArpackSupport AutoDiff BVH FFT IterativeSolvers KroneckerProduct LevenbergMarquardt
                  MatrixFunctions MoreVectorization MPRealSupport NonLinearOptimization NumericalDiff OpenGLSupport Polynomials
                  Skyline SparseExtra Splines
   )

if(EIGEN_INCLUDE_INSTALL_DIR)
  set(INCLUDE_INSTALL_DIR ${EIGEN_INCLUDE_INSTALL_DIR})
else()
  set(INCLUDE_INSTALL_DIR "${CMAKE_INSTALL_PREFIX}/include/eigen")
endif()

install(FILES
  ${Eigen_HEADERS}
  DESTINATION ${INCLUDE_INSTALL_DIR}/unsupported/Eigen COMPONENT Devel
  )

add_subdirectory(src)
