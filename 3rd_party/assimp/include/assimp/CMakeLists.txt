INCLUDE (${gazebo_cmake_dir}/GazeboUtils.cmake)

set (assimp_headers1
  aiAnim.h
  aiAssert.h
  aiCamera.h
  aiConfig.h
  aiDefines.h
  aiFileIO.h
  aiLight.h
  aiMaterial.h
  aiMaterial.inl
  aiMatrix3x3.h
  aiMatrix3x3.inl
  aiMatrix4x4.h
  aiMatrix4x4.inl
  aiMesh.h
  aiPostProcess.h
  aiQuaternion.h
  aiScene.h
  aiTexture.h
  aiTypes.h
  aiVector2D.h
  aiVector3D.h
  aiVector3D.inl
  aiVersion.h
  assimp.h
  assimp.hpp
  DefaultLogger.h
  IOStream.h
  IOSystem.h
  Logger.h
  LogStream.h
  NullLogger.h
)

set (assimp_headers2
  BoostWorkaround/boost/format.hpp
  BoostWorkaround/boost/scoped_array.hpp
  BoostWorkaround/boost/scoped_ptr.hpp
  BoostWorkaround/boost/static_assert.hpp
  BoostWorkaround/boost/common_factor_rt.hpp
  BoostWorkaround/boost/random/uniform_int.hpp
  BoostWorkaround/boost/random/mersenne_twister.hpp
  BoostWorkaround/boost/random/variate_generator.hpp
  BoostWorkaround/boost/foreach.hpp
  BoostWorkaround/boost/tuple/tuple.hpp
  Compiler/poppack1.h
  Compiler/pushpack1.h
  Compiler/pstdint.h
)

set (assimp_headers3
  Compiler/poppack1.h
  Compiler/pushpack1.h
  Compiler/pstdint.h
)

install (FILES ${assimp_headers1} DESTINATION ${CMAKE_INSTALL_PREFIX}/include/assimp COMPONENT headers)
install (FILES ${assimp_headers2} DESTINATION ${CMAKE_INSTALL_PREFIX}/include/assimp/BoostWorkaround/boost COMPONENT headers)
install (FILES ${assimp_headers3} DESTINATION ${CMAKE_INSTALL_PREFIX}/include/assimp/Compiler COMPONENT headers)


