include (${gazebo_cmake_dir}/GazeboUtils.cmake)

include_directories(SYSTEM
  ${CMAKE_CURRENT_BINARY_DIR} 
  ${CMAKE_SOURCE_DIR}/deps/opende/OPCODE/Ice
)


########################################
# Include all the search paths for headers

set ( sources 
OPC_AABBCollider.cpp
OPC_AABBTree.cpp
OPC_BaseModel.cpp
OPC_Collider.cpp
OPC_Common.cpp
OPC_HybridModel.cpp
OPC_LSSCollider.cpp
OPC_MeshInterface.cpp
OPC_Model.cpp
OPC_OBBCollider.cpp
Opcode.cpp
OPC_OptimizedTree.cpp
OPC_Picking.cpp
OPC_PlanesCollider.cpp
OPC_RayCollider.cpp
OPC_SphereCollider.cpp
OPC_TreeBuilders.cpp
OPC_TreeCollider.cpp
OPC_VolumeCollider.cpp
StdAfx.cpp
Ice/IceAABB.cpp
Ice/IceContainer.cpp
Ice/IceHPoint.cpp
Ice/IceIndexedTriangle.cpp
Ice/IceMatrix3x3.cpp
Ice/IceMatrix4x4.cpp
Ice/IceOBB.cpp
Ice/IcePlane.cpp
Ice/IcePoint.cpp
Ice/IceRandom.cpp
Ice/IceRay.cpp
Ice/IceRevisitedRadix.cpp
Ice/IceSegment.cpp
Ice/IceTriangle.cpp
Ice/IceUtils.cpp
)

set ( headers 
OPC_AABBCollider.h
OPC_AABBTree.h
OPC_BaseModel.h
OPC_BoxBoxOverlap.h
OPC_Collider.h
OPC_Common.h
OPC_HybridModel.h
OPC_IceHook.h
OPC_LSSAABBOverlap.h
OPC_LSSCollider.h
OPC_LSSTriOverlap.h
OPC_MeshInterface.h
OPC_Model.h
OPC_OBBCollider.h
Opcode.h
OPC_OptimizedTree.h
OPC_Picking.h
OPC_PlanesAABBOverlap.h
OPC_PlanesCollider.h
OPC_PlanesTriOverlap.h
OPC_RayAABBOverlap.h
OPC_RayCollider.h
OPC_RayTriOverlap.h
OPC_Settings.h
OPC_SphereAABBOverlap.h
OPC_SphereCollider.h
OPC_SphereTriOverlap.h
OPC_TreeBuilders.h
OPC_TreeCollider.h
OPC_TriBoxOverlap.h
OPC_TriTriOverlap.h
OPC_VolumeCollider.h
Stdafx.h
)

set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DBUILDING_DLL_OPCODE")

if (WIN32)
  add_library(gazebo_opcode SHARED ${sources})
else()
  gz_add_library(gazebo_opcode ${sources})
endif()

gz_install_library(gazebo_opcode)
gz_install_includes("opcode" ${headers})
