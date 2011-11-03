# Microsoft Developer Studio Project File - Name="OPCODE" - Package Owner=<4>
# Microsoft Developer Studio Generated Build File, Format Version 6.00
# ** DO NOT EDIT **

# TARGTYPE "Win32 (x86) Static Library" 0x0104

CFG=OPCODE - Win32 Debug
!MESSAGE This is not a valid makefile. To build this project using NMAKE,
!MESSAGE use the Export Makefile command and run
!MESSAGE 
!MESSAGE NMAKE /f "Opcode.mak".
!MESSAGE 
!MESSAGE You can specify a configuration when running NMAKE
!MESSAGE by defining the macro CFG on the command line. For example:
!MESSAGE 
!MESSAGE NMAKE /f "Opcode.mak" CFG="OPCODE - Win32 Debug"
!MESSAGE 
!MESSAGE Possible choices for configuration are:
!MESSAGE 
!MESSAGE "OPCODE - Win32 Release" (based on "Win32 (x86) Static Library")
!MESSAGE "OPCODE - Win32 Debug" (based on "Win32 (x86) Static Library")
!MESSAGE 

# Begin Project
# PROP AllowPerConfigDependencies 0
# PROP Scc_ProjName ""$/TR4/ODE/VC6", WNKAAAAA"
# PROP Scc_LocalPath "..\vc6"
CPP=cl.exe
RSC=rc.exe

!IF  "$(CFG)" == "OPCODE - Win32 Release"

# PROP BASE Use_MFC 0
# PROP BASE Use_Debug_Libraries 0
# PROP BASE Output_Dir "Release"
# PROP BASE Intermediate_Dir "Release"
# PROP BASE Target_Dir ""
# PROP Use_MFC 0
# PROP Use_Debug_Libraries 0
# PROP Output_Dir "Release"
# PROP Intermediate_Dir "Release"
# PROP Target_Dir ""
# ADD BASE CPP /nologo /W3 /GX /O2 /D "WIN32" /D "NDEBUG" /D "_MBCS" /D "_LIB" /YX /FD /c
# ADD CPP /nologo /G6 /Zp4 /MD /O2 /Ob0 /I ".\\" /D "WIN32" /D "NDEBUG" /D "_MBCS" /D "_LIB" /D "ICE_NO_DLL" /FD /c
# SUBTRACT CPP /Fr /YX
# ADD BASE RSC /l 0x409 /d "NDEBUG"
# ADD RSC /l 0x409 /d "NDEBUG"
BSC32=bscmake.exe
# ADD BASE BSC32 /nologo
# ADD BSC32 /nologo
LIB32=link.exe -lib
# ADD BASE LIB32 /nologo
# ADD LIB32 /nologo /out:"..\lib\OPCODE.lib"

!ELSEIF  "$(CFG)" == "OPCODE - Win32 Debug"

# PROP BASE Use_MFC 0
# PROP BASE Use_Debug_Libraries 1
# PROP BASE Output_Dir "Debug"
# PROP BASE Intermediate_Dir "Debug"
# PROP BASE Target_Dir ""
# PROP Use_MFC 0
# PROP Use_Debug_Libraries 1
# PROP Output_Dir "Debug"
# PROP Intermediate_Dir "Debug"
# PROP Target_Dir ""
# ADD BASE CPP /nologo /W3 /Gm /GX /ZI /Od /D "WIN32" /D "_DEBUG" /D "_MBCS" /D "_LIB" /YX /FD /GZ /c
# ADD CPP /nologo /G6 /Zp4 /MDd /Gm /ZI /Od /I ".\\" /D "WIN32" /D "_DEBUG" /D "_MBCS" /D "_LIB" /D "ICE_NO_DLL" /FR /FD /GZ /c
# SUBTRACT CPP /YX
# ADD BASE RSC /l 0x409 /d "_DEBUG"
# ADD RSC /l 0x409 /d "_DEBUG"
BSC32=bscmake.exe
# ADD BASE BSC32 /nologo
# ADD BSC32 /nologo
LIB32=link.exe -lib
# ADD BASE LIB32 /nologo
# ADD LIB32 /nologo /out:"..\lib\OPCODE_D.lib"

!ENDIF 

# Begin Target

# Name "OPCODE - Win32 Release"
# Name "OPCODE - Win32 Debug"
# Begin Source File

SOURCE=.\Ice\IceAABB.cpp
# End Source File
# Begin Source File

SOURCE=.\Ice\IceAABB.h
# End Source File
# Begin Source File

SOURCE=.\Ice\IceAxes.h
# End Source File
# Begin Source File

SOURCE=.\Ice\IceBoundingSphere.h
# End Source File
# Begin Source File

SOURCE=.\Ice\IceContainer.cpp
# End Source File
# Begin Source File

SOURCE=.\Ice\IceContainer.h
# End Source File
# Begin Source File

SOURCE=.\Ice\IceFPU.h
# End Source File
# Begin Source File

SOURCE=.\Ice\IceHPoint.cpp
# End Source File
# Begin Source File

SOURCE=.\Ice\IceHPoint.h
# End Source File
# Begin Source File

SOURCE=.\Ice\IceIndexedTriangle.cpp
# End Source File
# Begin Source File

SOURCE=.\Ice\IceIndexedTriangle.h
# End Source File
# Begin Source File

SOURCE=.\Ice\IceLSS.h
# End Source File
# Begin Source File

SOURCE=.\Ice\IceMatrix3x3.cpp
# End Source File
# Begin Source File

SOURCE=.\Ice\IceMatrix3x3.h
# End Source File
# Begin Source File

SOURCE=.\Ice\IceMatrix4x4.cpp
# End Source File
# Begin Source File

SOURCE=.\Ice\IceMatrix4x4.h
# End Source File
# Begin Source File

SOURCE=.\Ice\IceMemoryMacros.h
# End Source File
# Begin Source File

SOURCE=.\Ice\IceOBB.cpp
# End Source File
# Begin Source File

SOURCE=.\Ice\IceOBB.h
# End Source File
# Begin Source File

SOURCE=.\Ice\IcePairs.h
# End Source File
# Begin Source File

SOURCE=.\Ice\IcePlane.cpp
# End Source File
# Begin Source File

SOURCE=.\Ice\IcePlane.h
# End Source File
# Begin Source File

SOURCE=.\Ice\IcePoint.cpp
# End Source File
# Begin Source File

SOURCE=.\Ice\IcePoint.h
# End Source File
# Begin Source File

SOURCE=.\Ice\IcePreprocessor.h
# End Source File
# Begin Source File

SOURCE=.\Ice\IceRandom.cpp
# End Source File
# Begin Source File

SOURCE=.\Ice\IceRandom.h
# End Source File
# Begin Source File

SOURCE=.\Ice\IceRay.cpp
# End Source File
# Begin Source File

SOURCE=.\Ice\IceRay.h
# End Source File
# Begin Source File

SOURCE=.\Ice\IceRevisitedRadix.cpp
# End Source File
# Begin Source File

SOURCE=.\Ice\IceRevisitedRadix.h
# End Source File
# Begin Source File

SOURCE=.\Ice\IceSegment.cpp
# End Source File
# Begin Source File

SOURCE=.\Ice\IceSegment.h
# End Source File
# Begin Source File

SOURCE=.\Ice\IceTriangle.cpp
# End Source File
# Begin Source File

SOURCE=.\Ice\IceTriangle.h
# End Source File
# Begin Source File

SOURCE=.\Ice\IceTrilist.h
# End Source File
# Begin Source File

SOURCE=.\Ice\IceTypes.h
# End Source File
# Begin Source File

SOURCE=.\Ice\IceUtils.cpp
# End Source File
# Begin Source File

SOURCE=.\Ice\IceUtils.h
# End Source File
# Begin Source File

SOURCE=.\OPC_AABBCollider.cpp
# End Source File
# Begin Source File

SOURCE=.\OPC_AABBCollider.h
# End Source File
# Begin Source File

SOURCE=.\OPC_AABBTree.cpp
# End Source File
# Begin Source File

SOURCE=.\OPC_AABBTree.h
# End Source File
# Begin Source File

SOURCE=.\OPC_BaseModel.cpp
# End Source File
# Begin Source File

SOURCE=.\OPC_BaseModel.h
# End Source File
# Begin Source File

SOURCE=.\OPC_BoxBoxOverlap.h
# End Source File
# Begin Source File

SOURCE=.\OPC_BoxPruning.cpp
# End Source File
# Begin Source File

SOURCE=.\OPC_BoxPruning.h
# End Source File
# Begin Source File

SOURCE=.\OPC_Collider.cpp
# End Source File
# Begin Source File

SOURCE=.\OPC_Collider.h
# End Source File
# Begin Source File

SOURCE=.\OPC_Common.cpp
# End Source File
# Begin Source File

SOURCE=.\OPC_Common.h
# End Source File
# Begin Source File

SOURCE=.\OPC_HybridModel.cpp
# End Source File
# Begin Source File

SOURCE=.\OPC_HybridModel.h
# End Source File
# Begin Source File

SOURCE=.\OPC_IceHook.h
# End Source File
# Begin Source File

SOURCE=.\OPC_LSSAABBOverlap.h
# End Source File
# Begin Source File

SOURCE=.\OPC_LSSCollider.cpp
# End Source File
# Begin Source File

SOURCE=.\OPC_LSSCollider.h
# End Source File
# Begin Source File

SOURCE=.\OPC_LSSTriOverlap.h
# End Source File
# Begin Source File

SOURCE=.\OPC_MeshInterface.cpp
# End Source File
# Begin Source File

SOURCE=.\OPC_MeshInterface.h
# End Source File
# Begin Source File

SOURCE=.\OPC_Model.cpp
# End Source File
# Begin Source File

SOURCE=.\OPC_Model.h
# End Source File
# Begin Source File

SOURCE=.\OPC_OBBCollider.cpp
# End Source File
# Begin Source File

SOURCE=.\OPC_OBBCollider.h
# End Source File
# Begin Source File

SOURCE=.\OPC_OptimizedTree.cpp
# End Source File
# Begin Source File

SOURCE=.\OPC_OptimizedTree.h
# End Source File
# Begin Source File

SOURCE=.\OPC_Picking.cpp
# End Source File
# Begin Source File

SOURCE=.\OPC_Picking.h
# End Source File
# Begin Source File

SOURCE=.\OPC_PlanesAABBOverlap.h
# End Source File
# Begin Source File

SOURCE=.\OPC_PlanesCollider.cpp
# End Source File
# Begin Source File

SOURCE=.\OPC_PlanesCollider.h
# End Source File
# Begin Source File

SOURCE=.\OPC_PlanesTriOverlap.h
# End Source File
# Begin Source File

SOURCE=.\OPC_RayAABBOverlap.h
# End Source File
# Begin Source File

SOURCE=.\OPC_RayCollider.cpp
# End Source File
# Begin Source File

SOURCE=.\OPC_RayCollider.h
# End Source File
# Begin Source File

SOURCE=.\OPC_RayTriOverlap.h
# End Source File
# Begin Source File

SOURCE=.\OPC_Settings.h
# End Source File
# Begin Source File

SOURCE=.\OPC_SphereAABBOverlap.h
# End Source File
# Begin Source File

SOURCE=.\OPC_SphereCollider.cpp
# End Source File
# Begin Source File

SOURCE=.\OPC_SphereCollider.h
# End Source File
# Begin Source File

SOURCE=.\OPC_SphereTriOverlap.h
# End Source File
# Begin Source File

SOURCE=.\OPC_SweepAndPrune.cpp
# End Source File
# Begin Source File

SOURCE=.\OPC_SweepAndPrune.h
# End Source File
# Begin Source File

SOURCE=.\OPC_TreeBuilders.cpp
# End Source File
# Begin Source File

SOURCE=.\OPC_TreeBuilders.h
# End Source File
# Begin Source File

SOURCE=.\OPC_TreeCollider.cpp
# End Source File
# Begin Source File

SOURCE=.\OPC_TreeCollider.h
# End Source File
# Begin Source File

SOURCE=.\OPC_TriBoxOverlap.h
# End Source File
# Begin Source File

SOURCE=.\OPC_TriTriOverlap.h
# End Source File
# Begin Source File

SOURCE=.\OPC_VolumeCollider.cpp
# End Source File
# Begin Source File

SOURCE=.\OPC_VolumeCollider.h
# End Source File
# Begin Source File

SOURCE=.\Opcode.cpp
# End Source File
# Begin Source File

SOURCE=.\Opcode.h
# End Source File
# Begin Source File

SOURCE=.\StdAfx.cpp
# End Source File
# Begin Source File

SOURCE=.\StdAfx.h
# End Source File
# End Target
# End Project
