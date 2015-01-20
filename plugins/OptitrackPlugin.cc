/*
 * Copyright (C) 2014 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

// For data types
#include <sys/types.h>
// For socket(), connect(), send(), and recv()
#include <sys/socket.h>
// For gethostbyname()
#include <netdb.h>
// For inet_addr()
#include <arpa/inet.h>
// For close()
#include <unistd.h>
// For sockaddr_in
#include <netinet/in.h>

#include <cstring>
#include <iostream>
#include <string>
#include <sstream>

#include <ignition/transport/NetUtils.hh>
#include <gazebo/msgs/msgs.hh>

#include "OptitrackPlugin.hh"

using namespace gazebo;

GZ_REGISTER_WORLD_PLUGIN(OptitrackPlugin)

/////////////////////////////////////////////////
OptitrackPlugin::OptitrackPlugin(const bool _verbose)
  : verbose(_verbose)
{
  this->active = false;
  //this->myIPAddress = ignition::transport::determineHost();
}

void OptitrackPlugin::Load(physics::WorldPtr _parent,
        sdf::ElementPtr _sdf)
{
  // Get world name
  this->world = _parent->GetName();

  // Get IP from plugin SDF
  if (_sdf->HasElement("ip"))
  {
    sdf::ElementPtr elem = _sdf->GetElement("ip");
    this->myIPAddress = elem->Get<std::string>();
  }

  // Get names of tracked items from plugin SDF
  if (_sdf->HasElement("tracker"))
  {
    sdf::ElementPtr elem = _sdf->GetElement("tracker");
    while (elem)
    {
      this->trackerNames.push_back(elem->Get<std::string>());
      elem = elem->GetNextElement("tracker");
    }
  }

  if (_sdf->HasElement("multicast_address"))
  {
    sdf::ElementPtr elem = _sdf->GetElement("multicast_address");
    this->MulticastAddress = elem->Get<std::string>();
  }
  else
  {
    this->MulticastAddress = "239.255.42.99";
  }

  if (_sdf->HasElement("port_data"))
  {
    sdf::ElementPtr elem = _sdf->GetElement("port_data");
    this->PortData = elem->Get<int>();
  }
  else
  {
    this->PortData = 1511;
  }
}

void OptitrackPlugin::Init()
{
}

/////////////////////////////////////////////////
void OptitrackPlugin::StartReception()
{
  // Create a socket for receiving tracking updates.
  this->dataSocket = socket(AF_INET, SOCK_DGRAM, 0);

  // Socket option: SO_REUSEADDR.
  int value = 1;
  if (setsockopt(this->dataSocket, SOL_SOCKET, SO_REUSEADDR,
    reinterpret_cast<const char *>(&value), sizeof(value)) != 0)
  {
    gzerr << "Error setting socket option (SO_REUSEADDR)." << std::endl;
    close(this->dataSocket);
    return;
  }

  // Bind the socket to the "OptitrackPlugin tracking updates" port.
  struct sockaddr_in mySocketAddr;
  memset(&mySocketAddr, 0, sizeof(mySocketAddr));
  mySocketAddr.sin_family = AF_INET;
  mySocketAddr.sin_port = htons(this->PortData);
  mySocketAddr.sin_addr.s_addr = htonl(INADDR_ANY);;
  if (bind(this->dataSocket, (struct sockaddr *)&mySocketAddr,
    sizeof(struct sockaddr)) < 0)
  {
    gzerr << "Binding to a local port failed." << std::endl;
    return;
  }
  // Join the multicast group.
  struct ip_mreq mreq;
  mreq.imr_multiaddr.s_addr = inet_addr(this->MulticastAddress.c_str());
  mreq.imr_interface.s_addr = inet_addr(this->myIPAddress.c_str());
  if (setsockopt(this->dataSocket, IPPROTO_IP, IP_ADD_MEMBERSHIP,
    reinterpret_cast<const char*>(&mreq), sizeof(mreq)) != 0)
  {
    gzerr << "Error setting socket option (IP_ADD_MEMBERSHIP)."
              << std::endl;
    return;
  }

  this->gzNode = transport::NodePtr(new transport::Node());

  if (this->world.size() > 0)
  {
    this->gzNode->Init(this->world);
  }

  for (unsigned int i = 0; i < this->trackerNames.size(); i++)
  {
    this->trackerPubs[this->trackerNames[i]] =
      this->gzNode->Advertise<msgs::Pose>
        ("~/optitrack/"+this->trackerNames[i]);
  }

  this->active = true;
  this->RunReceptionTask();
}

/////////////////////////////////////////////////
bool OptitrackPlugin::IsActive()
{
  return this->active;
}

/////////////////////////////////////////////////
void OptitrackPlugin::RunReceptionTask()
{
  char buffer[20000];
  socklen_t addr_len = sizeof(struct sockaddr);
  sockaddr_in theirAddress;

  while (1)
  {
    // Block until we receive a datagram from the network (from anyone
    //  including ourselves)
    if (recvfrom(this->dataSocket, buffer, sizeof(buffer), 0,
         (sockaddr *)&theirAddress, &addr_len) < 0)
    {
      gzerr << "OptitrackPlugin::RunReceptionTask() Recvfrom failed" << std::endl;
      continue;
    }

    // Dispatch the data received.
    this->Unpack(buffer);

    // Publish messages

    for (ModelPoses::iterator it = this->lastModelMap.begin();
         it != this->lastModelMap.end(); ++it)
    {
      if (this->trackerPubs.find(it->first) != this->trackerPubs.end())
      {
        this->trackerPubs[it->first]->Publish
          (msgs::Convert(it->second));
      }
      else
      {
        std::cout << "Model name " << it->first << " not found!" << std::endl;
      }
    }

    this->lastModelMap.clear();
  }
  this->active = false;
}

/////////////////////////////////////////////////
void OptitrackPlugin::Unpack(char *pData)
{
  int major = this->NatNetVersionMajor;
  int minor = this->NatNetVersionMinor;

  std::stringstream output;

  char *ptr = pData;

  // message ID
  int MessageID = 0;
  memcpy(&MessageID, ptr, 2); ptr += 2;

  // size
  int nBytes = 0;
  memcpy(&nBytes, ptr, 2); ptr += 2;

  if (MessageID == 7)      // FRAME OF MOCAP DATA packet
  {
    // frame number
    int frameNumber = 0; memcpy(&frameNumber, ptr, 4); ptr += 4;
    output << "Frame # :" << frameNumber << std::endl;

    // number of data sets (markersets, rigidbodies, etc)
    int nMarkerSets = 0; memcpy(&nMarkerSets, ptr, 4); ptr += 4;
    output << "Marker Set Count : " << nMarkerSets << std::endl;

    ModelMarkers markerSets;

    for (int i = 0; i < nMarkerSets; i++)
    {
      // Markerset name
      std::string szName(ptr);
      int nDataBytes = (int) strlen(ptr) + 1;
      ptr += nDataBytes;
      output << "Model Name: " << szName << std::endl;
      markerSets[szName] = std::vector<math::Vector3>();

      // marker data
      int nMarkers = 0;
      memcpy(&nMarkers, ptr, 4);
      ptr += 4;
      output << "Marker Count: " << nMarkers << std::endl;

      for (int j = 0; j < nMarkers; j++)
      {
        float x = 0; memcpy(&x, ptr, 4); ptr += 4;
        float y = 0; memcpy(&y, ptr, 4); ptr += 4;
        float z = 0; memcpy(&z, ptr, 4); ptr += 4;
        output << "\tMarker " << j << " : [x="
              << x << ",y=" << y << ",z=" << z << "]" << std::endl;
        markerSets[szName].push_back(math::Vector3(x, y, z));
      }
    }

    // unidentified markers
    int nOtherMarkers = 0; memcpy(&nOtherMarkers, ptr, 4); ptr += 4;
    for (int j = 0; j < nOtherMarkers; j++)
    {
      float x = 0.0f; memcpy(&x, ptr, 4); ptr += 4;
      float y = 0.0f; memcpy(&y, ptr, 4); ptr += 4;
      float z = 0.0f; memcpy(&z, ptr, 4); ptr += 4;
    }

    // rigid bodies
    int nRigidBodies = 0;
    memcpy(&nRigidBodies, ptr, 4); ptr += 4;
    output << "Rigid Body Count : " << nRigidBodies << std::endl;
    for (int j = 0; j < nRigidBodies; j++)
    {
      // rigid body pos/ori
      int ID = 0; memcpy(&ID, ptr, 4); ptr += 4;
      float x = 0.0f; memcpy(&x, ptr, 4); ptr += 4;
      float y = 0.0f; memcpy(&y, ptr, 4); ptr += 4;
      float z = 0.0f; memcpy(&z, ptr, 4); ptr += 4;
      float qx = 0; memcpy(&qx, ptr, 4); ptr += 4;
      float qy = 0; memcpy(&qy, ptr, 4); ptr += 4;
      float qz = 0; memcpy(&qz, ptr, 4); ptr += 4;
      float qw = 0; memcpy(&qw, ptr, 4); ptr += 4;
      output << "ID : " << ID << std::endl;
      output << "pos: [" << x << "," << y << "," << z << "]" << std::endl;
      output << "ori: [" << qx << "," << qy << ","
                        << qz << "," << qw << "]" << std::endl;

      math::Pose rigidBodyPose(math::Vector3(x, y, z),
                             math::Quaternion(qw, qx, qy, qz));

      // associated marker positions
      unsigned int nRigidMarkers = 0; 
      memcpy(&nRigidMarkers, ptr, 4);
      ptr += 4;
      output << "Rigid Marker Count: " << nRigidMarkers << std::endl;
      int nMarkerBytes = nRigidMarkers*3*sizeof(float);
      float* markerData = (float*)malloc(nMarkerBytes);
      memcpy(markerData, ptr, nMarkerBytes);
      ptr += nMarkerBytes;

      for (ModelMarkers::iterator it = markerSets.begin();
           it != markerSets.end(); ++it)
      {
        if (it->second.size() == nRigidMarkers)
        {
          // Compare all the points
          unsigned int k = 0;
          for (k = 0; k < nRigidMarkers; k++)
          {
            if (std::fabs(it->second[k][0] - markerData[k*3]) > FLT_EPSILON ||
              std::fabs(it->second[k][1] - markerData[k*3+1]) > FLT_EPSILON ||
              std::fabs(it->second[k][2] - markerData[k*3+2]) > FLT_EPSILON)
            {
              break;
            }
          }
          if (k == nRigidMarkers)
          {
            this->lastModelMap[it->first] = rigidBodyPose;
          }
        }
      }

      if (major >= 2)
      {
        // associated marker IDs
        int nIntBytes = nRigidMarkers*sizeof(int);
        int* markerIDs = (int*)malloc(nIntBytes);
        memcpy(markerIDs, ptr, nIntBytes);
        ptr += nIntBytes;

        // associated marker sizes
        int nFloatBytes = nRigidMarkers*sizeof(float);
        float* markerSizes = (float*)malloc(nFloatBytes);
        memcpy(markerSizes, ptr, nFloatBytes);
        ptr += nFloatBytes;

        for (unsigned int k = 0; k < nRigidMarkers; k++)
        {
          output << "\tMarker " << k << ": id=" << markerIDs[k]
                << "\tsize=" << markerSizes[k]
                << "\tpos=[" << markerData[k*3] << "," << markerData[k*3+1]
                << "," << markerData[k*3+2] << std::endl;
        }

        if (markerIDs)
          free(markerIDs);
        if (markerSizes)
          free(markerSizes);
      }
      else
      {
        for (unsigned int k = 0; k < nRigidMarkers; k++)
        {
          output << "\tMarker " << k << ": "
                << "pos=[" << markerData[k*3] << "," << markerData[k*3+1]
                << "," << markerData[k*3+2] << std::endl;
        }
      }
      if (markerData)
        free(markerData);

      if (major >= 2)
      {
        // Mean marker error
        float fError = 0.0f; memcpy(&fError, ptr, 4); ptr += 4;
        output << "Mean marker error: " << fError << std::endl;
      }

      // 2.6 and later
      if (((major == 2) && (minor >= 6)) || (major > 2) || (major == 0) )
      {
        // params
        short params = 0; memcpy(&params, ptr, 2); ptr += 2;
        // 0x01 : rigid body was successfully tracked in this frame
        bool bTrackingValid = params & 0x01;
        output << "Tracking valid?: " << bTrackingValid << std::endl;
      }
    } // next rigid body

    // skeletons (version 2.1 and later)
    if (((major == 2) && (minor>0)) || (major>2))
    {
      int nSkeletons = 0;
      memcpy(&nSkeletons, ptr, 4); ptr += 4;
      output << "Skeleton Count : " << nSkeletons << std::endl;;
      for (int j = 0; j < nSkeletons; j++)
      {
        // skeleton id
        int skeletonID = 0;
        memcpy(&skeletonID, ptr, 4); ptr += 4;
        // # of rigid bodies (bones) in skeleton
        nRigidBodies = 0;
        memcpy(&nRigidBodies, ptr, 4); ptr += 4;
        output << "Rigid Body Count : " << nRigidBodies << std::endl;
        for (int b = 0; b < nRigidBodies; b++)
        {
          // rigid body pos/ori
          int ID = 0; memcpy(&ID, ptr, 4); ptr += 4;
          float x = 0.0f; memcpy(&x, ptr, 4); ptr += 4;
          float y = 0.0f; memcpy(&y, ptr, 4); ptr += 4;
          float z = 0.0f; memcpy(&z, ptr, 4); ptr += 4;
          float qx = 0; memcpy(&qx, ptr, 4); ptr += 4;
          float qy = 0; memcpy(&qy, ptr, 4); ptr += 4;
          float qz = 0; memcpy(&qz, ptr, 4); ptr += 4;
          float qw = 0; memcpy(&qw, ptr, 4); ptr += 4;
          output << "ID : " << ID << std::endl;
          output << "pos: [" << x << "," << y << "," << z << "]" << std::endl;
          output << "ori: [" << qx << "," << qy << ","
                            << qz << "," << qw << "]" << std::endl;

          // associated marker positions
          int nRigidMarkers = 0;  memcpy(&nRigidMarkers, ptr, 4); ptr += 4;
          output << "Marker Count: " << nRigidMarkers << std::endl;
          int nMarkerBytes = nRigidMarkers*3*sizeof(float);
          float* markerData = (float*)malloc(nMarkerBytes);
          memcpy(markerData, ptr, nMarkerBytes);
          ptr += nMarkerBytes;

          // associated marker IDs
          nMarkerBytes = nRigidMarkers*sizeof(int);
          int* markerIDs = (int*)malloc(nMarkerBytes);
          memcpy(markerIDs, ptr, nMarkerBytes);
          ptr += nMarkerBytes;

          // associated marker sizes
          nMarkerBytes = nRigidMarkers*sizeof(float);
          float* markerSizes = (float*)malloc(nMarkerBytes);
          memcpy(markerSizes, ptr, nMarkerBytes);
          ptr += nMarkerBytes;

          for (int k = 0; k < nRigidMarkers; k++)
          {
            output << "\tMarker " << k << ": id=" << markerIDs[k]
                  << "\tsize=" << markerSizes[k]
                  << "\tpos=[" << markerData[k*3] << "," << markerData[k*3+1]
                  << "," << markerData[k*3+2] << std::endl;
          }

          // Mean marker error
          float fError = 0.0f; memcpy(&fError, ptr, 4); ptr += 4;
          output << "Mean marker error: " << fError << std::endl;

          // release resources
          if(markerIDs)
            free(markerIDs);
          if(markerSizes)
            free(markerSizes);
          if(markerData)
            free(markerData);
        } // next rigid body
      } // next skeleton
    }

    // labeled markers (version 2.3 and later)
    if (((major == 2) && (minor>=3)) || (major>2))
    {
      int nLabeledMarkers = 0;
      memcpy(&nLabeledMarkers, ptr, 4); ptr += 4;
      output << "Labeled Marker Count : " << nLabeledMarkers << std::endl;
      for (int j = 0; j < nLabeledMarkers; j++)
      {
        int ID = 0; memcpy(&ID, ptr, 4); ptr += 4;
        float x = 0.0f; memcpy(&x, ptr, 4); ptr += 4;
        float y = 0.0f; memcpy(&y, ptr, 4); ptr += 4;
        float z = 0.0f; memcpy(&z, ptr, 4); ptr += 4;
        float size = 0.0f; memcpy(&size, ptr, 4); ptr += 4;

        // 2.6 and later
        if (((major == 2) && (minor >= 6)) || (major > 2) || (major == 0) )
        {
          // marker params
          short params = 0; memcpy(&params, ptr, 2); ptr += 2;
          output << "Params: " << params << std::endl;
          // marker was not visible (occluded) in this frame
          /* bool bOccluded = params & 0x01;
          // position provided by point cloud solve
          bool bPCSolved = params & 0x02;
          // position provided by model solve
          bool bModelSolved = params & 0x04; */
        }

        output << "ID  : " << ID << std::endl;
        output << "pos : [" << x << "," << y << "," << z<< "]" << std::endl;
        output << "size: [" << size << "]" << std::endl;
      }
    }

    // latency
    float latency = 0.0f; memcpy(&latency, ptr, 4); ptr += 4;
    output << "latency : " << latency << std::endl;

    // timecode
    unsigned int timecode = 0;  memcpy(&timecode, ptr, 4);  ptr += 4;
    unsigned int timecodeSub = 0; memcpy(&timecodeSub, ptr, 4); ptr += 4;
    char szTimecode[128] = "";
    this->TimecodeStringify(timecode, timecodeSub, szTimecode, 128);

    // timestamp
    double timestamp = 0.0f;
    // 2.7 and later - increased from single to double precision
    if( ((major == 2)&&(minor>=7)) || (major>2))
    {
        memcpy(&timestamp, ptr, 8); ptr += 8;
    }
    else
    {
      float fTemp = 0.0f;
      memcpy(&fTemp, ptr, 4); ptr += 4;
      timestamp = (double)fTemp;
    }
    output << "Timestamp: " << timestamp;

    // frame params
    short params = 0;  memcpy(&params, ptr, 2); ptr += 2;
    output << "Params: " << params << std::endl;
    // 0x01 Motive is recording
    /* bool bIsRecording = params & 0x01;
    // 0x02 Actively tracked model list has changed
    bool bTrackedModelsChanged = params & 0x02; */

    // end of data tag
    int eod = 0; memcpy(&eod, ptr, 4); ptr += 4;
    output << "End Packet\n-------------" << std::endl;
  }
  else if (MessageID == 5) // Data Descriptions
  {
    // number of datasets
    int nDatasets = 0; memcpy(&nDatasets, ptr, 4); ptr += 4;
    output << "Dataset Count : " << nDatasets << std::endl;

    for (int i = 0; i < nDatasets; i++)
    {
      output << "Dataset " << i << std::endl;

      int packetType = 0; memcpy(&packetType, ptr, 4); ptr += 4;
      output << "Type : " << packetType << std::endl;

      if (packetType == 0)   // markerset
      {
        // name
        std::string szName(ptr);
        int nDataBytes = (int) strlen(ptr) + 1;
        ptr += nDataBytes;
        output << "Markerset Name: " << szName << std::endl;

        // marker data
        int nMarkers = 0; memcpy(&nMarkers, ptr, 4); ptr += 4;
        output << "Marker Count : " << nMarkers << std::endl;

        for (int j = 0; j < nMarkers; j++)
        {
          std::string markerName(ptr);
          int nMarkerDataBytes = (int) strlen(ptr) + 1;
          ptr += nMarkerDataBytes;
          output << "Marker Name: " << markerName << std::endl;
        }
      }
      else if (packetType == 1)   // rigid body
      {
        if (major >= 2)
        {
          // name
          char szName[MAX_NAMELENGTH];
          strcpy(szName, ptr);
          ptr += strlen(ptr) + 1;
          output << "Name: " << szName << std::endl;
        }

        int ID = 0; memcpy(&ID, ptr, 4); ptr +=4;
        output << "ID : " << ID << std::endl;

        int parentID = 0; memcpy(&parentID, ptr, 4); ptr +=4;
        output << "Parent ID : " << parentID << std::endl;

        float xoffset = 0; memcpy(&xoffset, ptr, 4); ptr +=4;
        output << "X Offset : " << xoffset << std::endl;

        float yoffset = 0; memcpy(&yoffset, ptr, 4); ptr +=4;
        output << "Y Offset : " << yoffset << std::endl;;

        float zoffset = 0; memcpy(&zoffset, ptr, 4); ptr +=4;
        output << "Z Offset : " << zoffset << std::endl;
      }
      else if (packetType == 2)   // skeleton
      {
        char szName[MAX_NAMELENGTH];
        strcpy(szName, ptr);
        ptr += strlen(ptr) + 1;
        output << "Name: " << szName << std::endl;

        int ID = 0; memcpy(&ID, ptr, 4); ptr +=4;
        output << "ID : " << ID << std::endl;

        int nRigidBodies = 0; memcpy(&nRigidBodies, ptr, 4); ptr +=4;
        output << "RigidBody (Bone) Count : " << nRigidBodies << std::endl;

        for (int j = 0; j < nRigidBodies; j++)
        {
          if (major >= 2)
          {
            // RB name
            char rigidBodyName[MAX_NAMELENGTH];
            strcpy(rigidBodyName, ptr);
            ptr += strlen(ptr) + 1;
            output << "Rigid Body Name: " << rigidBodyName << std::endl;
          }

          int rigidBodyID = 0; memcpy(&rigidBodyID, ptr, 4); ptr +=4;
          output << "RigidBody ID : " << rigidBodyID << std::endl;

          int parentID = 0; memcpy(&parentID, ptr, 4); ptr +=4;
          output << "Parent ID : " << parentID << std::endl;

          float xoffset = 0; memcpy(&xoffset, ptr, 4); ptr +=4;
          output << "X Offset : " << xoffset << std::endl;

          float yoffset = 0; memcpy(&yoffset, ptr, 4); ptr +=4;
          output << "Y Offset : " << yoffset << std::endl;;

          float zoffset = 0; memcpy(&zoffset, ptr, 4); ptr +=4;
          output << "Z Offset : " << zoffset << std::endl;
        }
      }
    }   // next dataset

    output <<"End Packet\n-------------" << std::endl;
  }
  else
  {
    output << "Unrecognized Packet Type." << std::endl;
  }

  if (this->verbose)
  {
    std::cout << output.str();
  }
}

/////////////////////////////////////////////////
bool OptitrackPlugin::TimecodeStringify(unsigned int _inTimecode,
  unsigned int _inTimecodeSubframe, char * /*_buffer*/, int /*_bufferSize*/)
{
  bool bValid;
  int hour, minute, second, frame, subframe;
  bValid = this->DecodeTimecode(_inTimecode, _inTimecodeSubframe,
    &hour, &minute, &second, &frame, &subframe);

  return bValid;
}

/////////////////////////////////////////////////
bool OptitrackPlugin::DecodeTimecode(unsigned int _inTimecode,
  unsigned int _inTimecodeSubframe, int *_hour, int *_minute, int *_second,
  int *_frame, int *_subframe)
{
  bool bValid = true;

  *_hour     = (_inTimecode >> 24) & 255;
  *_minute   = (_inTimecode >> 16) & 255;
  *_second   = (_inTimecode >> 8) & 255;
  *_frame    = _inTimecode & 255;
  *_subframe = _inTimecodeSubframe;

  return bValid;
}

/////////////////////////////////////////////////
void OptitrackPlugin::SetWorld(const std::string &_world)
{
  this->world = _world;
}
