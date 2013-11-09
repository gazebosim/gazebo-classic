/*
 * Copyright (C) 2012-2013 Open Source Robotics Foundation
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

#include <boost/lexical_cast.hpp>
#include <iostream>
#include <fstream>
#include <string>
#include "LogParser.hh"
#include "ServerFixture.hh"

using namespace gazebo;
using namespace std;

//////////////////////////////////////////////////
LogParser::LogParser(const std::string &_filename)
{
  this->logFile.open(_filename.c_str(), ios::in | ios::binary);
  /// YUV 422 image
  unsigned char timestamp[8];
  unsigned char buffer[320 * 240 * 2];
  long counter = 0;

  // RGB888 image
  long srcSize = 320 * 240 * 2;

  do
  {
    //std::cout << "Get pointer before: " << this->logFile.tellg() << std::endl;
  //  this->logFile.seekg(srcSize * counter);
    //std::cout << "Get pointer after: " << this->logFile.tellg() << std::endl;
    this->logFile.read((char *)timestamp, 8);
    this->logFile.read((char *)buffer, srcSize);
    //std::cout << "Get pointer after2: " << this->logFile.tellg() << std::endl;
    this->ToRGB(buffer, this->rgbImage);
    std::string imgName = "/data/naoDataset/run10/images/" +
      boost::lexical_cast<std::string>(counter) + ".ppm";
    this->SaveCurrentImage(imgName.c_str());
    ++counter;
  } while (this->logFile);
  std::cout << counter << " images loaded" << std::endl;
  
  this->logFile.close();
}

//////////////////////////////////////////////////
LogParser::~LogParser()
{

}

//////////////////////////////////////////////////
void LogParser::GetNextImage()
{

}

//////////////////////////////////////////////////
void LogParser::SaveCurrentImage(const std::string &_filename)
{
  ofstream myFile(_filename.c_str(), ios::out | ios::binary);

  std::string header = "P6 320 240 255\n";

  myFile.write(header.c_str(), header.size());

  for (int i = 0; i < 240; ++i)
    for (int j = 0; j < 320; ++j)
    {
      unsigned char r, g, b;
      int index = (i * 320 + j) * 3;
      r = (unsigned char) this->rgbImage[index];
      g = (unsigned char) this->rgbImage[index + 1];
      b = (unsigned char) this->rgbImage[index + 2];
      //std::cout << "RGB(" << (int)r << "," << (int)g << "," << (int)b << ")" << std::endl;
      myFile.write((char *)&r, 1);
      myFile.write((char *)&g, 1);
      myFile.write((char *)&b, 1);
    }

  //myFile.write((char *)this->rgbImage, 320 * 240 * 3);
  myFile.close();
}

//////////////////////////////////////////////////
void LogParser::ToRGB(unsigned char *_src, unsigned char *_dst)
{
  for (int i = 0; i < 240; ++i)
  {
    for (int j = 0; j < 320; ++j)
    {
      int pixel = i * 320 + j;
      int index = pixel * 2;
      int dstIndex = pixel * 3;
      unsigned char r, g, b;
      if (pixel % 2 == 0)
      {
        this->Yuv2rgb(_src[index], _src[index + 1], _src[index + 3], r, g, b);
      }
      else
      {
        this->Yuv2rgb(_src[index], _src[index - 1], _src[index + 1], r, g, b);
      }
      _dst[dstIndex] = r;
      _dst[dstIndex + 1] = g;
      _dst[dstIndex + 2] = b;
      //std::cout << "(" << (int)r << "," << (int)g << "," << (int)b << ")" << std::endl;
    }
  }
}

//////////////////////////////////////////////////
void LogParser::Yuv2rgb(unsigned char _y, unsigned char _u, unsigned char _v, 
                        unsigned char &_r, unsigned char &_g, unsigned char &_b)
{
  // std::cout << "YUV (" << (int)_y << "," << (int)_u << ","
  //      << (int)_v << ")" << std::endl;
    int r = _y + ((1436 * (_v - 128)) >> 10),
        g = _y - ((354 * (_u - 128) + 732 * (_v - 128)) >> 10),
        b = _y + ((1814 * (_u - 128)) >> 10);

    if(r < 0) r = 0; else if(r > 255) r = 255;
    if(g < 0) g = 0; else if(g > 255) g = 255;
    if(b < 0) b = 0; else if(b > 255) b = 255;

    //Crop RGB
    _r = (unsigned char) r;
    _g = (unsigned char) g;
    _b = (unsigned char) b;

  //std::cout << "RGB (" << (int)_r << "," << (int)_g << ","
  //      << (int)_b << ")" << std::endl;  
}

//////////////////////////////////////////////////
int main(int argc, char **argv)
{
  LogParser *parser = new LogParser("/data/naoDataset/run10/camera_10.strm");
  parser->GetNextImage();

  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
