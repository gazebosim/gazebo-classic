/*
 * Copyright (C) 2015 Open Source Robotics Foundation
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

#include <gtest/gtest.h>

#include "test_config.h"

#include "gazebo/common/SVGLoader.hh"

#include "test/util.hh"

using namespace gazebo;

class SVGLoader : public gazebo::testing::AutoLogFixture { };


unsigned int samples = 10;
std::string foutput = "";

/////////////////////////////////////////////////
TEST_F(SVGLoader, LoadPaths)
{
  common::SVGLoader loader(samples);
  std::vector<common::SVGPath> paths;

  // bad path
  bool success;
  std::string bad = "/not/a/file.svg";
  success = loader.Parse(bad, paths);
  EXPECT_EQ(true, success);

  std::string filePath = std::string(PROJECT_SOURCE_PATH);
  filePath += "/test/data/loader.svg";
  loader.Parse(filePath, paths);

  // useful to see the points on screen
  // loader.DumpPaths(paths, std::cout);

  // or in a file
  if (!foutput.empty())
  {
    std::ofstream out(foutput.c_str() );
    loader.DumpPaths(paths, out);
    out.close();
  }

  // the test file has 3 paths inside
  EXPECT_EQ(3u, paths.size());
  common::SVGPath &a = paths[0];
  EXPECT_EQ("letterA", a.id);

  // the letter A has 2 subpaths:
  EXPECT_EQ(2u, a.subpaths.size());

  // The hole of A
  // 4 commands
  EXPECT_EQ(4u, a.subpaths[0].size());
  // 4 points
  EXPECT_EQ(4u, a.polylines[0].size());
  // THe A contour has 9
  EXPECT_EQ(9u, a.polylines[1].size());

  // see what's going on
  loader.DumpPaths(paths, std::cout);

  // the second path
  common::SVGPath &p2 = paths[1];
  EXPECT_EQ(1u, p2.subpaths.size());
  EXPECT_EQ("path2984", p2.id);

  // 8 commands
  EXPECT_EQ(8u, p2.subpaths[0].size());
  // since it has splines, there are more
  // points than commands
  EXPECT_EQ(67u, p2.polylines[0].size());
}

/////////////////////////////////////////////////
std::vector<std::string> &split(const std::string &_s,
                                char _delim,
                                std::vector<std::string> &_elems)
{
  std::stringstream ss(_s);
  std::string item;
  while (std::getline(ss, item, _delim))
  {
    _elems.push_back(item);
  }
  return _elems;
}

void strv(const std::vector<std::string> &strs)
{
  for(auto s : strs)
  {
    gzmsg << " \"" << s << "\"" <<  std::endl;
  }

}

math::Matrix3 GetTransformationMatrix(const std::string &_transform, const std::string &_numbersStr)
{

  std::vector<std::string> numbers;
  split(_numbersStr, ',', numbers);

  // how to unpack the values into 3x3 matrices
  // http://www.w3.org/TR/SVG/coords.html#TransformAttribute
  if(_transform.find("matrix") != std::string::npos)
  {
    gzmsg << "matrix" << std::endl;
    double v00 = stod(numbers[0]);
    double v10 = stod(numbers[1]);
    double v01 = stod(numbers[2]);
    double v11 = stod(numbers[3]);
    double v02 = stod(numbers[4]);
    double v12 = stod(numbers[5]);
    math::Matrix3 m(v00, v01, v02, v10, v11, v12, 0, 0, 1);
    return m;
  }

  if(_transform.find("skewX") != std::string::npos)
  {
    gzmsg << "skewX" << std::endl;
    
    math::Matrix3 m;
    return m;
  }

  if(_transform.find("skewY") != std::string::npos)
  {
    gzmsg << "skewY" << std::endl;
    math::Matrix3 m;
    return m;
  }  

  if(_transform.find("scale") != std::string::npos)
  {
    gzmsg << "scale" << std::endl;
    math::Matrix3 m;
    return m;
  }

  if(_transform.find("translate") != std::string::npos)
  {
    gzmsg << "translate" << std::endl;
    math::Matrix3 m;
    return m;
  }

  if(_transform.find("rotate") != std::string::npos)
  {
    gzmsg << "rotate" << std::endl;
    math::Matrix3 m;
    return m;
  }
    

  gzwarn << "Unsupported transformation: " << &_transform << std::endl;
  math::Matrix3 m;
  return m;
}


/////////////////////////////////////////////////
TEST_F(SVGLoader, ChainedTransforms)
{

//  std::string str = "matrix(0,0.55669897,-0.55669897,0,194.55441,-149.50402) translate(0,0) scale(1,3) skewX(22)";
//  common::SVGTransform tx(str);
  EXPECT_EQ(42, 41);
  
}



/*
/////////////////////////////////////////////////
TEST_F(SVGLoader, ChainedTransforms)
{

  std::string str = "matrix(0,0.55669897,-0.55669897,0,194.55441,-149.50402) translate(0,0) scale(1,3) skewX(22)";
 
  std::vector<math::Matrix3> transforms;
  
  std::vector<std::string> transformsStr;
  split(str, ')', transformsStr);

  for(size_t i=0; i < transformsStr.size(); ++i)
  {
    std::vector<std::string> tx;
    split(transformsStr[i], '(', tx);
    math::Matrix3 m = GetTransformationMatrix(tx[0], tx[1]);
    transforms.push_back(m);
  }
}


/////////////////////////////////////////////////
TEST_F(SVGLoader, Transforms)
{

  std::string str = "matrix(0,0.55669897,-0.55669897,0,194.55441,-149.50402)";

  size_t found1, found2;
  std::string mtrx("matrix(");
  found1 = str.find(mtrx) + mtrx.length();
  found2 = str.find(")");
  std::string matrixStr = str.substr(found1, found2 -found1);
  std::vector<std::string> numbers;
  split(matrixStr, ',', numbers);
  double matrix[6];

  for (size_t i =0; i < 6; i++)
  {
    matrix[i] = stod(numbers[i]);
  }

  //  for (size_t i =0; i < 6; i++) gzmsg << " [" << i << "] " << matrix[i] << std::endl;

  EXPECT_DOUBLE_EQ(matrix[0], 0);
  EXPECT_DOUBLE_EQ(matrix[1], 0.55669897);
  EXPECT_DOUBLE_EQ(matrix[2], -0.55669897);
  EXPECT_DOUBLE_EQ(matrix[3], 0);
  EXPECT_DOUBLE_EQ(matrix[4], 194.55441);
  EXPECT_DOUBLE_EQ(matrix[5], -149.50402);
}
*/
/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
