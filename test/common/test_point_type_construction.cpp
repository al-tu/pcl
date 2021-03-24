/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2012, Willow Garage, Inc.
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder(s) nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * $Id:  $
 *
 */

#include <pcl/test/gtest.h>
#include <pcl/pcl_tests.h>
#include <pcl/point_types.h>

using namespace pcl;
using namespace pcl::test;

static constexpr const std::uint8_t uint8_0{};
static constexpr const std::uint8_t uint8_255{};
static constexpr const float float_0{};
static constexpr const float float_1{1.0f};

TEST (PointTypeConstruction, PointXYZDefaultConstruction)
{
  constexpr const pcl::PointXYZ pt;
  static_assert(pt.x == float_0);
  static_assert(pt.y == float_0);
  static_assert(pt.z == float_0);
}

TEST (PointTypeConstruction, PointXYZThreeScalarsConstruction)
{
  constexpr const pcl::PointXYZ pt(float_1, float_1, float_1);
  static_assert(pt.x == float_1);
  static_assert(pt.y == float_1);
  static_assert(pt.z == float_1);

  //constexpr const auto vec4fmap { pt.getVector4fMap() };
  //static_assert(vec4fmap(3) == float_1);
}

/*namespace
{

// TODO make checks helpers for 
//      1. default ctor checks 
//      2. ctor with params checks

template <typename PointT>
constexpr auto is_xyz_default_constructed(PointT &&pt)
{
  return pt.x == float_0 && pt.y == float_0 && pt.z == float_0;
}

}*/

// TODO dont use == with floats
TEST (PointTypeConstruction, PointSurfelDefaultConstruction)
{
  constexpr const pcl::PointSurfel pt;
  static_assert(pt.x == float_0);
  static_assert(pt.y == float_0);
  static_assert(pt.z == float_0);

  static_assert(pt.normal_x == float_0);
  static_assert(pt.normal_y == float_0);
  static_assert(pt.normal_z == float_0);

  static_assert(pt.r == uint8_0);
  static_assert(pt.g == uint8_0);
  static_assert(pt.b == uint8_0);
  static_assert(pt.a == uint8_255);

  static_assert(pt.radius == float_0);
  static_assert(pt.confidence == float_0);
  static_assert(pt.curvature == float_0);
}


int
main (int argc, char** argv)
{
  testing::InitGoogleTest (&argc, argv);
  return (RUN_ALL_TESTS ());
}
