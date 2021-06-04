/*
 * SPDX-License-Identifier: BSD-3-Clause
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2014-, Open Perception Inc.
 *
 *  All rights reserved
 */

#include <pcl/test/gtest.h>
#include <pcl/pcl_tests.h>
#include <pcl/point_types.h>

using namespace pcl;
using namespace pcl::test;

TEST (PointTypeConstruction, PointXYZDefaultConstruction)
{
  static_assert(std::is_default_constructible<pcl::PointXYZ>::value, "");
  constexpr const pcl::PointXYZ pt;
  static_assert(pt.data[0] == 0.0f, "");
  static_assert(pt.data[1] == 0.0f, "");
  static_assert(pt.data[2] == 0.0f, "");
  static_assert(pt.data[3] == 1.0f, "");
}

TEST (PointTypeConstruction, PointXYZThreeScalarsConstruction)
{
  constexpr const pcl::PointXYZ pt(2.0f, 3.0f, 4.0f);
  static_assert(pt.data[0] == 2.0f, "");
  static_assert(pt.data[1] == 3.0f, "");
  static_assert(pt.data[2] == 4.0f, "");
  static_assert(pt.data[3] == 1.0f, "");
}

TEST (PointTypeConstruction, PointXYZIDefaultConstruction)
{
  static_assert(std::is_default_constructible<pcl::PointXYZI>::value, "");
  constexpr const pcl::PointXYZI pt;
  static_assert(pt.data[0] == 0.0f, "");
  static_assert(pt.data[1] == 0.0f, "");
  static_assert(pt.data[2] == 0.0f, "");
  static_assert(pt.data[3] == 1.0f, "");
  static_assert(pt.intensity == 0.0f, "");
}

TEST (PointTypeConstruction, PointXYZIFourScalarsConstruction)
{
  constexpr const pcl::PointXYZI pt(2.0f, 3.0f, 4.0f, 5.0f);
  static_assert(pt.data[0] == 2.0f, "");
  static_assert(pt.data[1] == 3.0f, "");
  static_assert(pt.data[2] == 4.0f, "");
  static_assert(pt.data[3] == 1.0f, "");
  static_assert(pt.intensity == 5.0f, "");
}


template <typename T> class PointTypeConstexprConstructorTest : public testing::Test { };
using AllPointTypes = ::testing::Types<pcl::PointXYZ, pcl::RGB, pcl::Intensity, pcl::PointXYZI>;
TYPED_TEST_SUITE (PointTypeConstexprConstructorTest, AllPointTypes);

TYPED_TEST (PointTypeConstexprConstructorTest, ConstexprDefaultConstructionTests)
{
  constexpr const TypeParam pt;
}

int
main (int argc, char** argv)
{
  testing::InitGoogleTest (&argc, argv);
  return (RUN_ALL_TESTS ());
}
