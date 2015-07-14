/* Original work copyright Paul Bovbel
 * Modified work copyright 2015 Institute of Digital Communication Systems - Ruhr-University Bochum
 * Modified by: Adrian Bauer
 *
 * This program is free software; you can redistribute it and/or modify it under the terms of
 * the GNU General Public License as published by the Free Software Foundation;
 * either version 3 of the License, or (at your option) any later version.
 * This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;
 * without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 * You should have received a copy of the GNU General Public License along with this program;
 * if not, see <http://www.gnu.org/licenses/>.
 *
 * This file is an extended version of the original file taken from the frontier_exploration ROS package by Paul Bovbel
 **/

#include "ros/ros.h"
#include "costmap_2d/costmap_2d.h"
#include "heatmap/geometry_tools.h"

#include <gtest/gtest.h>

class PointInPolygonTest : public ::testing::Test
{
protected:
  virtual void SetUp()
  {
    //make upright hourglass polygon
    geometry_msgs::Point32 point;
    point.x = -1;
    point.y = 1;
    polygon_.points.push_back(point);
    point.x = 1;
    point.y = 1;
    polygon_.points.push_back(point);
    point.x = -1;
    point.y = -1;
    polygon_.points.push_back(point);
    point.x = 1;
    point.y = -1;
    polygon_.points.push_back(point);
  }
  geometry_msgs::Polygon polygon_;
};

class BottomTopPointPolygonTest : public ::testing::Test
{
protected:
  virtual void SetUp()
  {
    //make upright hourglass polygon
    geometry_msgs::Point32 point;
    point.x = -1;
    point.y = 1;
    polygon_.points.push_back(point);
    point.x = 1;
    point.y = 1;
    polygon_.points.push_back(point);
    point.x = -1;
    point.y = -1;
    polygon_.points.push_back(point);
    point.x = 1;
    point.y = -1;
    polygon_.points.push_back(point);
  }
  geometry_msgs::Polygon polygon_;
};

TEST_F(PointInPolygonTest, outside)
{
  geometry_msgs::Point point;
  point.x = 0.5;
  point.y = 0;
  ASSERT_FALSE(heatmap::pointInPolygon(point, polygon_));
  point.x = -0.5;
  point.y = 0;
  ASSERT_FALSE(heatmap::pointInPolygon(point, polygon_));
  point.x = 0;
  point.y = 1.1;
  ASSERT_FALSE(heatmap::pointInPolygon(point, polygon_));
  point.x = 0;
  point.y = -1.1;
  ASSERT_FALSE(heatmap::pointInPolygon(point, polygon_));
}

TEST_F(PointInPolygonTest, inside)
{
  geometry_msgs::Point point;
  point.x = 0;
  point.y = 0.5;
  ASSERT_TRUE(heatmap::pointInPolygon(point, polygon_));
  point.x = 0;
  point.y = 0.5;
  ASSERT_TRUE(heatmap::pointInPolygon(point, polygon_));
}

TEST_F(BottomTopPointPolygonTest, bottom)
{
  geometry_msgs::Point32 p;
  p.x = -1;
  p.y = -1;

  ASSERT_EQ(heatmap::bottomLeftPointPolygon<geometry_msgs::Point32>(polygon_).x, p.x);
  ASSERT_EQ(heatmap::bottomLeftPointPolygon<geometry_msgs::Point32>(polygon_).y, p.y);
}

TEST_F(BottomTopPointPolygonTest, top)
{
  geometry_msgs::Point32 p;
  p.x = 1;
  p.y = 1;

  ASSERT_EQ(heatmap::topRightPointPolygon<geometry_msgs::Point32>(polygon_).x, p.x);
  ASSERT_EQ(heatmap::topRightPointPolygon<geometry_msgs::Point32>(polygon_).y, p.y);
}

TEST(PointsAdjacentTest, different)
{
  geometry_msgs::Point a, b;
  a.x = 1;
  ASSERT_FALSE(heatmap::pointsNearby(a, b, 0));
  ASSERT_FALSE(heatmap::pointsNearby(a, b, 0.1));
  ASSERT_TRUE(heatmap::pointsNearby(a, b, 1));
}

TEST(PointsAdjacentTest, identical)
{
  geometry_msgs::Point a, b;
  a.x = 1;
  b.x = 1;
  ASSERT_TRUE(heatmap::pointsNearby(a, b, 0));
  ASSERT_TRUE(heatmap::pointsNearby(a, b, 0.1));
  ASSERT_TRUE(heatmap::pointsNearby(a, b, 1));
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
