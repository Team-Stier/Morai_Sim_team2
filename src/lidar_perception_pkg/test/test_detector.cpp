#include "detector.h"
#include <gtest/gtest.h>
#include <limits>
#include <stdexcept>

using namespace lidar_perception;
static void blob(Cloud& cloud, float x, float y) {
  for (int i=0;i<4;++i) for (int j=0;j<4;++j) for (int k=0;k<2;++k)
    cloud.push_back(Point(x+i*0.05f,y+j*0.05f,k*0.05f));
}
TEST(Detector, TwoBoxesAndEmptyFrameDoNotRetainObjects) {
  Config c; Cloud cloud; blob(cloud,2,0); blob(cloud,4,2);
  const auto result=detect(cloud,c);
  ASSERT_EQ(2u,result.boxes.size());
  for (const auto& box : result.boxes) {
    EXPECT_NEAR(.15,box.size.x,1e-5); EXPECT_NEAR(.15,box.size.y,1e-5);
    EXPECT_EQ(32u,box.point_count);
  }
  EXPECT_TRUE(detect(Cloud(),c).boxes.empty());
}
TEST(Detector, RoiNonfiniteAndNoiseAreExcluded) {
  Config c; Cloud cloud; blob(cloud,-5,0); blob(cloud,2,0);
  cloud.push_back(Point(7,4,0));
  cloud.push_back(Point(std::numeric_limits<float>::quiet_NaN(),0,0));
  EXPECT_EQ(1u,detect(cloud,c).boxes.size());
}
TEST(Detector, MoreThanOneHundredClustersAreSupported) {
  Config c; c.x_max=30; c.y_max=30; c.min_points=1; c.epsilon=.1;
  Cloud cloud;
  for(int i=0;i<121;++i) cloud.push_back(Point(1+i%11,1+i/11,0));
  EXPECT_EQ(121u,detect(cloud,c).boxes.size());
}
TEST(Detector, InvalidParametersAndOversizedClusters) {
  Config c; Cloud cloud; blob(cloud,2,0); c.max_cluster_size=20;
  EXPECT_TRUE(detect(cloud,c).boxes.empty());
  c.leaf_size=0; EXPECT_THROW(detect(cloud,c),std::invalid_argument);
}
