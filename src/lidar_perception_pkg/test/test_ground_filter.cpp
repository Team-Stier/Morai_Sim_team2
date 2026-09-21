#include "ground_filter.h"
#include "horizontalization.h"
#include <gtest/gtest.h>
#include <cmath>
#include <limits>
using namespace lidar_perception;
namespace {
Cloud road(double slope=0) {
  Cloud c;
  for(int x=-20;x<=60;++x)for(int y=-20;y<=20;++y)
    c.push_back(Point(x*.5,y*.5,-1.9+slope*x*.5));
  return c;
}
size_t finite(const Cloud& c) {size_t n=0;for(const auto& p:c)if(std::isfinite(p.x))++n;return n;}
GroundConfig config() {GroundConfig c;c.enabled=true;return c;}
}
TEST(Ground, FlatAndBothSlopesRemoveRoadPreserveTenCentimeterObstacle) {
  for(double slope : {0.,.11,-.11}) {
    auto c=road(slope);const size_t n=c.size();
    for(int i=0;i<10;++i)for(int j=0;j<10;++j) c.push_back(Point(5+i*.05,2+j*.05,-1.9+slope*(5+i*.05)+.10));
    const auto r=removeGround(c,config());EXPECT_GT(r.removed,n*.95);
    ASSERT_EQ(c.size(),r.cloud.size());
    for(size_t i=n;i<c.size();++i)EXPECT_TRUE(std::isfinite(r.cloud[i].x));
    const auto again=removeGround(c,config());EXPECT_EQ(r.removed,again.removed);
  }
}
TEST(Ground, UnsupportedObjectOnlyCollinearAndEmptyAreRetained) {
  Cloud c;
  for(int i=0;i<30;++i)for(int j=0;j<30;++j)c.push_back(Point(5+i*.03,2+j*.03,-1.5));
  EXPECT_EQ(c.size(),finite(removeGround(c,config()).cloud));
  c.clear();for(int i=0;i<100;++i)c.push_back(Point(i*.1,0,-1.9));
  EXPECT_EQ(c.size(),finite(removeGround(c,config()).cloud));
  EXPECT_EQ(0u,removeGround(Cloud(),config()).removed);
}
TEST(Ground, RaisedDisconnectedSurfaceIsNotPropagatedAsRoad) {
  Cloud c;
  for(int x=-20;x<=60;++x)for(int y=-20;y<=20;++y)
    c.push_back(Point(x*.5,y*.5,x<20?-1.9:-1.0));
  const auto r=removeGround(c,config());
  for(size_t i=0;i<c.size();++i)if(c[i].x>=12)EXPECT_TRUE(std::isfinite(r.cloud[i].x));
}
TEST(Ground, DisabledNonfiniteOutOfRangeAndInvalidConfiguration) {
  auto c=road(); c.push_back(Point(100,0,-1.9));c.push_back(Point(NAN,0,0));
  GroundConfig cfg;EXPECT_EQ(c.size()-1,finite(removeGround(c,cfg).cloud));
  cfg.enabled=true;auto r=removeGround(c,cfg);EXPECT_TRUE(std::isfinite(r.cloud[c.size()-2].x));EXPECT_FALSE(std::isfinite(r.cloud.back().x));
  cfg.sample_size=0;EXPECT_THROW(removeGround(c,cfg),std::invalid_argument);
  cfg=config();cfg.remove_distance=NAN;EXPECT_THROW(removeGround(c,cfg),std::invalid_argument);
}
TEST(Ground, LevelThenRemoveKeepsOriginalIndicesForClusterConsumer) {
  auto level=road(.11);const auto n=level.size();
  for(int i=0;i<5;++i)for(int j=0;j<5;++j)level.push_back(Point(6+i*.06,1+j*.06,-1.0));
  const Eigen::Matrix3d rotation=Eigen::AngleAxisd(.12,Eigen::Vector3d::UnitY()).toRotationMatrix();
  const auto raw=rotateCloud(level,rotation.transpose());
  auto r=removeGround(rotateCloud(raw,rotation),config());Config detector;detector.z_min=-3;detector.x_min=-20;
  auto out=detect(r.cloud,detector);ASSERT_EQ(level.size(),out.raw_cluster_ids.size());
  for(size_t i=n;i<level.size();++i)EXPECT_GE(out.raw_cluster_ids[i],0);
  size_t road_output=0;for(size_t i=0;i<n;++i)road_output+=out.raw_cluster_ids[i]>=0;
  EXPECT_LT(road_output,50u);
}
