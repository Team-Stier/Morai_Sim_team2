#include "horizontalization.h"
#include "self_filter.h"
#include <gtest/gtest.h>
#include <limits>

using namespace lidar_perception;
static Eigen::Quaterniond attitude(double roll, double pitch, double yaw=0) {
  return Eigen::Quaterniond(Eigen::AngleAxisd(yaw,Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(pitch,Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(roll,Eigen::Vector3d::UnitX()));
}
TEST(Horizontalization, RecoversTiltedPlaneWithMountAndPreservesHeading) {
  const auto q=attitude(.2,-.15,1.4), mount=attitude(-.04,.06,.1);
  const auto rotation=levelRotation(q,mount);
  const Eigen::Matrix3d expected=attitude(.2,-.15).toRotationMatrix()*mount.toRotationMatrix();
  EXPECT_TRUE(rotation.isApprox(expected,1e-12));
  Cloud tilted;
  for (int x=-5;x<=5;++x) for (int y=-5;y<=5;++y) {
    const Eigen::Vector3d p=expected.transpose()*Eigen::Vector3d(x,y,-1.5);
    tilted.push_back(Point(p.x(),p.y(),p.z()));
  }
  const auto level=rotateCloud(tilted,rotation);
  for (const auto& p : level) EXPECT_NEAR(-1.5,p.z,1e-6);
  const auto recovered=rotateCloud(level,rotation.transpose());
  for (size_t i=0;i<tilted.size();++i)
    EXPECT_NEAR(0,(recovered[i].getVector3fMap()-tilted[i].getVector3fMap()).norm(),1e-6);
  EXPECT_TRUE(levelRotation(attitude(0,0,2.1),Eigen::Quaterniond::Identity()).isApprox(Eigen::Matrix3d::Identity(),1e-12));
}
TEST(Horizontalization, InverseBoxEnclosesAllCorners) {
  const auto rotation=levelRotation(attitude(.2,-.15),Eigen::Quaterniond::Identity());
  Box b; b.center=Point(8,2,-.4); b.size=Point(4,2,1.5); b.point_count=32;
  const auto box=boxInSensorFrame(b,rotation);
  EXPECT_EQ(32u,box.point_count);
  for (int x : {-1,1}) for (int y : {-1,1}) for (int z : {-1,1}) {
    const Eigen::Vector3d p=rotation.transpose()*Eigen::Vector3d(8+x*2,2+y,-.4+z*.75);
    for (int i=0;i<3;++i) EXPECT_LE(std::abs(p[i]-box.center.getVector3fMap()[i]),box.size.getVector3fMap()[i]/2+1e-6);
  }
}
TEST(Horizontalization, InterpolatesShortestQuaternionPathAndNeverExtrapolates) {
  AttitudeHistory history;
  EXPECT_TRUE(history.insert(1000000000,attitude(.1,.2,3.13),2));
  auto q=attitude(.1,.2,-3.13); q.coeffs()*=-1;
  EXPECT_TRUE(history.insert(1100000000,q,2));
  Eigen::Quaterniond result;
  EXPECT_TRUE(history.interpolate(1050000000,.101,result));
  EXPECT_TRUE(levelRotation(result,Eigen::Quaterniond::Identity()).isApprox(attitude(.1,.2).toRotationMatrix(),1e-6));
  EXPECT_FALSE(history.interpolate(999999999,.101,result));
  EXPECT_FALSE(history.interpolate(1100000001,.101,result));
  EXPECT_FALSE(history.interpolate(1050000000,.05,result));
  EXPECT_FALSE(history.insert(1000000000,q,2));
  EXPECT_FALSE(history.insert(1200000000,Eigen::Quaterniond(0,0,0,0),2));
  history.clear();
  EXPECT_FALSE(history.interpolate(1050000000,.101,result));
}
TEST(Horizontalization, RejectsInvalidAttitude) {
  EXPECT_THROW(levelRotation(Eigen::Quaterniond(0,0,0,0),Eigen::Quaterniond::Identity()),std::invalid_argument);
  EXPECT_THROW(levelRotation(attitude(0,1.5707963267948966),Eigen::Quaterniond::Identity()),std::invalid_argument);
}

TEST(Horizontalization, BrakingPitchRejectsRoadBeforeClusteringAndRetainsObstacle) {
  Config config;
  // The road is below the configured lower ROI boundary; leveling does not
  // claim to segment a road that is itself inside the accepted height range.
  const auto rotation=levelRotation(attitude(.08,.12,1.1),Eigen::Quaterniond::Identity());
  Cloud level;
  for (int x=0;x<4;++x) for (int y=0;y<4;++y)
    level.push_back(Point(10+x*.05f,y*.05f,-1.65f));
  const size_t ground_count=level.size();
  for (int x=0;x<4;++x) for (int y=0;y<4;++y) for (int z=0;z<2;++z)
    level.push_back(Point(6+x*.05f,2+y*.05f,-.4f+z*.05f));
  const Cloud raw=rotateCloud(level,rotation.transpose());
  const auto uncorrected=detect(raw,config);
  ASSERT_EQ(2u,uncorrected.boxes.size());
  for (size_t i=0;i<ground_count;++i) EXPECT_GE(uncorrected.raw_cluster_ids[i],0);
  const auto corrected=detect(rotateCloud(raw,rotation),config);
  ASSERT_EQ(1u,corrected.boxes.size());
  for (size_t i=0;i<ground_count;++i) EXPECT_EQ(-1,corrected.raw_cluster_ids[i]);
  for (size_t i=ground_count;i<level.size();++i) EXPECT_EQ(0,corrected.raw_cluster_ids[i]);
  // A consumer must receive the original sensor geometry, not a second leveling.
  const auto box=boxInSensorFrame(corrected.boxes[0],rotation);
  const Eigen::Vector3d expected=rotation.transpose()*Eigen::Vector3d(6.075,2.075,-.375);
  EXPECT_NEAR(expected.x(),box.center.x,1e-5);
  EXPECT_NEAR(expected.y(),box.center.y,1e-5);
  EXPECT_NEAR(expected.z(),box.center.z,1e-5);
}

TEST(SelfFilter, InclusiveBodyMaskPreservesIndicesAndNearbyExterior) {
  SelfFilterConfig c;
  Cloud raw;
  raw.push_back(Point(-1.5,0,-.35));
  raw.push_back(Point(c.x_min,c.y_min,c.z_min));
  raw.push_back(Point(c.x_max,c.y_max,c.z_max));
  raw.push_back(Point(c.x_min-.01,0,0));
  raw.push_back(Point(c.x_max+.01,0,0));
  raw.push_back(Point(0,c.y_min-.01,0));
  raw.push_back(Point(0,c.y_max+.01,0));
  raw.push_back(Point(0,0,c.z_min-.01));
  raw.push_back(Point(0,0,c.z_max+.01));
  const auto masked=maskSelfReturns(raw,c);
  ASSERT_EQ(raw.size(),masked.size());
  for (size_t i=0;i<3;++i) EXPECT_TRUE(std::isnan(masked[i].x));
  for (size_t i=3;i<raw.size();++i)
    EXPECT_EQ(raw[i].getVector3fMap(),masked[i].getVector3fMap());
  EXPECT_TRUE(std::isfinite(raw[0].x));
  c.enabled=false;
  EXPECT_EQ(raw[0].x,maskSelfReturns(raw,c)[0].x);
  c.x_min=c.x_max;
  EXPECT_THROW(maskSelfReturns(raw,c),std::invalid_argument);
}
TEST(SelfFilter, MaskBeforeLevelingRemovesOnlyBodyClusterUnderTilt) {
  Cloud raw;
  for (int i=0;i<4;++i) for (int j=0;j<4;++j)
    raw.push_back(Point(-1.8f+i*.05f,-.1f+j*.05f,-.35f));
  const size_t body_count=raw.size();
  for (int i=0;i<4;++i) for (int j=0;j<4;++j)
    raw.push_back(Point(3+i*.05f,2+j*.05f,0));
  Config detection; detection.x_min=-20;
  const auto rotation=levelRotation(attitude(.2,.15,1.1),Eigen::Quaterniond::Identity());
  const auto output=detect(rotateCloud(maskSelfReturns(raw,SelfFilterConfig()),rotation),detection);
  ASSERT_EQ(1u,output.boxes.size());
  for (size_t i=0;i<body_count;++i) EXPECT_EQ(-1,output.raw_cluster_ids[i]);
  for (size_t i=body_count;i<raw.size();++i) EXPECT_EQ(0,output.raw_cluster_ids[i]);
}
