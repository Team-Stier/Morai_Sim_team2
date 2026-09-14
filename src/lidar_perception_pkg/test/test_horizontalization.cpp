#include "horizontalization.h"
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
