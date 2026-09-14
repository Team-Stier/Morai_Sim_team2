#include "detector.h"
#include "cluster_points.h"
#include <pcl/filters/voxel_grid.h>
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
TEST(Detector, OriginalMembershipSurvivesVoxelAveragingAndNoiseFiltering) {
  Config c; Cloud cloud;blob(cloud,2,0);
  const size_t originals=cloud.size();
  for (size_t i=0;i<originals;++i) {
    auto p=cloud[i];p.x+=.001f;cloud.push_back(p);
  }
  cloud.push_back(Point(20,0,0));cloud.push_back(Point(-5,0,0));
  const auto result=detect(cloud,c);
  ASSERT_EQ(1u,result.boxes.size());ASSERT_EQ(cloud.size(),result.raw_cluster_ids.size());
  for (size_t i=0;i<originals*2;++i) EXPECT_EQ(0,result.raw_cluster_ids[i]);
  EXPECT_EQ(-1,result.raw_cluster_ids[originals*2]);
  EXPECT_EQ(-1,result.raw_cluster_ids[originals*2+1]);
  Cloud::Ptr roi(new Cloud(cloud));roi->resize(cloud.size()-1);
  pcl::VoxelGrid<Point> filter;filter.setInputCloud(roi);filter.setLeafSize(c.leaf_size,c.leaf_size,c.leaf_size);
  Cloud reference;filter.filter(reference);
  ASSERT_EQ(reference.size(),result.filtered->size());
  for (size_t i=0;i<reference.size();++i)
    EXPECT_NEAR(0,(reference[i].getVector3fMap()-(*result.filtered)[i].getVector3fMap()).norm(),1e-6);
}
TEST(ClusterPoints, CopiesRawRecordsIncludingRowPaddingAndAddsProvenance) {
  sensor_msgs::PointCloud2 raw;
  raw.header.frame_id="lidar_link";raw.header.stamp=ros::Time(10,42);
  raw.width=2;raw.height=2;raw.point_step=20;raw.row_step=48;
  raw.data.resize(96);
  for(size_t i=0;i<raw.data.size();++i)raw.data[i]=i;
  const auto out=clusterPoints(raw,{-1,2,0,-1});
  EXPECT_EQ(raw.header,out.header);EXPECT_EQ(2u,out.width);EXPECT_EQ(28u,out.point_step);
  EXPECT_TRUE(std::equal(raw.data.begin()+20,raw.data.begin()+40,out.data.begin()));
  EXPECT_TRUE(std::equal(raw.data.begin()+48,raw.data.begin()+68,out.data.begin()+28));
  EXPECT_EQ(2,out.data[20]);EXPECT_EQ(1,out.data[24]);
  EXPECT_EQ(0,out.data[48]);EXPECT_EQ(2,out.data[52]);
  EXPECT_EQ("cluster_id",out.fields[0].name);EXPECT_EQ("source_index",out.fields[1].name);
  EXPECT_EQ(0u,clusterPoints(raw,{-1,-1,-1,-1}).width);
  EXPECT_THROW(clusterPoints(raw,{0}),std::invalid_argument);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
