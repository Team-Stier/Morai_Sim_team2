#include "ground_filter.h"
#include <Eigen/QR>
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <map>
#include <random>
#include <stdexcept>
namespace lidar_perception {
void GroundConfig::validate() const {
  for (double v : {cell_size,sample_size,max_range,expected_z,seed_radius,seed_tolerance,
      max_slope_deg,fit_distance,remove_distance,min_span,min_inlier_ratio,continuity_distance})
    if (!std::isfinite(v)) throw std::invalid_argument("nonfinite ground filter parameter");
  if (cell_size<=0 || sample_size<=0 || sample_size>cell_size || max_range<=0 ||
      max_range/cell_size>100 || max_range/sample_size>1000 || seed_radius<=0 ||
      seed_tolerance<=0 || max_slope_deg<=0 || max_slope_deg>=45 || fit_distance<=0 ||
      remove_distance<fit_distance || min_span<=0 || min_span>3*cell_size ||
      min_inlier_ratio<=0 || min_inlier_ratio>1 || continuity_distance<=0 ||
      min_support<3 || iterations<1 || iterations>1000)
    throw std::invalid_argument("invalid ground filter parameters");
}
namespace {
using Key=std::pair<int,int>;
struct Model {
  Eigen::Vector3d plane;
  double xmin=0,xmax=0,ymin=0,ymax=0;
  bool accepted=false;
};
double height(const Model& m,double x,double y) {return m.plane.dot(Eigen::Vector3d(x,y,1));}
bool fit(const std::vector<Eigen::Vector3d>& pts,const GroundConfig& c,Model& out) {
  if (pts.size()<static_cast<size_t>(c.min_support)) return false;
  const double slope=std::tan(c.max_slope_deg*std::acos(-1)/180);
  auto inliers=[&](const Eigen::Vector3d& p) {
    std::vector<size_t> ids;
    for(size_t i=0;i<pts.size();++i)
      if(std::abs(pts[i].z()-p.dot(Eigen::Vector3d(pts[i].x(),pts[i].y(),1)))<=c.fit_distance) ids.push_back(i);
    return ids;
  };
  std::mt19937 rng(731); std::uniform_int_distribution<size_t> pick(0,pts.size()-1);
  std::vector<size_t> best;
  for(int k=0;k<c.iterations;++k) {
    const auto &a=pts[pick(rng)],&b=pts[pick(rng)],&d=pts[pick(rng)];
    const Eigen::Vector3d n=(b-a).cross(d-a);
    if(std::abs(n.z())<1e-8 || n.head<2>().norm()/std::abs(n.z())>slope) continue;
    Eigen::Vector3d p(-n.x()/n.z(),-n.y()/n.z(),n.dot(a)/n.z());
    auto ids=inliers(p); if(ids.size()>best.size()) best=std::move(ids);
  }
  for(int pass=0;pass<2;++pass) {
    if(best.size()<static_cast<size_t>(c.min_support) || double(best.size())/pts.size()<c.min_inlier_ratio) return false;
    Eigen::MatrixXd a(best.size(),3); Eigen::VectorXd z(best.size());
    for(size_t i=0;i<best.size();++i) {a.row(i)<<pts[best[i]].x(),pts[best[i]].y(),1;z[i]=pts[best[i]].z();}
    Eigen::ColPivHouseholderQR<Eigen::MatrixXd> qr(a);
    if(qr.rank()!=3) return false;
    out.plane=qr.solve(z);
    if(!out.plane.allFinite() || out.plane.head<2>().norm()>slope) return false;
    best=inliers(out.plane);
  }
  if(best.size()<static_cast<size_t>(c.min_support) || double(best.size())/pts.size()<c.min_inlier_ratio) return false;
  out.xmin=out.ymin=std::numeric_limits<double>::infinity();out.xmax=out.ymax=-out.xmin;
  for(auto i:best) {out.xmin=std::min(out.xmin,pts[i].x());out.xmax=std::max(out.xmax,pts[i].x());out.ymin=std::min(out.ymin,pts[i].y());out.ymax=std::max(out.ymax,pts[i].y());}
  return out.xmax-out.xmin>=c.min_span && out.ymax-out.ymin>=c.min_span;
}
}
GroundResult removeGround(const Cloud& input,const GroundConfig& c) {
  c.validate(); GroundResult r; r.cloud=input; if(!c.enabled) return r;
  // A bounded lower envelope gives dense walls no more votes than road cells.
  std::map<Key,Eigen::Vector3d> samples;
  for(const auto& p:input) {
    if(!std::isfinite(p.x)||!std::isfinite(p.y)||!std::isfinite(p.z)||std::hypot(p.x,p.y)>c.max_range) continue;
    Key key{int(std::floor(p.x/c.sample_size)),int(std::floor(p.y/c.sample_size))};
    auto it=samples.find(key); if(it==samples.end() || p.z<it->second.z()) samples[key]=p.getVector3fMap().cast<double>();
  }
  std::map<Key,std::vector<Eigen::Vector3d>> patches;
  for(const auto& kv:samples) {
    const auto& p=kv.second;int x=std::floor(p.x()/c.cell_size),y=std::floor(p.y()/c.cell_size);
    for(int dx=-1;dx<=1;++dx)for(int dy=-1;dy<=1;++dy)patches[{x+dx,y+dy}].push_back(p);
  }
  std::map<Key,Model> models;
  for(const auto& kv:patches) {
    Model m; if(!fit(kv.second,c,m)) continue;
    const double x=(kv.first.first+.5)*c.cell_size,y=(kv.first.second+.5)*c.cell_size;
    // Nearby broad planes must agree with the sensor-to-road height prior.
    m.accepted=std::hypot(x,y)<=c.seed_radius && std::abs(m.plane.z()-c.expected_z)<=c.seed_tolerance;
    models[kv.first]=m;
  }
  // Propagate only across continuous neighboring surfaces; no scan-to-scan cache.
  bool changed=true;
  while(changed) {
    changed=false;
    for(auto& kv:models) {
      if(kv.second.accepted) continue;
      for(const Key d : {Key{-1,0},Key{1,0},Key{0,-1},Key{0,1}}) {
        auto nb=models.find({kv.first.first+d.first,kv.first.second+d.second});
        if(nb==models.end() || !nb->second.accepted) continue;
        const double x=(kv.first.first+.5+.5*d.first)*c.cell_size,y=(kv.first.second+.5+.5*d.second)*c.cell_size;
        bool continuous=true;
        for(double offset : {-c.cell_size*.5,c.cell_size*.5}) {
          const double xx=x+(d.first==0?offset:0),yy=y+(d.second==0?offset:0);
          if(std::abs(height(kv.second,xx,yy)-height(nb->second,xx,yy))>c.continuity_distance) continuous=false;
        }
        if(continuous) {kv.second.accepted=true;changed=true;break;}
      }
    }
  }
  for(const auto& kv:models) if(kv.second.accepted) ++r.supported_cells;
  for(auto& p:r.cloud) {
    if(!std::isfinite(p.x)||!std::isfinite(p.y)||!std::isfinite(p.z)||std::hypot(p.x,p.y)>c.max_range) continue;
    auto it=models.find({int(std::floor(p.x/c.cell_size)),int(std::floor(p.y/c.cell_size))});
    if(it==models.end() || !it->second.accepted) continue;
    const auto& m=it->second;
    if(p.x<m.xmin || p.x>m.xmax || p.y<m.ymin || p.y>m.ymax) continue;
    if(std::abs(p.z-height(m,p.x,p.y))<=c.remove_distance) {
      p.x=p.y=p.z=std::numeric_limits<float>::quiet_NaN();r.cloud.is_dense=false;++r.removed;
    }
  }
  return r;
}
}
