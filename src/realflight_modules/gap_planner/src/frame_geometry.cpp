#include <gap_planner/frame_geometry.h>
#include <quickhull.hpp>
#include <map>
namespace gap_planner {
namespace {
void addAxis(BodyVertices &axes,Eigen::Vector3d n) {
  if(n.norm()<1e-9) {return;}n.normalize();
  for(const auto &a:axes) {if(std::abs(a.dot(n))>1-1e-8) {return;}}
  axes.push_back(n);
}
}
void validateTask(const std::vector<Gate> &gates,const Eigen::Vector3d &start,
  const Eigen::Vector3d &goal,const Options &o) {
  if(gates.empty() || !start.allFinite() || !goal.allFinite() || (goal-start).norm()<1e-3 ||
    !o.body.allFinite() || (o.body.array()<=0).any() || !std::isfinite(o.margin) || o.margin<0 ||
    !std::isfinite(o.optimization_buffer) || o.optimization_buffer<0) {
    throw std::invalid_argument("Invalid task endpoints/body/margin");
  }
  double diameter=2*o.body.minCoeff();
  if(!o.vertices.empty()) {
    validateVertices(o.vertices);Eigen::Matrix3Xd points(3,o.vertices.size());Eigen::Vector3d center=Eigen::Vector3d::Zero();
    for(size_t i=0;i<o.vertices.size();++i) {points.col(i)=o.vertices[i];center+=o.vertices[i];}
    center/=o.vertices.size();quickhull::QuickHull<double> qh;
    const auto hull=qh.getConvexHull(points.data(),o.vertices.size(),false,true);const auto &idx=hull.getIndexBuffer();
    diameter=INFINITY;
    for(size_t i=0;i<idx.size();i+=3) {
      const auto &a=o.vertices[idx[i]],&b=o.vertices[idx[i+1]],&c=o.vertices[idx[i+2]];
      const Eigen::Vector3d n=(b-a).cross(c-a).normalized();diameter=std::min(diameter,2*std::abs(n.dot(center-a)));
    }
  }
  for(const auto &g:gates) {
    if(!g.center.allFinite() || !g.rotation.coeffs().allFinite() || std::abs(g.rotation.norm()-1)>1e-6 ||
      !std::isfinite(g.width) || !std::isfinite(g.height) || !std::isfinite(g.thickness) ||
      !std::isfinite(g.frame_width) || g.width<=2*o.margin || g.height<=2*o.margin ||
      g.thickness<=0 || g.frame_width<=0) {throw std::invalid_argument("Invalid opening/rigid-body pose: "+g.id);}
    if(std::min(g.width,g.height)<=diameter+2*o.margin) {
      throw std::invalid_argument("Opening "+g.id+" smaller than body inscribed diameter + 2*margin: minimum="+
        std::to_string(diameter+2*o.margin)+" width="+std::to_string(g.width)+" height="+std::to_string(g.height));
    }
  }
}
FrameGeometry::FrameGeometry(const std::vector<Gate> &gates,const Options &o):vertices_(o.vertices),axes_(o.body) {
  radius=axes_.maxCoeff();
  if(!vertices_.empty()) {
    radius=0;for(const auto &v:vertices_) {radius=std::max(radius,v.norm());}
    validateVertices(vertices_);Eigen::Matrix3Xd points(3,vertices_.size());
    for(size_t i=0;i<vertices_.size();++i) {points.col(i)=vertices_[i];}
    quickhull::QuickHull<double> qh;const auto hull=qh.getConvexHull(points.data(),vertices_.size(),false,true);
    const auto &idx=hull.getIndexBuffer();
    std::map<std::pair<size_t,size_t>,BodyVertices> adjacency;
    for(size_t i=0;i<idx.size();i+=3) {
      const auto a=vertices_[idx[i]],b=vertices_[idx[i+1]],c=vertices_[idx[i+2]];
      const Eigen::Vector3d normal=(b-a).cross(c-a).normalized();addAxis(normals_,normal);
      for(int k=0;k<3;++k) {size_t u=idx[i+k],v=idx[i+(k+1)%3];if(u>v) {std::swap(u,v);}adjacency[{u,v}].push_back(normal);}
    }
    for(const auto &entry:adjacency) {
      const auto &n=entry.second;
      if(n.size()!=2 || std::abs(n[0].dot(n[1]))<1-1e-8) {addAxis(edges_,vertices_[entry.first.first]-vertices_[entry.first.second]);}
    }
  } else { // These support axes remain conservative for an ellipsoid.
    for(int k=0;k<3;++k) {normals_.push_back(Eigen::Vector3d::Unit(k));edges_.push_back(Eigen::Vector3d::Unit(k));}
  }
  for(const auto &g:gates) {
    const auto R=g.rotation.toRotationMatrix();
    for(int sign:{-1,1}) {
      boxes.push_back({g.center+R*Eigen::Vector3d(0,sign*(g.width+g.frame_width)/2,0),
        {g.thickness/2,g.frame_width/2,g.height/2+g.frame_width},R,g.id});
      boxes.push_back({g.center+R*Eigen::Vector3d(0,0,sign*(g.height+g.frame_width)/2),
        {g.thickness/2,g.width/2,g.frame_width/2},R,g.id});
    }
  }
}
Separation FrameGeometry::separation(const FrameBox &box,const Eigen::Vector3d &p,
  const Eigen::Vector4d &q,bool gradient) const {
  const Eigen::Matrix3d R=Eigen::Quaterniond(q[0],q[1],q[2],q[3]).normalized().toRotationMatrix();
  Separation best;Eigen::Vector3d best_vertex,best_axis,best_raw,best_source;
  int best_type=0,best_cross=0;double best_sign=1;
  const auto test=[&](const Eigen::Vector3d &raw,int type,const Eigen::Vector3d &source,int cross) {
    if(raw.squaredNorm()<1e-14) {return;}
    const Eigen::Vector3d unit=raw.normalized();
    for(int sign:{-1,1}) {
      const Eigen::Vector3d n=sign*unit,local=R.transpose()*n;
      Eigen::Vector3d vertex;
      if(vertices_.empty()) {vertex=-axes_.cwiseAbs2().cwiseProduct(local)/axes_.cwiseProduct(local).norm();}
      else {
        double support=std::numeric_limits<double>::infinity();
        for(const auto &v:vertices_) {const double dot=local.dot(v);if(dot<support) {support=dot;vertex=v;}}
      }
      const double distance=n.dot(p-box.center)+local.dot(vertex)-box.half.dot((box.rotation.transpose()*n).cwiseAbs());
      if(distance>best.distance) {
        best.distance=distance;best_axis=n;best_vertex=vertex;best_raw=raw;best_type=type;
        best_source=source;best_cross=cross;best_sign=sign;
      }
    }
  };
  for(int k=0;k<3;++k) {test(box.rotation.col(k),0,Eigen::Vector3d::Zero(),0);}
  for(const auto &n:normals_) {test(R*n,1,n,0);}
  for(const auto &e:edges_) {for(int k=0;k<3;++k) {test((R*e).cross(box.rotation.col(k)),2,e,k);}}
  // Additional valid support axis helps curved bodies near box corners.
  if(vertices_.empty()) {test(p-box.center,3,Eigen::Vector3d::Zero(),0);}
  if(!gradient) {return best;}
  const Eigen::Vector3d n=best_axis;
  best.position_gradient=n;
  best.quaternion_gradient=rotatedVertexGradient(q,best_vertex,n);
  if(best_type) {
    Eigen::Vector3d corner=box.rotation.transpose()*n;
    for(int k=0;k<3;++k) {corner[k]=corner[k]>=0?box.half[k]:-box.half[k];}
    const Eigen::Vector3d w=p+R*best_vertex-box.center-box.rotation*corner;
    const Eigen::Vector3d projected=best_sign*(w-n*n.dot(w))/best_raw.norm();
    if(best_type==1) {best.quaternion_gradient+=rotatedVertexGradient(q,best_source,projected);}
    else if(best_type==2) {best.quaternion_gradient+=rotatedVertexGradient(q,best_source,box.rotation.col(best_cross).cross(projected));}
    else {best.position_gradient+=projected;}
  }
  return best;
}
}
