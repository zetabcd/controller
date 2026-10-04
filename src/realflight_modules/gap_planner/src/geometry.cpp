#include <gap_planner/core.h>
#include <stdexcept>
#include <cmath>
#include <quickhull.hpp>
#include <Eigen/LU>
#include <set>
#include <sstream>
#include <iomanip>

namespace gap_planner
{
BodyVertices boxVertices(double radius,double height)
{
  if(!std::isfinite(radius) || !std::isfinite(height) || radius<=0 || height<=0) {
    throw std::invalid_argument("Body radius and height must be finite and positive");
  }
  BodyVertices points;
  for(int x:{-1,1}) {for(int y:{-1,1}) {for(int z:{-1,1}) {
    points.emplace_back(x*radius,y*radius,z*height/2);
  }}}return points;
}
void validateVertices(const BodyVertices &vertices)
{
  if(vertices.size()<4 || vertices.size()>64) {throw std::invalid_argument("vehicle.vertices needs 4..64 3D points");}
  Eigen::Matrix3Xd differences(3,vertices.size());
  for(std::size_t i=0;i<vertices.size();++i) {
    if(!vertices[i].allFinite()) {throw std::invalid_argument("vehicle.vertices must be finite");}
    differences.col(i)=vertices[i]-vertices.front();
  }
  if(differences.fullPivLu().rank()!=3) {throw std::invalid_argument("vehicle.vertices must enclose a 3D volume, not a plane/line");}
}
std::vector<std::pair<std::size_t,std::size_t>> hullEdges(const BodyVertices &vertices)
{
  validateVertices(vertices);Eigen::Matrix3Xd points(3,vertices.size());
  for(std::size_t i=0;i<vertices.size();++i) {points.col(i)=vertices[i];}
  quickhull::QuickHull<double> qh;
  const auto hull=qh.getConvexHull(points.data(),vertices.size(),false,true);
  const auto &indices=hull.getIndexBuffer();std::set<std::pair<std::size_t,std::size_t>> edges;
  for(std::size_t i=0;i<indices.size();i+=3) {for(int j=0;j<3;++j) {
    const auto a=indices[i+j],b=indices[i+(j+1)%3];edges.emplace(std::min(a,b),std::max(a,b));
  }}return {edges.begin(),edges.end()};
}

px4ctrl::ReferencePoint flatPoint(const Piece &piece,double t)
{
  px4ctrl::ReferencePoint r;Eigen::Vector3d d[5];
  for(int k=0;k<5;++k) {
    d[k].setZero();
    for(int j=k;j<6;++j) {
      double f=1;for(int n=0;n<k;++n) {f*=j-n;}
      d[k]+=piece.coefficients.col(j)*f*std::pow(t,j-k);
    }
  }
  r.position=d[0];r.velocity=d[1];r.acceleration=d[2];r.jerk=d[3];r.snap=d[4];return r;
}
}
