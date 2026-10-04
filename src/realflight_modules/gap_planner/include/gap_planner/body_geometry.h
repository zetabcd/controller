#pragma once
#include <Eigen/Geometry>
#include <vector>
#include <limits>
#include <algorithm>
#include <utility>

namespace gap_planner {
using BodyVertices=std::vector<Eigen::Vector3d>;
// d(n . R(normalize(q)) v)/dq, quaternion order w,x,y,z.
// Analytic local geometric derivative; independent of the flatness model.
inline Eigen::Vector4d rotatedVertexGradient(const Eigen::Vector4d &q,
  const Eigen::Vector3d &v,const Eigen::Vector3d &n)
{
  const double norm=q.norm();const Eigen::Vector4d unit=q/norm;
  const double w=unit[0];const Eigen::Vector3d u=unit.tail<3>();
  Eigen::Vector4d g;g[0]=2*w*n.dot(v)+2*n.dot(u.cross(v));
  g.tail<3>()=-2*u*n.dot(v)+2*n*u.dot(v)+2*v*n.dot(u)+2*w*v.cross(n);
  return (g-unit*unit.dot(g))/norm;
}
inline double bodySupport(const Eigen::Vector3d &direction,const Eigen::Vector3d &axes,
  const BodyVertices &vertices)
{
  if(vertices.empty()) {return axes.cwiseProduct(direction).norm();}
  double support=-std::numeric_limits<double>::infinity();
  for(const auto &v:vertices) {support=std::max(support,direction.dot(v));}return support;
}
BodyVertices boxVertices(double radius,double height);
// Triangulated convex hull edges, including coplanar face diagonals, for RViz.
std::vector<std::pair<std::size_t,std::size_t>> hullEdges(const BodyVertices &vertices);
void validateVertices(const BodyVertices &vertices);
}
