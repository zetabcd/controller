#pragma once
#include <gap_planner/core.h>
#include <gap_msgs/msg/flight_model.hpp>
namespace gap_planner
{
template<class V> Eigen::Vector3d vector(const V &v) {return {v.x,v.y,v.z};}
template<class V> void assign(V &v,const Eigen::Vector3d &p) {v.x=p.x();v.y=p.y();v.z=p.z();}
inline gap_msgs::msg::FlightModel message(const px4ctrl::ExternalModel &m)
{
  gap_msgs::msg::FlightModel r;r.version=1;r.gravity=m.gravity;r.mass=m.mass;
  r.heading=m.heading;assign(r.drag_acceleration,m.drag);r.horizontal_lift_acceleration=m.lift;return r;
}
inline px4ctrl::ExternalModel model(const gap_msgs::msg::FlightModel &m)
{px4ctrl::ExternalModel r;r.gravity=m.gravity;r.mass=m.mass;r.heading=m.heading;
  r.drag=vector(m.drag_acceleration);r.lift=m.horizontal_lift_acceleration;return r;}
}
