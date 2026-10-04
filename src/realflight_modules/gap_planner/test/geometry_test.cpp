#include <gtest/gtest.h>
#include <gap_planner/core.h>
#include <gap_planner/project_flatness.h>
#include <gap_planner/frame_geometry.h>
#include <gap_planner/sparse_optimizer.h>
using namespace gap_planner;
TEST(GapFiniteFrame, TiltAndFiniteExtent) {
  Options o;o.body={0.18635,0.18635,0.05};o.vertices=boxVertices(o.body.x(),0.1);
  Gate gate;gate.center={0,0,1};gate.width=0.5;gate.height=0.15;
  gate.rotation=Eigen::AngleAxisd(M_PI/6,Eigen::Vector3d::UnitX());
  FrameGeometry frames({gate},o);ASSERT_EQ(frames.boxes.size(),4u);
  const Eigen::Vector4d aligned(gate.rotation.w(),gate.rotation.x(),gate.rotation.y(),gate.rotation.z());
  double tilted=INFINITY,level=INFINITY;
  for(const auto &box:frames.boxes) {
    tilted=std::min(tilted,frames.separation(box,gate.center,aligned,false).distance);
    level=std::min(level,frames.separation(box,gate.center,{1,0,0,0},false).distance);
    EXPECT_GT(frames.separation(box,{0,0,2.5},{1,0,0,0},false).distance,o.margin);
  }
  EXPECT_NEAR(tilted,0.025,1e-12);EXPECT_LT(level,0);
}
TEST(GapFiniteFrame, NoArtificialWallBetweenVerticallySeparatedGates) {
  Options o;o.body={0.18635,0.18635,0.05};o.vertices=boxVertices(o.body.x(),0.1);
  Gate first,second;first.center={0,0,1};second.center={0.1,0,2.5};
  EXPECT_NO_THROW(validateTask({first,second},{-1.65,0,1.05},{2.35,0,1.05},o));
  FrameGeometry frames({first,second},o);
  for(const auto &box:frames.boxes) {EXPECT_GT(frames.separation(box,{0.05,0,1.75},{1,0,0,0},false).distance,o.margin);}
  // Neither large endpoints nor a reversed X leg creates a room constraint.
  EXPECT_NO_THROW(validateTask({first},{4,0,1},{-4,0,1},o));
}
TEST(GapGeometry, AnalyticQuaternionGradient)
{
  const Eigen::Vector4d q(0.9,0.2,-0.3,0.15);
  const Eigen::Vector3d point(0.18635,-0.18635,0.05),normal(0.2,0.8,-0.5);
  const auto grad=rotatedVertexGradient(q,point,normal);
  auto value=[&](const Eigen::Vector4d &v) {
    return normal.dot(Eigen::Quaterniond(v[0],v[1],v[2],v[3]).normalized()*point);
  };
  for(int k=0;k<4;++k) {
    auto plus=q,minus=q;plus[k]+=1e-6;minus[k]-=1e-6;
    EXPECT_NEAR(grad[k],(value(plus)-value(minus))/2e-6,1e-8);
  }
}
TEST(GapGeometry, RejectDegenerateVerticesAndReportFailedTiming)
{
  EXPECT_THROW(validateVertices({{0,0,0},{1,0,0},{0,1,0},{1,1,0}}),std::invalid_argument);
  Options o;Gate g;g.height=0.15;
  try {plan({g},1,{-1,0,1},{1,0,1},o);FAIL()<<"Must reject invalid opening before optimization";}
  catch(const PlanError &e) {
    EXPECT_EQ(e.timing.phase,"setup");EXPECT_EQ(e.timing.optimize_ms,0);
    EXPECT_GT(e.timing.total_ms,0);EXPECT_TRUE(e.timing.stage_ms.empty());
  }
}
TEST(GapFlatness, MatchesControllerAndFiniteDifferenceGradient)
{
  ProjectFlatness map;map.model.drag={0.32,0.35,0.52};map.model.lift=0.013;map.model.heading=0.4;
  const Eigen::Vector3d v(1,-0.5,0.2),a(1.2,4,0.6),j(-0.8,0.9,0.5);
  double thrust;Eigen::Vector4d q;Eigen::Vector3d w;
  map.forward(v,a,j,0,0,thrust,q,w);
  px4ctrl::ReferencePoint flat;flat.velocity=v;flat.acceleration=a;flat.jerk=j;
  const auto reference=px4ctrl::externalReference(flat,map.model);
  EXPECT_NEAR(thrust,reference.thrust_acceleration*map.model.mass,1e-10);
  EXPECT_LT((w-reference.body_rate).norm(),1e-10);
  EXPECT_NEAR(std::abs(Eigen::Quaterniond(q[0],q[1],q[2],q[3]).dot(reference.attitude)),1,1e-10);
  const Eigen::Vector4d gq(0.3,-0.6,0.4,0.1);const Eigen::Vector3d gw(-0.2,0.5,0.7);
  Eigen::Vector3d op,ov,oa,oj;double yaw,yr;
  map.backward(Eigen::Vector3d::Zero(),Eigen::Vector3d::Zero(),0.6,gq,gw,op,ov,oa,oj,yaw,yr);
  auto value=[&](Eigen::Vector3d v,Eigen::Vector3d a,Eigen::Vector3d j) {
    px4ctrl::ReferencePoint r;r.velocity=v;r.acceleration=a;r.jerk=j;
    auto actual=px4ctrl::externalReference(r,map.model);
    Eigen::Vector4d quaternion(actual.attitude.w(),actual.attitude.x(),actual.attitude.y(),actual.attitude.z());
    if(q.dot(quaternion)<0) {quaternion=-quaternion;}
    return 0.6*map.model.mass*actual.thrust_acceleration+gq.dot(quaternion)+gw.dot(actual.body_rate);
  };
  for(int k=0;k<9;++k) {
    auto vp=v,vm=v,ap=a,am=a,jp=j,jm=j;
    (k<3?vp[k]:(k<6?ap[k-3]:jp[k-6]))+=1e-5;
    (k<3?vm[k]:(k<6?am[k-3]:jm[k-6]))-=1e-5;
    EXPECT_NEAR(k<3?ov[k]:(k<6?oa[k-3]:oj[k-6]),(value(vp,ap,jp)-value(vm,am,jm))/2e-5,1e-7);
  }
}
TEST(GapFiniteFrame, SeparatingAxisGradient) {
  Options o;o.body={0.18635,0.18635,0.05};o.vertices=boxVertices(o.body.x(),0.1);
  Gate gate;gate.rotation=Eigen::AngleAxisd(0.71,Eigen::Vector3d::UnitX());
  FrameGeometry frames({gate},o);
  for(int sample=0;sample<40;++sample) {
    const Eigen::Vector3d p(0.13+0.014*sample,0.21-0.007*sample,0.23+0.009*sample);
    const Eigen::Vector4d q(0.93,0.12+sample*0.007,-0.08,0.06);
    for(const auto &box:frames.boxes) {
      const auto s=frames.separation(box,p,q);
      for(int k=0;k<7;++k) {
        auto pp=p,pm=p;auto qp=q,qm=q;
        if(k<3) {pp[k]+=1e-6;pm[k]-=1e-6;} else {qp[k-3]+=1e-6;qm[k-3]-=1e-6;}
        const double numerical=(frames.separation(box,pp,qp,false).distance-frames.separation(box,pm,qm,false).distance)/2e-6;
        EXPECT_NEAR(k<3?s.position_gradient[k]:s.quaternion_gradient[k-3],numerical,1e-7);
      }
    }
  }
}
TEST(GapMINCO, FullSparseObjectiveGradient) {
  Gate g;g.center={0,0,1};g.width=0.5;g.height=0.25;
  g.rotation=Eigen::AngleAxisd(0.8,Eigen::Vector3d::UnitX());
  Options o;o.body={0.18635,0.18635,0.05};o.vertices=boxVertices(o.body.x(),0.1);
  o.model.drag={0.32,0.35,0.52};o.model.lift=0.013;
  o.execution_limits.motor_max=2.1;o.execution_limits.angular_acceleration_max=2;
  SparseOptimizer optimizer({g},{-1.65,0,1.05},{2.35,0,1.05},o);
  auto x=optimizer.initial;
  for(int k=0;k<x.size();++k) {x[k]+=0.013*std::sin(k*1.31);}
  Eigen::VectorXd grad;optimizer.evaluate(x,grad);
  std::vector<std::vector<double>> extra(6);extra[2]={0.82};extra[3]={0.13};optimizer.refine(extra);
  optimizer.evaluate(x,grad);
  for(int k=0;k<x.size();++k) {
    auto xp=x,xm=x;xp[k]+=1e-6;xm[k]-=1e-6;Eigen::VectorXd ignored;
    const double numerical=(optimizer.evaluate(xp,ignored)-optimizer.evaluate(xm,ignored))/2e-6;
    EXPECT_NEAR(grad[k],numerical,2e-4*std::max(1.0,std::abs(numerical)))<<"variable "<<k;
  }
}

TEST(GapFlatness, AngularAccelerationAndSnapGradientMatchController) {
  ProjectFlatness map;map.model.drag={0.32,0.35,0.52};map.model.lift=0.013;map.model.heading=0.3;
  const Eigen::Vector3d v(1,-0.5,0.2),a(1.2,4,0.6),j(-0.8,0.9,0.5),s(2,-3,1);
  double thrust;Eigen::Vector4d q;Eigen::Vector3d w,alpha;
  map.forwardFull(v,a,j,s,thrust,q,w,alpha);
  px4ctrl::ReferencePoint flat;flat.velocity=v;flat.acceleration=a;flat.jerk=j;flat.snap=s;
  const auto r=px4ctrl::externalReference(flat,map.model);
  EXPECT_LT((alpha-r.body_acceleration).norm(),1e-10);EXPECT_LT((w-r.body_rate).norm(),1e-10);
  const Eigen::Vector4d gq(0.3,-0.6,0.4,0.1);const Eigen::Vector3d gw(-0.2,0.5,0.7),galpha(0.4,-0.3,0.8);
  Eigen::Vector3d op,ov,oa,oj,os;
  map.backwardFull(Eigen::Vector3d::Zero(),Eigen::Vector3d::Zero(),0.6,gq,gw,galpha,op,ov,oa,oj,os);
  auto value=[&](px4ctrl::ReferencePoint point) {
    const auto actual=px4ctrl::externalReference(point,map.model);
    Eigen::Vector4d quaternion(actual.attitude.w(),actual.attitude.x(),actual.attitude.y(),actual.attitude.z());
    if(q.dot(quaternion)<0) {quaternion=-quaternion;}
    return 0.6*map.model.mass*actual.thrust_acceleration+gq.dot(quaternion)+gw.dot(actual.body_rate)+galpha.dot(actual.body_acceleration);
  };
  const Eigen::Vector3d gradients[4]={ov,oa,oj,os};
  for(int k=0;k<12;++k) {
    auto plus=flat,minus=flat;
    Eigen::Vector3d *pv[]={&plus.velocity,&plus.acceleration,&plus.jerk,&plus.snap};
    Eigen::Vector3d *mv[]={&minus.velocity,&minus.acceleration,&minus.jerk,&minus.snap};
    (*pv[k/3])[k%3]+=1e-5;(*mv[k/3])[k%3]-=1e-5;
    EXPECT_NEAR(gradients[k/3][k%3],(value(plus)-value(minus))/2e-5,1e-7);
  }
}
