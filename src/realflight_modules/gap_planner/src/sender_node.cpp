#include <gap_planner/ros_conversion.h>
#include <gap_msgs/msg/polynomial_plan.hpp>
#include <gap_msgs/srv/upload_trajectory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <chrono>

class Sender : public rclcpp::Node
{
  using Upload=gap_msgs::srv::UploadTrajectory;
  rclcpp::Client<Upload>::SharedPtr client_;
  rclcpp::Subscription<gap_msgs::msg::PolynomialPlan>::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<Upload::Request> pending_;
  std::string last_id_;
  double dt_;
public:
  Sender() : Node("gap_trajectory_sender")
  {
    dt_=declare_parameter("sample_dt",0.005);
    if(!std::isfinite(dt_) || dt_<0.001 || dt_>0.05) {throw std::invalid_argument("sample_dt must be 1..50 ms");}
    client_=create_client<Upload>("/gap/upload");
    subscription_=create_subscription<gap_msgs::msg::PolynomialPlan>("/gap/polynomial",
      rclcpp::QoS(1).reliable().transient_local(),[this](gap_msgs::msg::PolynomialPlan::ConstSharedPtr m) {
        if(m->trajectory_id==last_id_) {return;}
        last_id_=m->trajectory_id;pending_.reset();
        try {
          if(m->durations.empty() || m->durations.size()>1000 ||
            m->coefficients.size()!=18*m->durations.size()) {throw std::invalid_argument("Malformed polynomial plan");}
          auto req=std::make_shared<Upload::Request>();req->header=m->header;
          req->trajectory_id=m->trajectory_id;req->scene_id=m->scene_id;req->model=m->model;
          double start=0;
          for(std::size_t i=0;i<m->durations.size();++i) {
            const double h=m->durations[i];
            if(!std::isfinite(h) || h<1e-4 || start+h>120) {throw std::invalid_argument("Invalid piece duration");}
            gap_planner::Piece piece;piece.duration=h;
            for(int row=0;row<3;++row) {for(int col=0;col<6;++col) {piece.coefficients(row,col)=m->coefficients[i*18+row*6+col];}}
            if(!piece.coefficients.allFinite()) {throw std::invalid_argument("Nonfinite coefficients");}
            const int n=std::max(1,static_cast<int>(std::ceil(h/dt_)));
            for(int k=(i?1:0);k<=n;++k) {
              const double t=h*k/n;const auto r=gap_planner::flatPoint(piece,t);
              gap_msgs::msg::FlatSample s;s.time=start+t;
              gap_planner::assign(s.position,r.position);gap_planner::assign(s.velocity,r.velocity);
              gap_planner::assign(s.acceleration,r.acceleration);req->samples.push_back(s);
              if(req->samples.size()>50000) {throw std::invalid_argument("Too many trajectory samples");}
            }
            start+=h;
          }
          pending_=req;
        } catch(const std::exception &e) {RCLCPP_ERROR(get_logger(),"Cannot sample plan: %s",e.what());}
      });
    timer_=create_wall_timer(std::chrono::milliseconds(100),[this]() {
      if(!pending_ || !client_->service_is_ready()) {return;}
      const auto req=std::move(pending_);
      const auto began=std::chrono::steady_clock::now();
      RCLCPP_INFO(get_logger(),"Uploading %zu samples, trajectory=%s",req->samples.size(),req->trajectory_id.c_str());
      client_->async_send_request(req,[this,began](rclcpp::Client<Upload>::SharedFuture f) {
        try {const auto r=f.get();RCLCPP_INFO(get_logger(),"Upload accepted=%d upload_ack_ms=%.2f: %s",r->accepted,
          std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-began).count(),r->message.c_str());}
        catch(const std::exception &e) {RCLCPP_ERROR(get_logger(),"Upload failed: %s",e.what());}
      });
    });
  }
};
int main(int argc,char **argv)
{rclcpp::init(argc,argv);rclcpp::spin(std::make_shared<Sender>());rclcpp::shutdown();}
