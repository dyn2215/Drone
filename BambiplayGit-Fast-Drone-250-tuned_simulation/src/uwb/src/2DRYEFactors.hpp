// This hpp file provide range, yaw, elevation factors in GTSAM for gtsam::Pose2
#include <gtsam/geometry/Pose2.h>
#include <gtsam/inference/Key.h>
#include <gtsam/nonlinear/NonlinearFactor.h> 

class RangeFactor: public gtsam::NoiseModelFactor1<gtsam::Pose2> {
  double r; // range measurement
  double px,py,deltaz; // position of landmark
public:
  RangeFactor(gtsam::Key j, double r, double px, double py, double deltaz, const gtsam::SharedNoiseModel& model):
    gtsam::NoiseModelFactor1<gtsam::Pose2>(model, j), r(r), px(px), py(py), deltaz(deltaz) {}

  gtsam::Vector evaluateError(const gtsam::Pose2& q, boost::optional<gtsam::Matrix&> H = boost::none) const{
    double r_est = sqrt(pow(q.x()-px,2)+pow(q.y()-py,2)+pow(deltaz,2));
    double theta = atan2(q.y()-py,q.x()-px)-q.theta();
    double elev = atan2(-deltaz,sqrt(pow(q.x()-px,2)+pow(q.y()-py,2)));
    if (H) (*H) = (gtsam::Matrix(1, 3) <<
            cos(theta)*cos(elev), sin(theta)*cos(elev), 0.0).finished();
    return (gtsam::Vector(1) << r_est - r).finished();
  }
}; 

class YawFactor: public gtsam::NoiseModelFactor1<gtsam::Pose2> {
  double b; // yaw measurement
  double px,py,deltaz; // position of landmark
public:
  YawFactor(gtsam::Key j, double b, double px, double py, double deltaz, const gtsam::SharedNoiseModel& model):
    gtsam::NoiseModelFactor1<gtsam::Pose2>(model, j), b(b), px(px), py(py), deltaz(deltaz) {}

  gtsam::Vector evaluateError(const gtsam::Pose2& q, boost::optional<gtsam::Matrix&> H = boost::none) const{
    double theta = atan2(py-q.y(),px-q.x())-q.theta();
    if (H) (*H) = (gtsam::Matrix(1, 3) <<
            0.0, 0.0, -1.0).finished();
    double delta_theta = theta - b;
    if (delta_theta > M_PI) delta_theta -= 2*M_PI;
    if (delta_theta < -M_PI) delta_theta += 2*M_PI;
    return (gtsam::Vector(1) << delta_theta).finished();    
  }
};

class ElevFactor: public gtsam::NoiseModelFactor1<gtsam::Pose2> {
  double a; // elevation measurement
  double px,py,deltaz; // position of landmark
public:
  ElevFactor(gtsam::Key j, double a, double px, double py, double deltaz, const gtsam::SharedNoiseModel& model):
    gtsam::NoiseModelFactor1<gtsam::Pose2>(model, j), a(a), px(px), py(py), deltaz(deltaz) {}

  gtsam::Vector evaluateError(const gtsam::Pose2& q, boost::optional<gtsam::Matrix&> H = boost::none) const{
    double elev = atan2(deltaz,sqrt(pow(q.x()-px,2)+pow(q.y()-py,2)));
    double theta = atan2(py-q.y(),px-q.x())-q.theta();
    double range = sqrt(pow(q.x()-px,2)+pow(q.y()-py,2)+pow(deltaz,2));
    if (H) (*H) = (gtsam::Matrix(1, 3) <<
            cos(theta)*sin(elev)/range, sin(theta)*sin(elev)/range, 0.0).finished();
    double delta_elev = elev - a;
    if (delta_elev > M_PI) delta_elev -= 2*M_PI;
    if (delta_elev < -M_PI) delta_elev += 2*M_PI;
    return (gtsam::Vector(1) << delta_elev).finished();    
  }
};