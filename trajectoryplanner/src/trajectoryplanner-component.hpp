#ifndef OROCOS_TRAJECTORYPLANNER_COMPONENT_HPP
#define OROCOS_TRAJECTORYPLANNER_COMPONENT_HPP

#include <rtt/RTT.hpp>
#include <rtt/Property.hpp>
#include <rtt/Attribute.hpp>
#include <rtt/Operation.hpp>
#include <rtt/Port.hpp>

//trajectory realted 
#include <frames.hpp>
#include <frames_io.hpp>
#include <trajectory.hpp>
#include <trajectory_segment.hpp>
#include <trajectory_stationary.hpp>
#include <trajectory_composite.hpp>
#include <trajectory_composite.hpp>
#include <velocityprofile_trap.hpp>
#include <path_roundedcomposite.hpp>
#include <path_line.hpp>
#include <path_circle.hpp>
#include <rotational_interpolation_sa.hpp>
#include <trajectory_composite.hpp>
#include <utilities/error.h>

using namespace KDL;
using namespace RTT;
//component setting
class Trajectoryplanner : public RTT::TaskContext{
  public:
    Trajectoryplanner(std::string const& name);
    bool configureHook();
    bool startHook();
    void updateHook();
    void stopHook();
    void cleanupHook();
    bool moveLine(Frame start, Frame end);
    bool moveCircle(Frame &start, Vector& center, Vector& endp, Rotation& endr);
    bool moveCircle_close();
    bool movePTP();
  public:
  // property and attribute for velocity profile
    double velocityprofile_maxacc_;
    double velocityprofile_maxvel_;
  public:
  // property for path
    double eqradio_;
    double scale_;
    double circle_alpha_;
  public:
  //trajectory key variables
    Trajectory * trajectory_;
    VelocityProfile * velocityprofile_;
    Path * path_;
};
#endif
