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
  public:
  // trajectory operation
    bool moveLine(Frame start,Frame end);
    bool moveCircle();
    bool moveCircle_close();
    bool movePTP();
  public:
  // property and attribute for velocity profile
    double _velocityprofile_maxacc;
    double _velocityprofile_maxvel;
  public:
  // property for path
    double _eqradio;
    double _scale;
  public:
  //trajectory key variables
    Trajectory * _trajectory;
    VelocityProfile * _velocityprofile;
    Path * _path;
};
#endif
