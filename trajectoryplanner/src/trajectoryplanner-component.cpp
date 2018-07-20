#include "trajectoryplanner-component.hpp"
#include <rtt/Component.hpp>
#include <iostream>

Trajectoryplanner::Trajectoryplanner(std::string const& name) : TaskContext(name){
  std::cout << "Trajectoryplanner constructed !" <<std::endl;
}

bool Trajectoryplanner::configureHook(){
  std::cout << "Trajectoryplanner configured !" <<std::endl;
  return true;
}

bool Trajectoryplanner::startHook(){
  std::cout << "Trajectoryplanner started !" <<std::endl;
  return true;
}

void Trajectoryplanner::updateHook(){
  std::cout << "Trajectoryplanner executes updateHook !" <<std::endl;
}

void Trajectoryplanner::stopHook() {
  std::cout << "Trajectoryplanner executes stopping !" <<std::endl;
}

void Trajectoryplanner::cleanupHook() {
  std::cout << "Trajectoryplanner cleaning up !" <<std::endl;
}

bool Trajectoryplanner::moveLine(Frame start, Frame end)
{
    try{
        //step1:create line path
        path_ = new Path_Line(start ,end ,new RotationalInterpolation_SingleAxis(), eqradio_, true);
        //step2:create velocity profile
        velocityprofile_ = new VelocityProfile_Trap(velocityprofile_maxvel_, velocityprofile_maxacc_);
        velocityprofile_->SetProfile(0, path_->PathLength());
        trajectory_ = new Trajectory_Segment(path_, velocityprofile_);
        return true;
    }catch(KDL::Error& error){
        std::cout <<"I encountered this error : " << error.Description() << std::endl;
        std::cout << "with the following type " << error.GetType() << std::endl;
        return false;
    }
}

bool Trajectoryplanner::moveCircle(Frame &start, Vector& center, Vector& endp, Rotation& endr){
    try{
        //step1: create circle path
        path_ = new Path_Circle(start,center,endp,endr,circle_alpha_,new RotationalInterpolation_SingleAxis(),eqradio_,true);
        //step2: create velocity profile
        velocityprofile_ = new VelocityProfile_Trap(velocityprofile_maxvel_, velocityprofile_maxacc_);
        velocityprofile_->SetProfile(0, path_->PathLength());
        trajectory_ = new Trajectory_Segment(path_, velocityprofile_);
        return true;
    }catch(KDL::Error& error){
        std::cout <<"I encountered this error : " << error.Description() << std::endl;
        std::cout << "with the following type " << error.GetType() << std::endl;
        return false;
    }
}

/*
 * Using this macro, only one component may live
 * in one library *and* you may *not* link this library
 * with another component library. Use
 * ORO_CREATE_COMPONENT_TYPE()
 * ORO_LIST_COMPONENT_TYPE(Trajectoryplanner)
 * In case you want to link with another library that
 * already contains components.
 *
 * If you have put your component class
 * in a namespace, don't forget to add it here too:
 */
ORO_CREATE_COMPONENT(Trajectoryplanner)
