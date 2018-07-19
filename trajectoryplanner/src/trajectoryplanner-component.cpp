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

//define the operation function
bool Trajectoryplanner::moveLine(Frame start, Frame end){

	try{
		//step1:create path
	    _path = new Path_Line(start ,end ,new RotationalInterpolation_SingleAxis(), _eqradio, true);
	    //step2:create velocity profile
	    _velocityprofile = new VelocityProfile_Trap(_velocityprofile_maxvel, _velocityprofile_maxacc);
	    _velocityprofile->SetProfile(0, _path->PathLength());
	    _trajectory = new Trajectory_Segment(_path, _velocityprofile);
	    return true;
	}
	catch(Error& error){
	    Error error;
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
