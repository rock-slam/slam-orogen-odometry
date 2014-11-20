/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "pureSkid.hpp"

using namespace odometry;

pureSkid::pureSkid(std::string const& name)
    : pureSkidBase(name)
{
}

pureSkid::pureSkid(std::string const& name, RTT::ExecutionEngine* engine)
    : pureSkidBase(name, engine)
{
}

pureSkid::~pureSkid()
{
}

void pureSkid::prepareForNewState() {
    previous_timestep = actual_timestep; // save last timestep
    
    _actuator_samples.read(actual_timestep.actuator_sample); // read new, actual sample
    actual_timestep.ts = actual_timestep.actuator_sample.time;
    
    if (!prev_sample_exists) {
        previous_timestep = actual_timestep; // copy actual timestep to previous, to have a start value
        prev_sample_exists = true;
    }
}

void pureSkid::calcBodySpeed() {
    
    // check if the wheelcount stays the same between the updates
    int tmp_act_wheel_count = actual_timestep.actuator_sample.elements.size();
    int tmp_prev_wheel_count = previous_timestep.actuator_sample.elements.size();
    if (((tmp_act_wheel_count == 0) && (tmp_prev_wheel_count == 0))
      || (tmp_act_wheel_count != tmp_prev_wheel_count))  {
        return; // elements of the joints are not valid/the same
    }

    // calculate average speed of all wheels as velocity over ground
    actual_timestep.body_speed = 0;


    actual_timestep.engine_count.right = 0;
    actual_timestep.engine_count.left = 0;
    for(std::vector<std::string>::const_iterator it = rightWheelNames.begin();
	it != rightWheelNames.end(); it++)
    {
        base::JointState const &state(actual_timestep.actuator_sample[*it]);
        if(!state.hasSpeed())
        {
            actual_timestep.body_speed = 0;
            return;
            throw std::runtime_error("Did not get needed speed value");
        }
        actual_timestep.body_speed += state.speed;
        actual_timestep.engine_speed.right += state.speed;
        actual_timestep.engine_count.right++;
    }

    for(std::vector<std::string>::const_iterator it = leftWheelNames.begin();
	it != leftWheelNames.end(); it++)
    {
        base::JointState const &state(actual_timestep.actuator_sample[*it]);
        if(!state.hasSpeed())
          {
            actual_timestep.body_speed = 0;
            return;
            throw std::runtime_error("Did not get needed speed value");
          }
        actual_timestep.body_speed += state.speed;
        actual_timestep.engine_speed.left += state.speed;
        actual_timestep.engine_count.left++;
    }

    // [m/s] = [rad/s] * [m/rad]
    actual_timestep.body_speed = (actual_timestep.body_speed / (actual_timestep.engine_count.left + actual_timestep.engine_count.right)) * _wheelRadiusAvg.value() / (2*M_PI);
    actual_timestep.engine_speed.right = (actual_timestep.engine_speed.right / actual_timestep.engine_count.right) * _wheelRadiusAvg.value();
    actual_timestep.engine_speed.left = (actual_timestep.engine_speed.left / actual_timestep.engine_count.left) * _wheelRadiusAvg.value();
};

void pureSkid::calcDeltaOri() {
    // get difference of engine speeds
    double diff = actual_timestep.engine_speed.right - actual_timestep.engine_speed.left;
    // get timestamp difference
    double dt = (actual_timestep.ts - previous_timestep.ts).toSeconds();
    // calculate alpha
    double alpha = diff * dt / _trackWidth;
    Eigen::AngleAxisd aa(alpha, Eigen::Vector3d::UnitZ()); // turning axis
    actual_timestep.delta_ori = Eigen::Quaterniond(aa);
};

void pureSkid::updateBodyState() {


    // get timestamp difference
    double dt = (actual_timestep.ts - previous_timestep.ts).toSeconds();
    //double dist = (actual_timestep.engine_speed.right - actual_timestep.engine_speed.left) * dt;
    double dist = actual_timestep.body_speed * dt;
    
    // be sure the distance is valid
    if (std::isfinite(dist)) {
        
        // calc new orientation
        actual_timestep.orientation = actual_timestep.delta_ori * previous_timestep.orientation;
        actual_timestep.orientation.normalize();
                
        // update odometry state
        odometry->update(dist, actual_timestep.orientation);
        // create a transform with uncertainty based on the odometry
        envire::TransformWithUncertainty body2PrevBody(
            odometry->getPoseDelta().toTransform(),
            odometry->getPoseError() );
        
        // push the transformations
        pushState( actual_timestep.ts, body2PrevBody, actual_timestep.orientation );

    }
};

void pureSkid::actuator_samplesTransformerCallback(const base::Time &ts, const ::base::samples::Joints &actuator_samples_sample)
{

    calcBodySpeed();
    calcDeltaOri();
    updateBodyState();

}

/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See pureSkid.hpp for more detailed
// documentation about them.

bool pureSkid::configureHook()
{
    if (! pureSkidBase::configureHook())
        return false;

    odometry = boost::shared_ptr<odometry::SkidOdometry>(new odometry::SkidOdometry(
	    _odometry_config.get(),
	    _wheelRadiusAvg.get(),
	    _trackWidth.get(),
	    _wheelBase.get()));


    odometry->setBodyCenterCompensation(false);
    
    rightWheelNames = _rightWheelNames.get();
    leftWheelNames = _leftWheelNames.get();

    return true;
}

bool pureSkid::startHook()
{
    if (! pureSkidBase::startHook())
        return false;

    // initialization
    prev_sample_exists = false;

    base::Position pos(0.0, 0.0, 0.0);
    Eigen::Quaterniond ori(1, 0, 0, 0);

    actual_timestep.ts = base::Time();
    actual_timestep.delta_ori = ori;
    actual_timestep.orientation = ori;


    return true;
}
void pureSkid::updateHook()
{
    pureSkidBase::updateHook();

    prepareForNewState();
    if (prev_sample_exists) {
        actuator_samplesTransformerCallback(actual_timestep.ts, actual_timestep.actuator_sample); // process it
    }

}
void pureSkid::errorHook()
{
    pureSkidBase::errorHook();
}
void pureSkid::stopHook()
{
    pureSkidBase::stopHook();
}
void pureSkid::cleanupHook()
{
    pureSkidBase::cleanupHook();
}
