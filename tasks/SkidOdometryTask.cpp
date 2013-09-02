/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "SkidOdometryTask.hpp"

using namespace odometry;

SkidOdometryTask::SkidOdometryTask(std::string const& name)
    : SkidOdometryTaskBase(name)
{
}

SkidOdometryTask::SkidOdometryTask(std::string const& name, RTT::ExecutionEngine* engine)
    : SkidOdometryTaskBase(name, engine)
{
}

SkidOdometryTask::~SkidOdometryTask()
{
}

void SkidOdometryTask::actuator_samplesTransformerCallback(const base::Time &ts, const ::base::samples::Joints &actuator_samples_sample)
{
    // calculate average speed of all wheels as velocity over ground
    moving_speed = 0;
    for(size_t i=0; i<actuator_samples_sample.size(); ++i)
	moving_speed += actuator_samples_sample[i].speed;
    moving_speed = moving_speed / actuator_samples_sample.size() * _wheelRadiusAvg.value();
}

void SkidOdometryTask::body2imu_enuTransformerCallback(const base::Time& ts)
{
    // use the transformer to get the body2world transformation 
    // this should include the imu reading
    base::Transform3d body2IMUWorld;
    if( !_body2imu_enu.get( ts, body2IMUWorld ) )
	return;

    // calculates the rotation from body to world base on the orientation measurment 
    Eigen::Quaterniond R_body2World(body2IMUWorld.rotation()); 

    // calculate the travelled distance based on the speed over ground
    // and the time since the last update
    if( prev_ts == base::Time() )
	prev_ts = ts;

    double dt = (ts - prev_ts).toSeconds();
    double moving_dist = moving_speed * dt;

    // update the odometry
    odometry->update( moving_dist, R_body2World );

    // create a transform with uncertainty based on the odometry 
    envire::TransformWithUncertainty body2PrevBody( 
	    odometry->getPoseDelta().toTransform(),
	    odometry->getPoseError() );

    // push the transformations
    pushState( ts, body2PrevBody, R_body2World );
}

/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See SkidOdometryTask.hpp for more detailed
// documentation about them.

bool SkidOdometryTask::configureHook()
{
    if (! SkidOdometryTaskBase::configureHook())
        return false;

    odometry = boost::shared_ptr<odometry::SkidOdometry>(new odometry::SkidOdometry(
	    _odometry_config.get(),
	    _wheelRadiusAvg.get(),
	    _trackWidth.get(),
	    _wheelBase.get()));

    _body2imu_enu.registerUpdateCallback(boost::bind(&SkidOdometryTask::body2imu_enuTransformerCallback, this, _1));

    return true;
}
bool SkidOdometryTask::startHook()
{
    if (! SkidOdometryTaskBase::startHook())
        return false;
    
    moving_speed = 0;
    prev_ts = base::Time();

    return true;
}
void SkidOdometryTask::updateHook()
{
    SkidOdometryTaskBase::updateHook();
}
void SkidOdometryTask::errorHook()
{
    SkidOdometryTaskBase::errorHook();
}
void SkidOdometryTask::stopHook()
{
    SkidOdometryTaskBase::stopHook();
}
void SkidOdometryTask::cleanupHook()
{
    SkidOdometryTaskBase::cleanupHook();
}
