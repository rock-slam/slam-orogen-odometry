/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Skid4OdometryTask.hpp"

using namespace odometry;

Skid4OdometryTask::Skid4OdometryTask(std::string const& name)
    : Skid4OdometryTaskBase(name)
{
}

Skid4OdometryTask::Skid4OdometryTask(std::string const& name, RTT::ExecutionEngine* engine)
    : Skid4OdometryTaskBase(name, engine)
{
}

Skid4OdometryTask::~Skid4OdometryTask()
{
}

void Skid4OdometryTask::actuator_samplesTransformerCallback(const base::Time &ts, const ::base::actuators::Status &actuator_samples_sample)
{
    // just copy the readings, we don't need to process them at this high update rate.
    body_state.time = ts;
    for(int i=0; i<4;i++)
	body_state.setWheelPos(
                static_cast<odometry::wheelIdx>(i),
                actuator_samples_sample.states[i].positionExtern);

    if(!od->state.isValid()) 
	od->state.update(body_state);
}

void Skid4OdometryTask::body2imu_enuTransformerCallback(const base::Time& ts)
{
    //wait until we got an actuator status
    if(!od->state.isValid())
    {    
	return;
    }
    
    // use the transformer to get the body2world transformation 
    // this should include the imu reading
    base::Transform3d body2IMUWorld;
    if( !_body2imu_enu.get( ts, body2IMUWorld ) )
    {
	return;
    }

    // calculates the rotation from body to world base on the orientation measurment 
    Eigen::Quaterniond R_body2World(body2IMUWorld.rotation()); 

    // update the odometry state 
    od->update(body_state, R_body2World);

    // create a transform with uncertainty based on the odometry 
    envire::TransformWithUncertainty body2PrevBody( 
	    od->getPoseDelta().toTransform(),
	    od->getPoseError() );

    // Generate the output for the pose delta
    base::samples::RigidBodyState state;
    state.invalidate();
    state.time = ts;
    body2PrevBody.copyToRigidBodyState(state);
    state.angular_velocity = od->getAngularVelocity();
    state.velocity = od->getVelocity();

    // write to port if connected
    if(_odometry_delta_samples.connected())
	_odometry_delta_samples.write(state);
    
    // also sum up the relative changes in an absolute odometry frame
    // and output to a separate port. 
    lastBody2Odometry = lastBody2Odometry * body2PrevBody;
    // need to set the absolute body to world orientation from the imu
    // because of potential drift, and a missing proper initial value
    Eigen::Affine3d body2Odometry = lastBody2Odometry.getTransform();
    body2Odometry.linear() = R_body2World.toRotationMatrix();
    lastBody2Odometry.setTransform(body2Odometry);

    // this will update the global rotation covariance to that of the IMU,
    // which is providing the error estimate itself
    Eigen::Matrix<double,6,6> cov = lastBody2Odometry.getCovariance();
    cov.topLeftCorner<3,3>() = (Eigen::Vector3d(2, 2, 10) * M_PI/180.0).array().square().matrix().asDiagonal();
    cov.bottomLeftCorner<3,3>().setZero();
    cov.topRightCorner<3,3>().setZero();
    lastBody2Odometry.setCovariance( cov );

    state.sourceFrame = _body_frame.get();
    state.targetFrame = _odometry_frame.get();
    lastBody2Odometry.copyToRigidBodyState(state);

    // write to port for summed odometry readings
    _odometry_samples.write(state);

    // also write out the body state
    _bodystate_samples.write(body_state);
}

/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See Skid4OdometryTask.hpp for more detailed
// documentation about them.

bool Skid4OdometryTask::configureHook()
{
    od = boost::shared_ptr<odometry::Skid4Odometry>(new odometry::Skid4Odometry(
	    _odometry_config.get(),
	    _wheelRadiusAvg.get(),
	    _trackWidth.get(),
	    _wheelBase.get()));

    _body2imu_enu.registerUpdateCallback(boost::bind(&Skid4OdometryTask::body2imu_enuTransformerCallback, this, _1));

    return Skid4OdometryTaskBase::configureHook();
}

bool Skid4OdometryTask::startHook()
{   
    lastBody2Odometry = envire::TransformWithUncertainty::Identity();
    return true;
}

void Skid4OdometryTask::updateHook()
{
    Skid4OdometryTaskBase::updateHook();
}

// void Skid4OdometryTask::errorHook()
// {
//     Skid4OdometryTaskBase::errorHook();
// }
// void Skid4OdometryTask::stopHook()
// {
//     Skid4OdometryTaskBase::stopHook();
// }
// void Skid4OdometryTask::cleanupHook()
// {
//     Skid4OdometryTaskBase::cleanupHook();
// }

