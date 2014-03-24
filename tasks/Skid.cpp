/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Skid.hpp"

using namespace odometry;

Skid::Skid(std::string const& name)
    : SkidBase(name)
{
}

Skid::Skid(std::string const& name, RTT::ExecutionEngine* engine)
    : SkidBase(name, engine)
{
}

Skid::~Skid()
{
}

void Skid::actuator_samplesTransformerCallback(const base::Time &ts, const ::base::samples::Joints &actuator_samples)
{
    currentActuatorSample = actuator_samples;
    actuatorUpdated = true;
}

double Skid::getMovingSpeed()
{
    if(!actuatorUpdated)
        return lastMovingSpeed;
    
    // calculate average speed of all wheels as velocity over ground
    int numWheels = 0;
    for(std::vector<std::string>::const_iterator it = rightWheelNames.begin();
        it != rightWheelNames.end(); it++)
    {
        base::JointState const &state(currentActuatorSample[*it]);
        if(!state.hasSpeed())
          {
            lastMovingSpeed = 0;
            return lastMovingSpeed;
            throw std::runtime_error("Did not get needed speed value");
          }
        lastMovingSpeed += state.speed;
        numWheels++;
    }

    for(std::vector<std::string>::const_iterator it = leftWheelNames.begin();
        it != leftWheelNames.end(); it++)
    {
        base::JointState const &state(currentActuatorSample[*it]);
        if(!state.hasSpeed())
          {
            lastMovingSpeed = 0;
            return lastMovingSpeed;
            throw std::runtime_error("Did not get needed speed value");
          }
        lastMovingSpeed += state.speed;
        numWheels++;
    }

    lastMovingSpeed = lastMovingSpeed / numWheels * wheelRadius;    
    actuatorUpdated = false;

    return lastMovingSpeed;
}


void Skid::body2imu_enuTransformerCallback(const base::Time& ts)
{
    // use the transformer to get the body2world transformation 
    // this should include the imu reading
    base::Transform3d body2IMUWorld;
    if( !_body2imu_world.get( ts, body2IMUWorld ) )
	return;

    // calculates the rotation from body to world base on the orientation measurment 
    Eigen::Quaterniond R_body2World(body2IMUWorld.rotation()); 

    if(!usePosition)
    {
        double moving_dist = 0;
        double moving_speed = getMovingSpeed();
        // calculate the travelled distance based on the speed over ground
        // and the time since the last update
        if( prev_ts.isNull())
            prev_ts = ts;

        double dt = (ts - prev_ts).toSeconds();
        moving_dist = moving_speed * dt;
        prev_ts = ts;
        
        // make sure we don't have any nans or infs flying around
        if( std::isfinite( moving_dist ) )
        {
            // update the odometry
            odometry->update( moving_dist, R_body2World );
        }
        else
        {
            LOG_ERROR_S << "Calculation of distance moved between two time steps is NaN." << std::endl;
            return;
        }
    }
    else
    {
        odometry->update(currentActuatorSample, R_body2World);
    }

    // create a transform with uncertainty based on the odometry 
    envire::TransformWithUncertainty body2PrevBody( 
            odometry->getPoseDelta().toTransform(),
            odometry->getPoseError() );

    // push the transformations
    pushState( ts, body2PrevBody, R_body2World );

}

/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See Skid.hpp for more detailed
// documentation about them.

bool Skid::configureHook()
{
    if (! SkidBase::configureHook())
        return false;

    rightWheelNames = _rightWheelNames.get();
    leftWheelNames = _leftWheelNames.get();

    wheelRadius = _wheelRadiusAvg.value();
    usePosition = _usePosition.get();

    odometry = boost::shared_ptr<odometry::SkidOdometry>(new odometry::SkidOdometry(
	    _odometry_config.get(),
	    _wheelRadiusAvg.get(),
	    _trackWidth.get(),
	    _wheelBase.get(),
            leftWheelNames, rightWheelNames));

    _body2imu_world.registerUpdateCallback(boost::bind(&Skid::body2imu_enuTransformerCallback, this, _1));

    return true;
}
bool Skid::startHook()
{
    if (! SkidBase::startHook())
        return false;
    
    prev_ts = base::Time();
    actuatorUpdated = false;
    lastMovingSpeed = 0.0;

    return true;
}
void Skid::updateHook()
{
    SkidBase::updateHook();
}
void Skid::errorHook()
{
    SkidBase::errorHook();
}
void Skid::stopHook()
{
    SkidBase::stopHook();
}
void Skid::cleanupHook()
{
    SkidBase::cleanupHook();
}
