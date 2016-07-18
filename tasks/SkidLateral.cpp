/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "SkidLateral.hpp"
#include <sstream>

using namespace odometry;

SkidLateral::SkidLateral(std::string const& name)
    : SkidLateralBase(name)
{
}

SkidLateral::SkidLateral(std::string const& name, RTT::ExecutionEngine* engine)
    : SkidLateralBase(name, engine)
{
}

SkidLateral::~SkidLateral()
{
}

void SkidLateral::actuator_samplesTransformerCallback(const base::Time &ts, const ::base::samples::Joints &actuator_samples)
{
    currentActuatorSample = actuator_samples;
    actuatorUpdated = true;
    gotActuatorReading = true;
}

void SkidLateral::printInvalidSample()
{
    std::cerr << "Invalid actuator sample:" << std::endl;
    std::cerr << "  Expected the following joint names:" << std::endl;
    std::stringstream s;
    std::copy(leftWheelNames.begin(),leftWheelNames.end(), std::ostream_iterator<std::string>(s,", "));
    std::copy(rightWheelNames.begin(),rightWheelNames.end(), std::ostream_iterator<std::string>(s,", "));
    std::copy(leftSteeringNames.begin(),leftSteeringNames.end(), std::ostream_iterator<std::string>(s,", "));
    std::copy(rightSteeringNames.begin(),rightSteeringNames.end(), std::ostream_iterator<std::string>(s,", "));
    std::cerr << "    " << s.str() << std::endl;
    std::cerr << "  But got the following joint names:" << std::endl;
    std::stringstream s2;
    std::copy(currentActuatorSample.names.begin(),currentActuatorSample.names.end(), std::ostream_iterator<std::string>(s2,", "));
    std::cerr << "    " << s2.str() << std::endl;
}

void SkidLateral::body2imu_enuTransformerCallback(const base::Time& ts)
{
    //we need to receive an actuator reading first
    if(!gotActuatorReading)
        return;
    
    // use the transformer to get the body2world transformation 
    // this should include the imu reading
    base::Transform3d body2IMUWorld;
    if( !_body2imu_world.get( ts, body2IMUWorld ) )
        return;

    // calculates the rotation from body to world base on the orientation measurment 
    Eigen::Quaterniond R_body2World(body2IMUWorld.rotation()); 

    try
    {
        odometry->update(currentActuatorSample, currentActuatorSample, R_body2World);
    }
    catch(const base::NamedVector<base::JointState>::InvalidName &e)
    {
        printInvalidSample();
        error(EXCEPTION);
    }

    // create a transform with uncertainty based on the odometry 
    envire::TransformWithUncertainty body2PrevBody( 
            odometry->getPoseDelta().toTransform(),
            odometry->getPoseError() );

    // push the transformations
    pushState( ts, body2PrevBody, R_body2World );

}

/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See SkidLateral.hpp for more detailed
// documentation about them.

bool SkidLateral::configureHook()
{
    if (! SkidLateralBase::configureHook())
        return false;

    rightWheelNames = _rightWheelNames.get();
    leftWheelNames = _leftWheelNames.get();
    
    rightSteeringNames = _rightSteeringNames.get();
    leftSteeringNames = _leftSteeringNames.get();

    wheelRadius = _wheelRadiusAvg.value();

    odometry = boost::shared_ptr<odometry::SkidOdometry>(new odometry::SkidOdometry(
            _odometry_config.get(),
            _wheelRadiusAvg.get(),
            _trackWidth.get(),
            _wheelBase.get(),
            leftWheelNames, rightWheelNames, leftSteeringNames, rightSteeringNames));

    _body2imu_world.registerUpdateCallback(boost::bind(&SkidLateral::body2imu_enuTransformerCallback, this, _1));

    return true;
}
bool SkidLateral::startHook()
{
    if (! SkidLateralBase::startHook())
        return false;
    
    prev_ts = base::Time();
    actuatorUpdated = false;
    gotActuatorReading = false;
    
    return true;
}
void SkidLateral::updateHook()
{
    SkidLateralBase::updateHook();
}
void SkidLateral::errorHook()
{
    SkidLateralBase::errorHook();
}
void SkidLateral::stopHook()
{
    SkidLateralBase::stopHook();
}
void SkidLateral::cleanupHook()
{
    SkidLateralBase::cleanupHook();
}
