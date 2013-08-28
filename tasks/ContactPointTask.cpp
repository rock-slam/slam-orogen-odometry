/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "ContactPointTask.hpp"
#include <odometry/ContactOdometry.hpp>
#include <base/Eigen.hpp>

using namespace odometry;

ContactPointTask::ContactPointTask(std::string const& name)
    : ContactPointTaskBase(name), contactOdometry(NULL)
{
}

ContactPointTask::ContactPointTask(std::string const& name, RTT::ExecutionEngine* engine)
    : ContactPointTaskBase(name, engine), contactOdometry(NULL)
{
}

ContactPointTask::~ContactPointTask()
{
}

void ContactPointTask::contact_samplesTransformerCallback(const base::Time &ts, const ::odometry::BodyContactState &contact_samples_sample)
{
    contactState = contact_samples_sample;
}

void ContactPointTask::body2imu_enuTransformerCallback(const base::Time& ts)
{
    // use the transformer to get the body2world transformation 
    // this should include the imu reading
    base::Transform3d body2IMUWorld;
    if( !_body2imu_enu.get( ts, body2IMUWorld ) )
    {
	return;
    }

    // get only the rotation component
    Eigen::Quaterniond R_body2World = Eigen::Quaterniond( body2IMUWorld.linear() ); 

    contactOdometry->update(contactState, R_body2World);

    if (contactOdometry->state.isValid())
    {
        pushState(ts, *contactOdometry, R_body2World);
    }
}

void ContactPointTask::pushState(base::Time const& ts,
        odometry::Gaussian3D& odometry,
        base::Quaterniond const& R_body2World,
        base::Vector3d const& velocity,
        base::Vector3d const& angular_velocity)
{
    // create a transform with uncertainty based on the odometry 
    envire::TransformWithUncertainty body2PrevBody( 
            odometry.getPoseDelta().toTransform(),
            odometry.getPoseError() );

    // Generate the output for the pose delta
    base::samples::RigidBodyState state;
    state.invalidate();
    state.time = ts;
    body2PrevBody.copyToRigidBodyState( state );
    state.angular_velocity = angular_velocity;
    state.velocity = velocity;

    // write to port if connected
    if( _odometry_delta_samples.connected() )
        _odometry_delta_samples.write( state );

    // also sum up the relative changes in an absolute odometry frame
    // and output to a separate port. 
    lastBody2Odometry = lastBody2Odometry * body2PrevBody;
    // need to set the absolute body to world orientation from the imu
    // because of potential drift, and a missing proper initial value
    Eigen::Affine3d body2Odometry = lastBody2Odometry.getTransform();
    body2Odometry.linear() = R_body2World.toRotationMatrix();
    lastBody2Odometry.setTransform( body2Odometry );

    // this will update the global rotation covariance to that of the IMU,
    // which is providing the error estimate itself
    // TODO remove hardcoded rotation covariance
    Eigen::Matrix<double,6,6> cov = lastBody2Odometry.getCovariance();
    cov.topLeftCorner<3,3>() = (Eigen::Vector3d( 2, 2, 10 ) * M_PI/180.0 ).array().square().matrix().asDiagonal();
    cov.bottomLeftCorner<3,3>().setZero();
    cov.topRightCorner<3,3>().setZero();
    lastBody2Odometry.setCovariance( cov );

    state.sourceFrame = _body_frame.get();
    state.targetFrame = _odometry_frame.get();
    lastBody2Odometry.copyToRigidBodyState( state );

    // write to port for summed odometry readings
    _odometry_samples.write( state );
}


/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See ContactPointTask.hpp for more detailed
// documentation about them.

// bool ContactPointTask::configureHook()
// {
//     if (! ContactPointTaskBase::configureHook())
//         return false;
//     return true;
// }
bool ContactPointTask::startHook()
{
    if (! ContactPointTaskBase::startHook())
        return false;

    delete contactOdometry;
    contactOdometry = new odometry::FootContact(odometryConfiguration);

    // reset absolute odometry integration
    lastBody2Odometry = envire::TransformWithUncertainty::Identity();

    _body2imu_enu.registerUpdateCallback(boost::bind(&ContactPointTask::body2imu_enuTransformerCallback, this, _1));
    
    return true;
}
// void ContactPointTask::updateHook()
// {
//     ContactPointTaskBase::updateHook();
// }
// void ContactPointTask::errorHook()
// {
//     ContactPointTaskBase::errorHook();
// }
// void ContactPointTask::stopHook()
// {
//     ContactPointTaskBase::stopHook();
// }
// void ContactPointTask::cleanupHook()
// {
//     ContactPointTaskBase::cleanupHook();
// }

