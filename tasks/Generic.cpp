/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Generic.hpp"

using namespace odometry;

Generic::Generic(std::string const& name)
    : GenericBase(name)
{
}

Generic::Generic(std::string const& name, RTT::ExecutionEngine* engine)
    : GenericBase(name, engine)
{
}

Generic::~Generic()
{
}

/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See Generic.hpp for more detailed
// documentation about them.

// bool Generic::configureHook()
// {
//     if (! GenericBase::configureHook())
//         return false;
//     return true;
// }
// bool Generic::startHook()
// {
//     if (! GenericBase::startHook())
//         return false;
//     return true;
// }
// void Generic::updateHook()
// {
//     GenericBase::updateHook();
// }
// void Generic::errorHook()
// {
//     GenericBase::errorHook();
// }
// void Generic::stopHook()
// {
//     GenericBase::stopHook();
// }
// void Generic::cleanupHook()
// {
//     GenericBase::cleanupHook();
// }

