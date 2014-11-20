/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef ODOMETRY_PURESKID_TASK_HPP
#define ODOMETRY_PURESKID_TASK_HPP

#define _USE_MATH_DEFINES

#include "odometry/pureSkidBase.hpp"
#include <odometry/Odometry.hpp>
#include <math.h>
#include <base/Angle.hpp>
#include <base/Eigen.hpp>

namespace odometry {

    /*! \class pureSkid
     * \brief The task context provides and requires services. It uses an ExecutionEngine to perform its functions.
     * Essential interfaces are operations, data flow ports and properties. These interfaces have been defined using the oroGen specification.
     * In order to modify the interfaces you should (re)use oroGen and rely on the associated workflow.
     * normal skid-steering odometry for any number of skid steering wheels, with option to use IMU ori instead of odometry ori
     * \details
     * The name of a TaskContext is primarily defined via:
     \verbatim
     deployment 'deployment_name'
         task('custom_task_name','odometry::pureSkid')
     end
     \endverbatim
     *  It can be dynamically adapted when the deployment is called with a prefix argument.
     */
    class pureSkid : public pureSkidBase
    {
	friend class pureSkidBase;
    protected:

        /** Odometry object */
        boost::shared_ptr<odometry::SkidOdometry> odometry;

        /**
        * Can contain information about right and left engines.
        */
        template <typename T>
        struct engine {
            T left;
            T right;
        };

        struct {
            /**
             * Timestamp.
             */
            base::Time ts;
            /**
             * Contains the delta orientation of the timestep.
             */
            Eigen::Quaterniond delta_ori;
            
            /**
             * Contains the absolute orientation since initialization.
             */
            Eigen::Quaterniond orientation;
            /**
             * Speed of the engine [rad/s].
             */
            engine<double> engine_speed;
            /**
             * Count of "wheels". Split up into right and left to support uneven wheel arrangement.
             */
            engine<double> engine_count;
            /**
            * Contains body speed in [m/s].
            */
            double body_speed;
            /**
             * Informations about Joints at the given timestamp in ts.
             */
            base::samples::Joints actuator_sample;
            
        } actual_timestep, previous_timestep;
        
        /**
         * This variable was created to fix/ignore the first measurement.
         * Because there is no previous one, the actual measurement is copied
         * into the previous one to have a delta pose of zero.
         */
        bool prev_sample_exists;
        
        /**
         * Wheelnames to distinguish between every wheel.
         */
        std::vector<std::string> leftWheelNames;
        std::vector<std::string> rightWheelNames;


        /**
         * Copies the actual_timestep into the previous_timestep, then reads
         * the new Joints from the input port.
         * If it is called for the first time, it copies the new Joints also into
         * the previous_timestep to prevent runtime errors.
         */
        void prepareForNewState();
        
        /**
         * Calculates the actual average body speed by getting the average speed
         * of left and right wheels.
         * Stores the result in actual_timestep.body_speed
         */
        void calcBodySpeed();
        
        /**
         * Calculates the difference of the orientation between the last two timesteps.
         * Stores the result in actual_timestep.delta_ori
         */
        void calcDeltaOri();
        
        /**
         * Updates the body state via odometry->update.
         * Be sure you called prepareForNewState(), calcBodySpeed() and calcDeltaOri()
         * previously, or set the required parameters in actual_timestep and
         * previous_timestep by yourself.
         */
        void updateBodyState();
        
        
        virtual void actuator_samplesTransformerCallback(const base::Time &ts, const ::base::samples::Joints &actuator_samples_sample);

    public:
        /** TaskContext constructor for pureSkid
         * \param name Name of the task. This name needs to be unique to make it identifiable via nameservices.
         * \param initial_state The initial TaskState of the TaskContext. Default is Stopped state.
         */
        pureSkid(std::string const& name = "odometry::pureSkid");

        /** TaskContext constructor for pureSkid
         * \param name Name of the task. This name needs to be unique to make it identifiable for nameservices.
         * \param engine The RTT Execution engine to be used for this task, which serialises the execution of all commands, programs, state machines and incoming events for a task.
         *
         */
        pureSkid(std::string const& name, RTT::ExecutionEngine* engine);

        /** Default deconstructor of pureSkid
         */
	~pureSkid();

        /** This hook is called by Orocos when the state machine transitions
         * from PreOperational to Stopped. If it returns false, then the
         * component will stay in PreOperational. Otherwise, it goes into
         * Stopped.
         *
         * It is meaningful only if the #needs_configuration has been specified
         * in the task context definition with (for example):
         \verbatim
         task_context "TaskName" do
           needs_configuration
           ...
         end
         \endverbatim
         */
        bool configureHook();

        /** This hook is called by Orocos when the state machine transitions
         * from Stopped to Running. If it returns false, then the component will
         * stay in Stopped. Otherwise, it goes into Running and updateHook()
         * will be called.
         */
        bool startHook();

        /** This hook is called by Orocos when the component is in the Running
         * state, at each activity step. Here, the activity gives the "ticks"
         * when the hook should be called.
         *
         * The error(), exception() and fatal() calls, when called in this hook,
         * allow to get into the associated RunTimeError, Exception and
         * FatalError states.
         *
         * In the first case, updateHook() is still called, and recover() allows
         * you to go back into the Running state.  In the second case, the
         * errorHook() will be called instead of updateHook(). In Exception, the
         * component is stopped and recover() needs to be called before starting
         * it again. Finally, FatalError cannot be recovered.
         */
        void updateHook();

        /** This hook is called by Orocos when the component is in the
         * RunTimeError state, at each activity step. See the discussion in
         * updateHook() about triggering options.
         *
         * Call recover() to go back in the Runtime state.
         */
        void errorHook();

        /** This hook is called by Orocos when the state machine transitions
         * from Running to Stopped after stop() has been called.
         */
        void stopHook();

        /** This hook is called by Orocos when the state machine transitions
         * from Stopped to PreOperational, requiring the call to configureHook()
         * before calling start() again.
         */
        void cleanupHook();
    };
}

#endif

