require 'orocos'
require 'vizkit'
include Orocos

## Initialize orocos ##
Orocos.initialize



## Execute the task 'message_producer::Task' ##
Orocos.run 'odometry::Skid' => 'skidTask' do
  
  ###########################
  # SET PROPERTIES
  ###########################
  puts "Setting properties..."
  ## Get the task context##
  skidTask = Orocos.name_service.get 'skidTask'
  skidTask.actuator_samples_period = 0.5
  skidTask.wheelRadiusAvg = 0.2;
  skidTask.trackWidth = 0.8;
  skidTask.wheelBase = 1.4;
  skidTask.leftWheelNames do |l|
    l[0] = "leftChain"
  end
  skidTask.rightWheelNames do |r|
    r[0] = "rightChain"
  end
  skidTask.transformer_max_latency = 0.5
  #skidTask.useIMU = true;
  
  ###########################
  # SET STATIC TRANSFORMATION
  ###########################
  staticTransform = Types::Base::Samples::RigidBodyState.new
  staticTransform.sourceFrame = "odometry"
  staticTransform.targetFrame = "body"
  staticTransform.position = Eigen::Vector3.new(0, 0, 0)
  staticTransform.orientation = Eigen::Quaternion.new(0, 0, 0, 0)
  staticTransform.time = Time.now
  
  skidTask.static_transformations do |sT|
    sT[0] = staticTransform
  end
  
  
  skidTask.configure
  
  ###########################
  # GET INPUT PORT WRITER
  ###########################
  actuatorInput = skidTask.actuator_samples.writer
  dynamicTransformation = skidTask.dynamic_transformations.writer
  
  ###########################
  # SET INPUT OBJECTS
  ###########################
  #Types::Base::Samples::RigidBodyState.new
  #Types::Base::Samples::Joints.new
  
  rbsDynTransform = dynamicTransformation.new_sample
  rbsDynTransform.position = Eigen::Vector3.new(0, 0, 0)
  rbsDynTransform.sourceFrame = "imu_world"
  rbsDynTransform.targetFrame = "body"
  
  joints = actuatorInput.new_sample
  joints.names.push("leftChain")
  joints.names.push("rightChain")
  e = joints.elements.element_t.new
  e.position = 0.0
  joints.elements.push(e)
  joints.elements.push(e)
  #puts joints.elements.methods.sort
  
  puts "Starting Task..."
  
  ## Start the tasks ##
  skidTask.start
  
  while true
    joints.time = Time.now
    #rbsDynTransform.time = joints.time
    #puts "#{joints.time}"
    dynamicTransformation.write(rbsDynTransform)
    actuatorInput.write(joints)
    
      sleep 0.2
  end
end
