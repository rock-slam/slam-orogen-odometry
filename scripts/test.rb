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
  skidTask.wheelRadiusAvg = 0.2;
  skidTask.trackWidth = 0.8;
  skidTask.wheelBase = 1.4;
  skidTask.leftWheelNames do |l|
    l[0] = "leftChain"
  end
  skidTask.rightWheelNames do |r|
    r[0] = "rightChain"
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
  rbsDynTransform.sourceFrame = "sensor"
  rbsDynTransform.targetFrame = "body"
  joints = actuatorInput.new_sample
  joints.names do |n|
    n[0] = "leftChain"
    n[1] = "rightChain"
  end
  e = joints.elements.element_t.new
  e.position = 0.0
  joints.elements.push(e)
  joints.elements.push(e)
  
  
  puts "Starting Task..."
  
  ## Start the tasks ##
  skidTask.start
  
  dynamicTransformation.write(rbsDynTransform)
  
  while true
    joints.elements do |e|
      e.position += 0.1
    end
    joints.time = Types::Base::Time.new
    #puts "#{joints.time}"
    actuatorInput.write(joints)
    
      sleep 0.5
  end
end
