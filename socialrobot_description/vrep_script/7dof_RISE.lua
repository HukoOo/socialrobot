-- DO NOT WRITE CODE OUTSIDE OF THE if-then-end SECTIONS BELOW!! (unless the code is a function definition)

if (sim_call_type==sim_childscriptcall_initialization) then

    armJoints={-1,-1,-1,-1,-1,-1,-1}
    for i=1,7,1 do
        armJoints[i]=simGetObjectHandle('7_Joint'..i)
    end
    
    -- Check if the required RosInterface is there:
    moduleName=0
    index=0
    rosInterfacePresent=false
    while moduleName do
        moduleName=simGetModuleName(index)
        if (moduleName=='RosInterface') then
            rosInterfacePresent=true
        end
        index=index+1
    end

    -- Publisher
    if(rosInterfacePresent==true) then     
        
    joints_pub = simExtRosInterface_advertise('/joint_states', 'sensor_msgs/JointState')
    simExtRosInterface_publisherTreatUInt8ArrayAsString(joints_pub)
        
        armJointNames={-1,-1,-1,-1,-1,-1,-1}               
        for i=1,7,1 do
            armJointNames[i]='7dof_RISE_joint_'..i
        end
    end
end


if (sim_call_type==sim_childscriptcall_actuation) then


end


if (sim_call_type==sim_childscriptcall_sensing) then

    if(rosInterfacePresent==true) then   
        simTime = simExtRosInterface_getTime()
       -- Joint State Data Processing & Publish
       local position = {}
       local velocity = {}
       local torque = {}
        
        for i=1,7 do
            position[i] = simGetJointPosition(armJoints[i])
            r, velocity[i] = simGetObjectFloatParameter(armJoints[i],sim_jointfloatparam_velocity)
            torque[i] = simGetJointForce(armJoints[i])
        end

        local joint_msg_data = {}

        joint_msg_data['header'] = {seq = 0, stamp = simTime, frame_id = "7dof_RISE_base_footprint"}
        joint_msg_data['name'] = {}
        
        for i=1,7,1 do
            joint_msg_data['name'][i] = armJointNames[i]
        end
        joint_msg_data['position'] = position
        joint_msg_data['velocity'] = velocity
        joint_msg_data['effort'] = torque

        simExtRosInterface_publish(joints_pub,joint_msg_data)
    end

end


if (sim_call_type==sim_childscriptcall_cleanup) then

    if(rosInterfacePresent==true) then     
        simExtRosInterface_shutdownPublisher(joints_pub)
    end

end