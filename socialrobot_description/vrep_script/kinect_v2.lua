if (sim_call_type==sim_childscriptcall_initialization) then     
    depthCam=simGetObjectHandle('Kinect_v2_depth')
    depthView=simFloatingViewAdd(0.9,0.9,0.2,0.2,0)
    simAdjustView(depthView,depthCam,64)

    colorCam=simGetObjectHandle('Kinect_v2_rgb')
    colorView=simFloatingViewAdd(0.69,0.9,0.2,0.2,0)
    simAdjustView(colorView,colorCam,64)
    
    sensorBaseHandle=simGetObjectAssociatedWithScript(sim_handle_self)
    maxScanDistance=simGetScriptSimulationParameter(sim_handle_self,'maxScanDistance')
    simSetObjectFloatParameter(depthCam,sim_visionfloatparam_far_clipping,maxScanDistance)
    maxScanDistance_=maxScanDistance*0.9999
    
    scanningAngle=simGetScriptSimulationParameter(sim_handle_self,'scanAngle')
    if scanningAngle>70 then scanningAngle=70 end
    if scanningAngle<1 then scanningAngle=1 end
    scanningAngle=scanningAngle*math.pi/180
    simSetObjectFloatParameter(depthCam,sim_visionfloatparam_perspective_angle,scanningAngle)
    
    pubPointclouds=simGetScriptSimulationParameter(sim_handle_self,'pubPointclouds')
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

    -- Enable an image publisher and subscriber:
    if(rosInterfacePresent==true) then
        pub_color=simExtRosInterface_advertise('/image', 'sensor_msgs/Image')
        simExtRosInterface_publisherTreatUInt8ArrayAsString(pub_color)
        pub_depth=simExtRosInterface_advertise('/image_depth', 'sensor_msgs/Image')
        simExtRosInterface_publisherTreatUInt8ArrayAsString(pub_depth)       

        pub_cloud=simExtRosInterface_advertise('/point_cloud', 'sensor_msgs/PointCloud2')
        simExtRosInterface_publisherTreatUInt8ArrayAsString(pub_cloud)

    end
end 


if (sim_call_type==sim_childscriptcall_sensing) then
    if(rosInterfacePresent==true) then
        -- Publish the image of the active vision sensor:
        local color_data,w,h=simGetVisionSensorCharImage(colorCam)
        simTransformImage(color_data,simGetVisionSensorResolution(colorCam) ,4)
        dc={}
        dc['header']={seq=0,stamp=simExtRosInterface_getTime(), frame_id="kinect_v2_rgb"}
        dc['height']=h
        dc['width']=w
        dc['encoding']='rgb8'
        dc['is_bigendian']=1
        dc['step']=w*3
        dc['data']=color_data
        simExtRosInterface_publish(pub_color,dc)


        local depth_data_tmp=simGetVisionSensorDepthBuffer(depthCam+sim_handleflag_codedstring)
        local res,nearClippingPlane=simGetObjectFloatParameter(depthCam,sim_visionfloatparam_near_clipping)
        local res,farClippingPlane=simGetObjectFloatParameter(depthCam,sim_visionfloatparam_far_clipping)
        nearClippingPlane=nearClippingPlane*1000 -- we want mm
        farClippingPlane=farClippingPlane*1000 -- we want mm
        depth_data=simTransformBuffer(depth_data_tmp,sim_buffer_float,farClippingPlane-nearClippingPlane,nearClippingPlane,sim_buffer_uint16)
        local res=simGetVisionSensorResolution(depthCam)
        dd={}
        r,t1,u1=simReadVisionSensor(depthCam)

        dd['header']={seq=0,stamp=simExtRosInterface_getTime(), frame_id="kinect_v2_depth"}
        dd['height']=res[2]
        dd['width']=res[1]
        dd['encoding']='16UC1' 
        dd['is_bigendian']=0
        dd['step']=res[1]*2
        dd['data']=depth_data

        simExtRosInterface_publish(pub_depth,dd)    

        -- Publish pointcloud2
        if (pubPointclouds==true) then
            m1=simGetObjectMatrix(depthCam,-1)
            m01=simGetInvertedMatrix(simGetObjectMatrix(sensorBaseHandle,-1))
            m01=simMultiplyMatrices(m01,m1)

            local cloud_data={}
            if u1 then
                for j=0,u1[2]-1,1 do
                    for i=0,u1[1]-1,1 do
                        w=2+4*(j*u1[1]+i)
                        v1=u1[w+1]
                        v2=u1[w+2]
                        v3=u1[w+3]
                        v4=u1[w+4]
                        if (v4<maxScanDistance_) then
                            p={v1,v2,v3}
                            p=simMultiplyVector(m01,p)
                            table.insert(cloud_data,p[1])
                            table.insert(cloud_data,p[2])
                            table.insert(cloud_data,p[3])
                        end
                    end
                end
            end
            if (#cloud_data>0) then
                t=simGetSystemTime()
                tb_depth={}
                tb_depth['header']={stamp=t, frame_id="kinect_v2_depth"}
                tb_depth['height']= 1 --res[2]    -- 480
                tb_depth['width'] = #cloud_data/3 --res[1]    -- 640
                tb_depth['is_bigendian']=false
                -- Length of a point in bytes
                tb_depth['point_step']= 12   
                -- Length of a row in bytes
                tb_depth['row_step']=  tb_depth['width'] * tb_depth['point_step']
                tb_depth['is_dense']=true
                tb_depth['fields'] = {  { name='x', offset=0, datatype= 7,count=1 } , 
                                            { name='y', offset=4, datatype= 7,count=1 } , 
                                            { name='z', offset=8, datatype= 7,count=1 }  }
                
                tb_depth['data'] =simPackFloats(cloud_data)
                simExtRosInterface_publish(pub_cloud, tb_depth)
            end
        end

    end
end

if (sim_call_type==sim_childscriptcall_cleanup) then 
    if(rosInterfacePresent==true) then
        simExtRosInterface_shutdownPublisher(pub_color)
        simExtRosInterface_shutdownPublisher(pub_depth)
        simExtRosInterface_shutdownPublisher(pub_cloud)
    end
end 


