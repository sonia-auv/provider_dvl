#include "provider_dvl/provider_dvl_node.h"

///Pathfinder Implementation
//==============================================================================
// C / D T O R S   S E C T I O N

//------------------------------------------------------------------------------
//
PathfinderDvl::PathfinderDvl(const ros::NodeHandlePtr & nh, std::string hostName, size_t pUDP, size_t pTCP)
: ProviderDvl(nh,hostName,pUDP, pTCP,sizeof(mDvl_data))
{
  Connect();
  SetupROSCommunication();
  setRate(20);
  mSendReceivedMessage = std::thread(std::bind(&PathfinderDvl::SendReceivedMessageThread, this));
}
//------------------------------------------------------------------------------
//

//==============================================================================
// M E T H O D   S E C T I O N

//------------------------------------------------------------------------------
//


void PathfinderDvl::SetupROSCommunication() {


  dvl_velocity_publisher_ = nh()->advertise<sonia_common::BodyVelocityDVL>( "/provider_dvl/dvl_velocity", 100);
  dvl_leak_sensor_publisher_ = nh()->advertise<std_msgs::Bool>("/provider_dvl/dvl_leak_sensor", 100);
  enableDisablePingSub = nh()->subscribe("/provider_dvl/enable_disable_ping", 100, &PathfinderDvl::EnableDisablePingCallback, this);
}

void PathfinderDvl::EnableDisablePingCallback(const std_msgs::Bool& msg)
{
    std::string str, cmd;
    
    if(msg.data == true)
    {   
        str = "===\n";
        socket().Send(&str[0]);
        ros::Duration(5).sleep();
        cmd = "CS\n";
        socket().Send(&cmd[0]);
        ROS_INFO("Start Ping Sent");
    }
    else if(msg.data == false)
    {
        str = "===\n";
        socket().Send(&str[0]);
        ROS_INFO("Stop Ping Sent");
    }
    else
    {
        ROS_WARN_STREAM("Message isn't boolean");
    }
}

void PathfinderDvl::SendReceivedMessageThread()
{
    ros::Rate r(rate()); // 20 Hz

    while(!ros::isShuttingDown())
    {
        mSocket.Receive();

        ROS_DEBUG("Data received");

        getData<PathfinderFormat_t>(mDvl_data);

        ROS_DEBUG("Data obtained");

        if(mDvl_data.pd4.pathfinderDataId == 0x7D)
        {
            ROS_DEBUG("ID correct");
            if(mDvl_data.pd4.checksum == CalculateChecksum<PathfinderFormat_t>(reinterpret_cast<uint8_t*>(mSocket.GetRawData())))
            {
                sonia_common::BodyVelocityDVL message;

                message.header.stamp.sec = mDvl_data.pd4.secondFirstPing;
                message.header.stamp.nsec = mDvl_data.pd4.hundredthFirstPing;
                message.header.frame_id = "/ENU"; //PD4

                message.xVelBtm = ((double_t)mDvl_data.pd4.xVelBtm)/1000.0;
                message.yVelBtm = ((double_t)mDvl_data.pd4.yVelBtm)/1000.0;
                message.zVelBtm = ((double_t)mDvl_data.pd4.zVelBtm)/1000.0;
                message.eVelBtm = ((double_t)mDvl_data.pd4.eVelBtm)/1000.0;

                message.velocity1 = ((double_t)mDvl_data.pd4.velocity1)/1000.0;
                message.velocity2 = ((double_t)mDvl_data.pd4.velocity2)/1000.0;
                message.velocity3 = ((double_t)mDvl_data.pd4.velocity3)/1000.0;
                message.velocity4 = ((double_t)mDvl_data.pd4.velocity4)/1000.0;

                dvl_velocity_publisher_.publish(message);
            }
        }
        else
        {
            ROS_INFO("Pathfinder ID mismatch : %d", 0x7D);
        }
        r.sleep();
    }
}


/// Nortek Implementation
//==============================================================================
// C / D T O R S   S E C T I O N

//------------------------------------------------------------------------------
//
NortekDvl::NortekDvl(const ros::NodeHandlePtr & nh, std::string hostName, size_t pTCP)
: ProviderDvl(nh,hostName,0, pTCP,sizeof(mDvl_data)), depthOffset_{}
{
  Connect();
  SetupROSCommunication();
  setRate(20);
  mSendReceivedMessage = std::thread(std::bind(&NortekDvl::SendReceivedMessageThread, this));
}
//------------------------------------------------------------------------------
//

//==============================================================================
// M E T H O D   S E C T I O N

//------------------------------------------------------------------------------
//

void NortekDvl::SetupROSCommunication() 
{
    dvl_speed_publisher_ = nh()->advertise<sonia_common::BodyVelocityDVL>("/provider_dvl/dvl_velocity", 100);
    dvl_fluid_pressure_publisher_ = nh()->advertise<sensor_msgs::FluidPressure>("/provider_dvl/dvl_pressure", 100);
    dvl_temperature_publisher_ = nh()->advertise<sensor_msgs::Temperature>("/provider_dvl/dvl_water_temperature", 100);
    dvl_relative_depth_publisher_ = nh()->advertise<std_msgs::Float32>("/provider_dvl/dvl_relative_depth", 100);
    setDepthOffset = nh()->subscribe("/provider_dvl/setDepthOffset", 100, &NortekDvl::setDepthOffsetCallback, this);
    //dvl_bottom_tracking_publisher_ = nh()->advertise<sonia_common::BottomTracking>("/provider_dvl/dvl_data", 100);
}

void NortekDvl::SendReceivedMessageThread()
{
    ros::Rate r(rate()); // 20 HzFloat32

    while (ros::ok())
        {
            mSocket.Receive();

            ROS_DEBUG("Data received");

            getData<NortekFormat_t>(mDvl_data);

            ROS_DEBUG("Data obtained");

            if (mDvl_data.header.sync == 0xA5)
            {
                if (mDvl_data.header.dataChecksum == CalculateChecksum<NortekFormat_t>(reinterpret_cast<uint8_t*>(mSocket.GetRawData())))
                {
                    timestamp_ = ros::Time::now();
                    FillTwistMessage(timestamp_);
                    FillFluidPressureMessage(timestamp_);
                    FillTemperatureMessage(timestamp_);
                    FillRelativeDepthMessage(timestamp_);
                    // FillBottomTracking(timestamp_);
                }
                else
                {
                    ROS_INFO("CheckSum error : %d != %d", mDvl_data.header.dataChecksum, CalculateChecksum<NortekFormat_t>(reinterpret_cast<uint8_t*>(mSocket.GetRawData())));
                }
            }
            else
            {
                ROS_INFO("Nortek ID mismatch : %d != %d", 0xA5, mDvl_data.header.sync);
            }
            r.sleep();
        }
}

void NortekDvl::FillTwistMessage(ros::Time timestamp) {
        sonia_common::BodyVelocityDVL message;

        message.header.stamp = timestamp;
        message.header.frame_id = "/ENU";

        if(mDvl_data.data.status.bit_field.xVelValid)
        {
            message.xVelBtm = ((double_t)mDvl_data.data.velX);
        }
        else
        {
            message.xVelBtm = 0.0;
        }
        if(mDvl_data.data.status.bit_field.yVelValid)
        {
            message.yVelBtm = ((double_t)mDvl_data.data.velY);
        }
        else
        {
            message.yVelBtm = 0.0;
        }
        if(mDvl_data.data.status.bit_field.z1VelValid)
        {
            message.zVelBtm = ((double_t)mDvl_data.data.velZ1);
        }
        else
        {
            message.zVelBtm = 0.0;
        }

        dvl_speed_publisher_.publish(message);
    }

    //------------------------------------------------------------------------------
    //
    void NortekDvl::FillRelativeDepthMessage(ros::Time timestamp)
    {
        std_msgs::Float32 message; // remplacer avec header

        //message.header.stamp = timestamp;
        //message.header.frame_id = "/ENU";
        message.data = ProviderDvl::convertDBarToMeters(mDvl_data.data.pressure - depthOffset_);

        dvl_fluid_pressure_publisher_.publish(message);
    }

    void NortekDvl::FillTemperatureMessage(ros::Time timestamp)
    {
        sensor_msgs::Temperature message; // remplacer avec header

        message.header.stamp = timestamp;
        message.header.frame_id = "/ENU";
        message.temperature = ProviderDvl::convertDBarToMeters(mDvl_data.data.pressure - depthOffset_);

        dvl_fluid_pressure_publisher_.publish(message);
    }

    void NortekDvl::FillFluidPressureMessage(ros::Time timestamp)
    {
        sensor_msgs::FluidPressure message;

        message.header.stamp = timestamp;
        message.header.frame_id = "/ENU";
        message.fluid_pressure = mDvl_data.data.pressure;

        dvl_fluid_pressure_publisher_.publish(message);
    }

    void NortekDvl::setDepthOffsetCallback(const std_msgs::Bool& msg)
    {
        depthOffset_ = mDvl_data.data.pressure;
    }


    // Garder pour pfe capteur
    // void NortekDvl::FillBottomTracking(ros::Time timestamp)
    // {
    //     sonia_common::BottomTracking message;

    //     message.header.stamp = timestamp;
    //     message.header.frame_id = "/ENU";

    //     message.sync = mDvl_data.header.sync;
    //     message.hdrSize = mDvl_data.header.hdrSize;
    //     message.ID = mDvl_data.header.ID;
    //     message.family = mDvl_data.header.family;
    //     message.datasize = mDvl_data.header.dataSize;
    //     message.dataChecksum = mDvl_data.header.dataChecksum;
    //     message.hdrCecksum = mDvl_data.header.hdrChecksum;


    //     message.version = mDvl_data.data.version;
    //     message.offsetOfData = mDvl_data.data.offsetOfData;
    //     message.serial_number = mDvl_data.data.serialNumber;
    //     message.year = mDvl_data.data.year;
    //     message.month = mDvl_data.data.month;
    //     message.day = mDvl_data.data.day;
    //     message.hour = mDvl_data.data.hour;
    //     message.minutes = mDvl_data.data.minute;
    //     message.seconds = mDvl_data.data.seconds;
    //     message.microSeconds100 = mDvl_data.data.microSeconds100;
    //     message.nbBeams = mDvl_data.data.nBeams;
    //     message.error = mDvl_data.data.error;
    //     message.status = mDvl_data.data.status.integer;
    //     message.sound_speed = mDvl_data.data.soundSpeed;
    //     message.temperature  = mDvl_data.data.temperature;
    //     message.pressure  = mDvl_data.data.pressure;
    //     message.velBeam1  = mDvl_data.data.velBeam[0];
    //     message.velBeam2  = mDvl_data.data.velBeam[1];
    //     message.velBeam3  = mDvl_data.data.velBeam[2];
    //     message.velBeam4  = mDvl_data.data.velBeam[3];
    //     message.distBeam1  = mDvl_data.data.distBeam[0];
    //     message.distBeam2 = mDvl_data.data.distBeam[1];
    //     message.distBeam3 = mDvl_data.data.distBeam[2];
    //     message.distBeam4 = mDvl_data.data.distBeam[3];
    //     message.fomBeam1  = mDvl_data.data.fomBeam[0];
    //     message.fomBeam2  = mDvl_data.data.fomBeam[1];
    //     message.fomBeam3  = mDvl_data.data.fomBeam[2];
    //     message.fomBeam4  = mDvl_data.data.fomBeam[3];
    //     message.timeDiff1Beam1  = mDvl_data.data.timeDiff1Beam[0];
    //     message.timeDiff1Beam2  = mDvl_data.data.timeDiff1Beam[1];
    //     message.timeDiff1Beam3  = mDvl_data.data.timeDiff1Beam[2];
    //     message.timeDiff1Beam4  = mDvl_data.data.timeDiff1Beam[3];
    //     message.timeDiff2Beam1  = mDvl_data.data.timeDiff2Beam[0];
    //     message.timeDiff2Beam2  = mDvl_data.data.timeDiff2Beam[1];
    //     message.timeDiff2Beam3  = mDvl_data.data.timeDiff2Beam[2];
    //     message.timeDiff2Beam4  = mDvl_data.data.timeDiff2Beam[3];
    //     message.timeVelEstBeam1  = mDvl_data.data.timeVelEstBeam[0];
    //     message.timeVelEstBeam2  = mDvl_data.data.timeVelEstBeam[1];
    //     message.timeVelEstBeam3  = mDvl_data.data.timeVelEstBeam[2];
    //     message.timeVelEstBeam4  = mDvl_data.data.timeVelEstBeam[3];
    //     message.velX = mDvl_data.data.velX;
    //     message.velY = mDvl_data.data.velY;
    //     message.velZ1 = mDvl_data.data.velZ1;
    //     message.velZ2 = mDvl_data.data.velZ2;
    //     message.fomX = mDvl_data.data.fomX;
    //     message.fomY = mDvl_data.data.fomY;
    //     message.fomZ1 = mDvl_data.data.fomZ1;
    //     message.fomZ2 = mDvl_data.data.fomZ2;
    //     message.timeDiff1X = mDvl_data.data.timeDiff1X;
    //     message.timeDiff1Y = mDvl_data.data.timeDiff1Y;
    //     message.timeDiff1Z1 = mDvl_data.data.timeDiff1Z1;
    //     message.timeDiff1Z2 = mDvl_data.data.timeDiff1Z2;
    //     message.timeDiff2X = mDvl_data.data.timeDiff2X;
    //     message.timeDiff2Y = mDvl_data.data.timeDiff2Y;
    //     message.timeDiff2Z1 = mDvl_data.data.timeDiff2Z1;
    //     message.timeDiff2Z2 = mDvl_data.data.timeDiff2Z2;
    //     message.timeVelEstX = mDvl_data.data.timeVelEstX;
    //     message.timeVelEstY = mDvl_data.data.timeVelEstY;
    //     message.timeVelEstZ1 = mDvl_data.data.timeVelEstZ1;
    //     message.timeVelEstZ2 = mDvl_data.data.timeVelEstZ2;

    //     dvl_bottom_tracking_publisher_.publish(message);
    // }
