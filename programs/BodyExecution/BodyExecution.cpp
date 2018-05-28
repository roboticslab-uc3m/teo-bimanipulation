// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "BodyExecution.hpp"

namespace teo
{

/************************************************************************/

bool BodyExecution::configure(yarp::os::ResourceFinder &rf)
{
    std::string robot = rf.check("robot",yarp::os::Value(DEFAULT_ROBOT),"name of /robot to be used").asString();

    printf("--------------------------------------------------------------\n");
    if (rf.check("help"))
    {
        printf("BodyExecution options:\n");
        printf("\t--help (this help)\t--from [file.ini]\t--context [path]\n");
        printf("\t--robot: %s [%s]\n",robot.c_str(),DEFAULT_ROBOT);
        ::exit(0);
    }

    std::string bodyExecutionStr("/bodyExecution");


    // ------ HEAD -------
    yarp::os::Property headOptions;
    headOptions.put("device","remote_controlboard");
    headOptions.put("remote",robot+"/head");
    headOptions.put("local",bodyExecutionStr+robot+"/head");
    headDevice.open(headOptions);
    if(!headDevice.isValid()) {
      printf("robot head device not available.\n");
      headDevice.close();
      yarp::os::Network::fini();
      return false;
    }

    if (!headDevice.view(headIControlMode2) ) { // connecting our device with "control mode 2" interface, initializing which control mode we want (position)
        printf("[warning] Problems acquiring headIControlMode2 interface\n");
        return false;
    } else printf("[success] Acquired headIControlMode2 interface\n");

    if (!headDevice.view(headIPositionControl2) ) { // connecting our device with "position control 2" interface (configuring our device: speed, acceleration... and sending joint positions)
        printf("[warning] Problems acquiring headIPositionControl2 interface\n");
        return false;
    } else printf("[success] Acquired headIPositionControl2 interface\n");


    // ------ LEFT ARM -------
    yarp::os::Property leftArmOptions;
    leftArmOptions.put("device","remote_controlboard");
    leftArmOptions.put("remote",robot+"/leftArm");
    leftArmOptions.put("local",bodyExecutionStr+robot+"/leftArm");
    leftArmDevice.open(leftArmOptions);
    if(!leftArmDevice.isValid()) {
      printf("robot leftArm device not available.\n");
      leftArmDevice.close();
      yarp::os::Network::fini();
      return false;
    }

    if (!leftArmDevice.view(leftArmIControlMode2) ) { // connecting our device with "control mode 2" interface, initializing which control mode we want (position)
        printf("[warning] Problems acquiring leftArmIControlMode2 interface\n");
        return false;
    } else printf("[success] Acquired leftArmIControlMode2 interface\n");

    if (!leftArmDevice.view(leftArmIPositionControl2) ) { // connecting our device with "position control 2" interface (configuring our device: speed, acceleration... and sending joint positions)
        printf("[warning] Problems acquiring leftArmIPositionControl2 interface\n");
        return false;
    } else printf("[success] Acquired leftArmIPositionControl2 interface\n");


    // ------ RIGHT ARM -------
    yarp::os::Property rightArmOptions;
    rightArmOptions.put("device","remote_controlboard");
    rightArmOptions.put("remote",robot+"/rightArm");
    rightArmOptions.put("local",bodyExecutionStr+robot+"/rightArm");
    rightArmDevice.open(rightArmOptions);
    if(!rightArmDevice.isValid()) {
      printf("robot rightArm device not available.\n");
      rightArmDevice.close();
      yarp::os::Network::fini();
      return false;
    }

    if (!rightArmDevice.view(rightArmIControlMode2) ) { // connecting our device with "control mode 2" interface, initializing which control mode we want (position)
        printf("[warning] Problems acquiring rightArmIControlMode2 interface\n");
        return false;
    } else printf("[success] Acquired rightArmIControlMode2 interface\n");


    if (!rightArmDevice.view(rightArmIPositionControl2) ) { // connecting our device with "position control 2" interface (configuring our device: speed, acceleration... and sending joint positions)
        printf("[warning] Problems acquiring rightArmIPositionControl2 interface\n");
        return false;
    } else printf("[success] Acquired rightArmIPositionControl2 interface\n");

    //-- Set control modes
    int headAxes;
    headIPositionControl2->getAxes(&headAxes);
    std::vector<int> headControlModes(headAxes,VOCAB_CM_POSITION);
    if(! headIControlMode2->setControlModes( headControlModes.data() )){
        printf("[warning] Problems setting position control mode of: head\n");
        return false;
    }
    
    int leftArmAxes;
    leftArmIPositionControl2->getAxes(&leftArmAxes);
    std::vector<int> leftArmControlModes(leftArmAxes,VOCAB_CM_POSITION);
    if(! leftArmIControlMode2->setControlModes( leftArmControlModes.data() )){
        printf("[warning] Problems setting position control mode of: left-arm\n");
        return false;
    }

    int rightArmAxes;
    rightArmIPositionControl2->getAxes(&rightArmAxes);
    std::vector<int> rightArmControlModes(rightArmAxes,VOCAB_CM_POSITION);
    if(! rightArmIControlMode2->setControlModes(rightArmControlModes.data())){
        printf("[warning] Problems setting position control mode of: right-arm\n");
        return false;
    }

    // -- Configuring ports
    inDialogPort.open("/bodyExecution/rpc:s");
    inDialogPort.setReader(*this);  //-- Callback reader: avoid need to call inSrPort.read().

    return this->start();  //-- Start the thread (calls run).
}

/************************************************************************/

bool BodyExecution::interruptModule()
{
    this->stop();
    inDialogPort.interrupt();
    rightArmDevice.close();
    leftArmDevice.close();
    headDevice.close();
    return true;
}

/************************************************************************/

double BodyExecution::getPeriod()
{
    return 4.0; // Fixed, in seconds, the slow thread that calls updateModule below
}

/************************************************************************/

bool BodyExecution::updateModule()
{
    printf("Entered updateModule...\n");

    return true;
}

/************************************************************************/

bool BodyExecution::jointsMoveAndWait(std::vector<double>& leftArm, std::vector<double> &rightArm, std::vector<double> &head)
{
    // -- Configuring Speeds and Accelerations

    // -- Arms
    std::vector<double> armSpeeds(7,25.0); // 7,30.0
    std::vector<double> armAccelerations(7,25.0); // 7,30.0
    // -- Head
    std::vector<double> headSpeed(2,25.0); // 7,30.0
    std::vector<double> headAcceleration(2,25.0); // 7,30.0

    // -- configuring..

    if(!headIPositionControl2->setRefSpeeds(headSpeed.data())){
        printf("[Error] Problems setting reference speed on head joints.\n");
        return false;
    }
    
    if(!rightArmIPositionControl2->setRefSpeeds(armSpeeds.data())){
        printf("[Error] Problems setting reference speed on right-arm joints.\n");
        return false;
    }
    if(!leftArmIPositionControl2->setRefSpeeds(armSpeeds.data())){
        printf("[Error] Problems setting reference speed on left-arm joints.\n");
        return false;
    }
    if(!headIPositionControl2->setRefAccelerations(headAcceleration.data())){
        printf("[Error] Problems setting reference acceleration on head joints.\n");
        return false;
    }
    if(!rightArmIPositionControl2->setRefAccelerations(armAccelerations.data())){
        printf("[Error] Problems setting reference acceleration on right-arm joints.\n");
        return false;
    }
    if(!leftArmIPositionControl2->setRefAccelerations(armAccelerations.data())){
        printf("[Error] Problems setting reference acceleration on left-arm joints.\n");
        return false;
    }


    // -- move to position
    if(!headIPositionControl2->positionMove( head.data() )){
            printf("[Error: positionMove] Problems setting new reference point for head axes.\n");
            return false;
    }
    if(!rightArmIPositionControl2->positionMove( rightArm.data() )){
        printf("[Error: positionMove] Problems setting new reference point for right-arm axes.\n");
        return false;
    }
    if(!leftArmIPositionControl2->positionMove( leftArm.data() )){
            printf("[Error: positionMove] Problems setting new reference point for left-arm axes.\n");
            return false;
    }


    // -- checking movement done...
    bool doneRight = false;
    bool doneLeft = false;
    bool doneHead = false;

    while(!doneRight)
    {
        yarp::os::Time::delay(0.1);
        rightArmIPositionControl2->checkMotionDone(&doneRight);
    }

    while(!doneLeft)
    {
        yarp::os::Time::delay(0.1);
        leftArmIPositionControl2->checkMotionDone(&doneLeft);
    }    

/*  // to avoid problems, we have commented checkMotionDone for the head
    while(!doneHead)
    {
        yarp::os::Time::delay(0.1);
        headIPositionControl2->checkMotionDone(&doneHead);
    }
*/
    //printf("\n");
    return true;
}

/************************************************************************/

bool BodyExecution::read(yarp::os::ConnectionReader& connection)
{
     yarp::os::Bottle in, out; // in: the VOCAB_STATE, out: boolean to check if the movement has finished
     bool ok = in.read(connection);
     //if (!ok) return false;

     state = in.get(0).asVocab();

     if(state == VOCAB_RETURN_MOVEMENT_STATE){

         // -- Gets a way to reply to the message, if possible.
         ConnectionWriter *returnToSender = connection.getWriter();

         if(done) {
             out.addInt(1); // done = 1 (true)
         }
         if (returnToSender!=NULL)
             out.write(*returnToSender);
     }
     return true;
}

/************************************************************************/

void BodyExecution::run()
{
    while(true){
        printf("Movement 1\n");
        {
            double rightArmPoss[7] = {-15.7117919921875, -11.230224609375, 0.175746917724609, 9.22671318054199, -0.509674072265625, 37.6098403930664, 0.0};
            double lefArmPoss[7] = {15.7117919921875, 11.230224609375, -0.175746917724609, -9.22671318054199, 0.509674072265625, -37.6098403930664, 0.0};
            std::vector<double> rightArm(&rightArmPoss[0], &rightArmPoss[0]+7); //teoSim (+6) teo (+7)
            std::vector<double> leftArm(&lefArmPoss[0], &lefArmPoss[0]+7);
            std::vector<double> head(2,0.0);
            jointsMoveAndWait(leftArm,rightArm,head);
            getchar();
        }

        printf("Movement 2\n");
        {
            double rightArmPoss[7] = {-42.0738220214844, -13.2513122558594, 3.16344451904297, 53.8664321899414, -0.509674072265625, 38.4007034301758, 0.0};
            double lefArmPoss[7] = {42.0738220214844, 13.2513122558594, -3.16344451904297, -53.8664321899414, 0.509674072265625, -38.4007034301758, 0.0};
            std::vector<double> rightArm(&rightArmPoss[0], &rightArmPoss[0]+7); //teoSim (+6) teo (+7)
            std::vector<double> leftArm(&lefArmPoss[0], &lefArmPoss[0]+7);
            std::vector<double> head(2,0.0);
            jointsMoveAndWait(leftArm,rightArm,head);
            getchar();
        }

        printf("Movement 3\n");
        {
            double rightArmPoss[7] = {-57.1001892089844, -15.2723999023438, 10.8084354400635, 57.3813705444336, -0.5975341796875, 82.0738143920898, 0.0};
            double lefArmPoss[7] = {57.1001892089844, 15.2723999023438, -10.8084354400635, -57.3813705444336, 0.5975341796875, -82.0738143920898, 0.0};
            std::vector<double> rightArm(&rightArmPoss[0], &rightArmPoss[0]+7); //teoSim (+6) teo (+7)
            std::vector<double> leftArm(&lefArmPoss[0], &lefArmPoss[0]+7);
            std::vector<double> head(2,0.0);
            jointsMoveAndWait(leftArm,rightArm,head);
            getchar();
        }
    }
}

/************************************************************************/

}  // namespace teo

