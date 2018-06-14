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
    yarp::os::Property options;
    options.put("device", "CartesianControlClient"); // our device (a dynamically loaded library)
    options.put("cartesianRemote", "/teoSim/rightArm/CartesianControl"); // remote port through which we'll talk to the server
    options.put("cartesianLocal", "/CartesianControlExample");
    options.put("transform", 1);  // Was yarp::os::Value::getNullValue()


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


    // Set control modes

    headIPositionControl2->getAxes(&numHeadJoints);
    std::vector<int> headControlModes(numHeadJoints,VOCAB_CM_POSITION);
    if(! headIControlMode2->setControlModes( headControlModes.data() )){
        printf("[warning] Problems setting position control mode of: head\n");
        return false;
    }
    
    leftArmIPositionControl2->getAxes(&numLeftArmJoints);
    std::vector<int> leftArmControlModes(numLeftArmJoints,VOCAB_CM_POSITION);
    if(! leftArmIControlMode2->setControlModes( leftArmControlModes.data() )){
        printf("[warning] Problems setting position control mode of: left-arm\n");
        return false;
    }

    rightArmIPositionControl2->getAxes(&numRightArmJoints);
    std::vector<int> rightArmControlModes(numRightArmJoints,VOCAB_CM_POSITION);
    if(! rightArmIControlMode2->setControlModes(rightArmControlModes.data())){
        printf("[warning] Problems setting position control mode of: right-arm\n");
        return false;
    }

    // ----- Configuring KDL Solver for right-arm -----

    if( ! rightArmDevice.view(rightArmIControlLimits) ) {
        printf("Could not view iControlLimits in rightArmDevice\n");
        return false;
    }

    //  Getting the limits of each joint
    printf("---- Joint limits of right-arm ----\n");
    yarp::os::Bottle qrMin, qrMax;
        for(unsigned int joint=0;joint<numRightArmJoints;joint++)
        {
            double min, max;
            rightArmIControlLimits->getLimits(joint,&min,&max);
            qrMin.addDouble(min);
            qrMax.addDouble(max);
            printf("Joint %d limits: [%f,%f]\n",joint,min,max);
        }

    yarp::os::Property rightArmSolverOptions;
    rightArmSolverOptions.fromConfigFile("/usr/local/share/teo-configuration-files/contexts/kinematics/rightArmKinematics.ini");
    rightArmSolverOptions.put("device","KdlSolver");
    //rightArmSolverOptions.put("maxIter", 10000); // iterator configuration
    //rightArmSolverOptions.put("eps", 1e-9);
    rightArmSolverOptions.put("mins", yarp::os::Value::makeList(qrMin.toString().c_str()));
    rightArmSolverOptions.put("maxs", yarp::os::Value::makeList(qrMax.toString().c_str()));
    rightArmSolverDevice.open(rightArmSolverOptions);

    if( ! rightArmSolverDevice.isValid() )
    {
        printf("[ERROR] KDLSolver solver device for right-arm is not valid \n");
        return false;
    }

    if( ! rightArmSolverDevice.view(rightArmICartesianSolver) )
    {
        printf("[ERROR] Could not view iCartesianSolver in KDLSolver \n");
        return false;
    } else printf("[success] Acquired rightArmICartesianSolver interface\n");


    // ----- Configuring KDL Solver for left-arm -----

    if( ! leftArmDevice.view(leftArmIControlLimits) ) {
        printf("Could not view iControlLimits in leftArmDevice\n");
        return false;
    }

    //  Getting the limits of each joint
    printf("---- Joint limits of left-arm ---- \n");
    yarp::os::Bottle qlMin, qlMax;
        for(unsigned int joint=0;joint<numLeftArmJoints;joint++)
        {
            double min, max;
            leftArmIControlLimits->getLimits(joint,&min,&max);
            qlMin.addDouble(min);
            qlMax.addDouble(max);
            printf("Joint %d limits: [%f,%f]\n",joint,min,max);
        }

    yarp::os::Property leftArmSolverOptions;
    leftArmSolverOptions.fromConfigFile("/usr/local/share/teo-configuration-files/contexts/kinematics/leftArmKinematics.ini");
    leftArmSolverOptions.put("device", "KdlSolver");
    rightArmSolverOptions.put("mins", yarp::os::Value::makeList(qlMin.toString().c_str()));
    rightArmSolverOptions.put("maxs", yarp::os::Value::makeList(qlMax.toString().c_str()));
    leftArmSolverDevice.open(leftArmSolverOptions);

    if( ! leftArmSolverDevice.isValid() )
    {
        printf("[ERROR] KDLSolver solver device for left-arm is not valid \n");
        return false;
    }

    if( ! leftArmSolverDevice.view(leftArmICartesianSolver) )
    {
        printf("[ERROR] Could not view iCartesianSolver in KDLSolver\n");
        return false;
    } else printf("[success] Acquired leftArmICartesianSolver interface\n");


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
    //printf("Entered updateModule...\n");

    return true;
}

/************************************************************************/

bool BodyExecution::jointsMoveAndWait(std::vector<double>& leftArm, std::vector<double> &rightArm, std::vector<double> &head, bool wait)
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

    if(wait){
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
    }

/*  // to avoid problems, we have commented checkMotionDone for the head
    while(!doneHead)
    {
        yarp::os::Time::delay(0.1);
        headIPositionControl2->checkMotionDone(&doneHead);
    }
*/
    return true;
}

/************************************************************************/

bool BodyExecution::read(yarp::os::ConnectionReader& connection)
{
     return true;
}

/************************************************************************/
bool BodyExecution::getRightArmFwdKin(std::vector<double> *currentX)
{

    /** ----- Obtain current joint position ----- **/
    int rightArmAxes;
    rightArmIPositionControl2->getAxes(&rightArmAxes);

    if (!rightArmDevice.view(rightArmIEncoders) ) { // connecting our device with "IEncoders" interface
        printf("[warning] Problems acquiring rightArmIEncoders interface\n");
        return false;
    } else printf("[success] Acquired rightArmIEncoders interface\n");

    std::vector<double> currentQ(rightArmAxes);
    if ( ! rightArmIEncoders->getEncoders( currentQ.data() ) ){
        printf("[ERROR] ForgetEncoders failed\n");
        return false;
    }
    else
    {
        printf("[success] Encoder values of right-arm: (");
        for(int i=0; i<rightArmAxes; i++)
            printf("%f ",currentQ[i]);
        printf(")\n ");
    }

    /** ----- Obtain current cartesian position ---------- **/
    if ( ! rightArmICartesianSolver->fwdKin(currentQ, *currentX) )    {
        printf("[ERROR] Forward Kinematic failed.\n");
        return false;
    }

    return true;
}

/************************************************************************/
bool BodyExecution::getLeftArmFwdKin(std::vector<double> *currentX)
{

    /** ----- Obtain current joint position ----- **/
    int leftArmAxes;
    leftArmIPositionControl2->getAxes(&leftArmAxes);

    if (!leftArmDevice.view(leftArmIEncoders) ) { // connecting our device with "IEncoders" interface
        printf("[warning] Problems acquiring leftArmIEncoders interface\n");
        return false;
    } else printf("[success] Acquired leftArmIEncoders interface\n");

    std::vector<double> currentQ(leftArmAxes);
    if ( ! leftArmIEncoders->getEncoders( currentQ.data() ) ){
        printf("[ERROR] ForgetEncoders failed\n");
        return false;
    }
    else
    {
        printf("[success] Encoder values of left-arm: (");
        for(int i=0; i<leftArmAxes; i++)
            printf("%f ",currentQ[i]);
        printf(")\n ");
    }

    /** ----- Obtain current cartesian position ---------- **/
    if ( ! leftArmICartesianSolver->fwdKin(currentQ, *currentX) )    {
        printf("[ERROR] Forward Kinematic failed.\n");
        return false;
    }

    return true;
}

/************************************************************************/

std::vector<double> BodyExecution::interpolate(double pta, double ptb, int res)
{
    printf("------ Interpolation results ------\n");
    std::vector<double> path(res);
    double factor = (ptb - pta)/res;
    for (int i = 0; i < res; i++)
    {
        path[i] = ((i+1) * factor) + pta;
        printf("%f ", path[i]);
    }
    printf("\n");

    printf("------------------------------------\n");
    getchar();
    return path;
}

/************************************************************************/

/* int axis:
    axis = 0 -> x
    axis = 1 -> y
    axis = 2 -> z
*/
bool BodyExecution::moveTrayLinearly(int axis, double dist)
{
    int status;
    int res = 20; // interpolate resolution
    std::vector<double> rightArmVector;
    std::vector<double> leftArmVector;

    double rightArmPto; // origin point
    double leftArmPto;
    double rightArmPtd; // destination point
    double leftArmPtd;

    // Resulting paths:
    std::vector<double> rightArmPath(res);
    std::vector<double> leftArmPath(res);

    // Showing fordward kinematic
    if(! getRightArmFwdKin(&rightArmVector))
        printf("[ERROR] Doing Forward Kinematic...\n");

    printf("-> FK of right-arm: (");
    for(int i=0; i<rightArmVector.size(); i++)
        printf("%f ",rightArmVector[i]);
    printf(")\n ");

    if(! getLeftArmFwdKin(&leftArmVector))
        printf("[ERROR] Doing Forward Kinematic...\n");

    printf("> FK of left-arm: (");
    for(int i=0; i<leftArmVector.size(); i++)
        printf("%f ",leftArmVector[i]);
    printf(")\n ");

    // axis selection
    rightArmPto = rightArmVector[axis]; // pto origin
    leftArmPto = leftArmVector[axis]; // pto origin

    // Calculating new destination point
    rightArmPtd = rightArmPto + dist; // pto destination = pto origin + dist
    leftArmPtd = leftArmPto + dist; // pto destination = pto origin + dist

    // Interpolation
    printf("> Interpolating right-arm path: \n");
    rightArmPath = interpolate(rightArmPto,rightArmPtd,res);
    printf("> Interpolating left-arm path: \n");
    leftArmPath = interpolate(leftArmPto,leftArmPtd,res);

    // move arms in the cartesian space
    for (int i = 0; i < res; i++)
    {
        printf("-------- Position: %d -------\n", i+1);
        rightArmVector[axis] = rightArmPath[i]; // new point obtained by interpolation
        leftArmVector[axis] = leftArmPath[i]; // new point obtained by interpolation

        printf("* Position of RIGHT-ARMS in cartesian space: (");
        for(int i=0; i<rightArmVector.size(); i++)
            printf("%f ",rightArmVector[i]);
        printf(")\n ");

        printf("* Position of LEFT-ARMS in cartesian space: (");
        for(int i=0; i<leftArmVector.size(); i++)
            printf("%f ",leftArmVector[i]);
        printf(")\n ");

        std::vector<double> rightArmCurrentQ(numRightArmJoints);
        std::vector<double> rightArmDesireQ(numRightArmJoints);
        std::vector<double> leftArmCurrentQ(numLeftArmJoints);
        std::vector<double> leftArmDesireQ(numLeftArmJoints);

        // initialize to 0
        rightArmCurrentQ.clear();
        rightArmDesireQ.clear();
        leftArmCurrentQ.clear();
        leftArmDesireQ.clear();

        // currentQ: pequeña ayuda que le das al solver sobre la posición inicial en posicion articular
        if ( ! rightArmIEncoders->getEncoders( rightArmCurrentQ.data() ) ){
            printf("[ERROR] Failed getEncoders of right-arm\n");
            return false;
        }

        if ( ! rightArmICartesianSolver->invKin(rightArmVector, rightArmCurrentQ, rightArmDesireQ) )    {
            printf("[ERROR] invKin failed.\n");
        }

        printf("-> IK of right-arm: (");
        for(int i=0; i<rightArmDesireQ.size(); i++)
            printf("%f ",rightArmDesireQ[i]);
        printf(")\n ");

        // currentQ: pequeña ayuda que le das al solver sobre la posición inicial en posicion articular
        if ( ! leftArmIEncoders->getEncoders( leftArmCurrentQ.data() ) ){
            printf("[ERROR] Failed getEncoders of left-arm\n");
            return false;
        }

        if ( ! leftArmICartesianSolver->invKin(leftArmVector, leftArmCurrentQ, leftArmDesireQ) )    {
            printf("[ERROR] invKin failed.\n");
        }

        printf("-> IK of left-arm: (");
        for(int i=0; i<leftArmDesireQ.size(); i++)
            printf("%f ",leftArmDesireQ[i]);
        printf(")\n\n ");

        std::vector<double> head(2,0.0);
        std::vector<double> rightArm(&rightArmDesireQ[0], &rightArmDesireQ[0]+7); //teoSim (+6) teo (+7)
        std::vector<double> leftArm(&leftArmDesireQ[0], &leftArmDesireQ[0]+7);

        jointsMoveAndWait(leftArm, rightArm, head, false);

    }

    return true;
}


/************************************************************************/

void BodyExecution::run()
{
    while(true){
        printf("[doing...] Captured movement (1)\n");
        {
            double rightArmPoss[7] = {-22.9173, 2.6362, 31.8101, 15.9050, -22.3022, 22.6713, 0.0};
            double lefArmPoss[7] = {22.9173889160156, -2.63620376586914, -31.8101921081543, -15.9050960540771, 22.3022766113281, -22.6713523864746, 0.0};
            std::vector<double> rightArm(&rightArmPoss[0], &rightArmPoss[0]+7); //teoSim (+6) teo (+7)
            std::vector<double> leftArm(&lefArmPoss[0], &lefArmPoss[0]+7);            
            std::vector<double> head(2,0.0);
            jointsMoveAndWait(leftArm,rightArm,head,true);
        }

        printf("[doing...] Captured movement (2)\n");
        {
            double rightArmPoss[7] = {-22.1265563964844, -5.6942138671875, 64.9384841918945, 15.9050960540771, -59.8242492675781, 22.5834789276123, 0.0};
            double lefArmPoss[7] = {22.1265563964844, 5.6942138671875, -64.9384841918945, -15.9050960540771, 59.8242492675781, -22.5834789276123, 0.0};
            std::vector<double> rightArm(&rightArmPoss[0], &rightArmPoss[0]+7); //teoSim (+6) teo (+7)
            std::vector<double> leftArm(&lefArmPoss[0], &lefArmPoss[0]+7);
            std::vector<double> head(2,0.0);
            jointsMoveAndWait(leftArm,rightArm,head,true);
        }

        printf("[doing...] Captured movement (3)\n");
        {
            double rightArmPoss[7] = {-36.8014221191406, -8.1546630859375, 49.2970123291016, 30.579963684082, -85.1318054199219, 22.6713523864746, 0.0};
            double lefArmPoss[7] = {36.8014221191406, 8.1546630859375, -49.2970123291016, -30.579963684082, 85.1318054199219, -22.6713523864746, 0.0};
            std::vector<double> rightArm(&rightArmPoss[0], &rightArmPoss[0]+7); //teoSim (+6) teo (+7)
            std::vector<double> leftArm(&lefArmPoss[0], &lefArmPoss[0]+7);
            std::vector<double> head(2,0.0);
            jointsMoveAndWait(leftArm,rightArm,head,true);

        }
        printf("[doing...] Captured movement (4)\n");
        {
            double rightArmPoss[7] = {-35.0439453125, -27.3110656738281, 59.8418273925781, 52.8119506835938, -85.1318054199219, 38.4885749816895, 0.0};
            double lefArmPoss[7] = {35.0439453125, 27.3110656738281, -59.8418273925781, -52.8119506835938, 85.1318054199219, -38.4885749816895, 0.0};
            std::vector<double> rightArm(&rightArmPoss[0], &rightArmPoss[0]+7); //teoSim (+6) teo (+7)
            std::vector<double> leftArm(&lefArmPoss[0], &lefArmPoss[0]+7);
            std::vector<double> head(2,0.0);
            jointsMoveAndWait(leftArm,rightArm,head,true);
        }

        printf("[doing...] Captured movement (5)\n");
        {
            double rightArmPoss[7] = {-9.3848876953125, -52.3550109863281, 66.9595794677734, 52.8998222351074, -81.4411315917969, 48.9455184936523, 0.0};
            double lefArmPoss[7] = {9.3848876953125, 52.3550109863281, -66.9595794677734, -52.8998222351074, 81.4411315917969, -48.9455184936523, 0.0};
            std::vector<double> rightArm(&rightArmPoss[0], &rightArmPoss[0]+7); //teoSim (+6) teo (+7)
            std::vector<double> leftArm(&lefArmPoss[0], &lefArmPoss[0]+7);
            std::vector<double> head(2,0.0);
            jointsMoveAndWait(leftArm,rightArm,head,true);
        }

        printf("[doing...] Captured movement (6)\n");
        {
            double rightArmPoss[7] = {30.579963684082, -52.1792602539062, 50.2636184692383, 52.8119506835938, -81.5289916992188, 64.8506164550781, 0.0};
            double lefArmPoss[7] = {-30.579963684082, 52.1792602539062, -50.2636184692383, -52.8119506835938, 81.5289916992188, -64.8506164550781, 0.0};
            std::vector<double> rightArm(&rightArmPoss[0], &rightArmPoss[0]+7); //teoSim (+6) teo (+7)
            std::vector<double> leftArm(&lefArmPoss[0], &lefArmPoss[0]+7);
            std::vector<double> head(2,0.0);
            jointsMoveAndWait(leftArm,rightArm,head,true);
        }

        printf("[doing...] Captured movement (7)\n");
        {
            double rightArmPoss[7] = {32.1616859436035, -44.1827697753906, 22.0562381744385, 52.7240753173828, -81.5289916992188, 87.5219650268555, 1200.0};
            double lefArmPoss[7] = {-32.1616859436035, 44.1827697753906, -22.0562381744385, -52.7240753173828, 81.5289916992188, -87.5219650268555, 1200.0};
            std::vector<double> rightArm(&rightArmPoss[0], &rightArmPoss[0]+7); //teoSim (+6) teo (+7)
            std::vector<double> leftArm(&lefArmPoss[0], &lefArmPoss[0]+7);
            std::vector<double> head(2,0.0);
            jointsMoveAndWait(leftArm,rightArm,head,true);
        }

        printf("[doing...] Captured movement (8)\n");
        {
            double rightArmPoss[7] = {32.1616859436035, -29.5079040527344, 4.04217910766602, 52.8119506835938, -81.5289916992188, 93.0579986572266, 1200.0};
            double lefArmPoss[7] = {-32.1616859436035, 29.5079040527344, -4.04217910766602, -52.8119506835938, 81.5289916992188, -93.0579986572266, 1200.0};
            std::vector<double> rightArm(&rightArmPoss[0], &rightArmPoss[0]+7); //teoSim (+6) teo (+7)
            std::vector<double> leftArm(&lefArmPoss[0], &lefArmPoss[0]+7);
            std::vector<double> head(2,0.0);
            jointsMoveAndWait(leftArm,rightArm,head,true);
        }


        printf("[doing...] Captured movement (9)\n");
        {
            double rightArmPoss[7] = {33.4797897338867, -19.8418273925781, -5.25482177734375, 52.7240753173828, -81.5289916992188, 98.5940246582031, 1200.0};
            double lefArmPoss[7] = {-33.4797897338867, 19.8418273925781, 5.25482177734375, -52.7240753173828, 81.5289916992188, -98.5940246582031, 1200.0};
            std::vector<double> rightArm(&rightArmPoss[0], &rightArmPoss[0]+7); //teoSim (+6) teo (+7)
            std::vector<double> leftArm(&lefArmPoss[0], &lefArmPoss[0]+7);
            std::vector<double> head(2,0.0);
            jointsMoveAndWait(leftArm,rightArm,head,true);
        }

        printf("[doing...] Captured movement (10)\n");
        {
            double rightArmPoss[7] = {31.2829513549805, -14.0421752929688, -10.7908630371094, 52.6362037658691, -81.5289916992188, 88.4885787963867, 1200.0};
            double lefArmPoss[7] = {-31.2829513549805, 14.0421752929688, 10.7908630371094, -52.6362037658691, 81.5289916992188, -88.4885787963867, 1200.0};
            std::vector<double> rightArm(&rightArmPoss[0], &rightArmPoss[0]+7); //teoSim (+6) teo (+7)
            std::vector<double> leftArm(&lefArmPoss[0], &lefArmPoss[0]+7);
            std::vector<double> head(2,0.0);
            jointsMoveAndWait(leftArm,rightArm,head,true);
        }

        printf("[doing...] Closing hands\n");
        {
            double rightArmPoss[7] = {31.2829, -14.0421, -10.7908, 52.6362, -81.5289, 88.4885, -1200.0};
            double lefArmPoss[7] = {-31.2829, 14.0421, 10.7908, -52.6362, 81.5289, -88.4885, -1200.0};
            std::vector<double> rightArm(&rightArmPoss[0], &rightArmPoss[0]+7); //teoSim (+6) teo (+7)
            std::vector<double> leftArm(&lefArmPoss[0], &lefArmPoss[0]+7);
            std::vector<double> head(2,0.0);
            jointsMoveAndWait(leftArm,rightArm,head,true);
        }
        printf("Cartesian space movement...(press a key to continue)\n");
        {
            printf("waiting to do inward movement (2cm)...\n");
            moveTrayLinearly(0, -0.02); // hacia adentro
            printf("waiting to do right movement (4cm)...\n");
            getchar();
            moveTrayLinearly(1, -0.04); // hacia derecha
            printf("waiting to move to the center (4cm)...\n");
            getchar();
            moveTrayLinearly(1, 0.04); // hacia centro
            printf("waiting to do left movement (4cm)... ");
            getchar();
            moveTrayLinearly(1, 0.04); // hacia izquierda
            printf("waiting to move to the center (4cm)...\n");
            getchar();
            moveTrayLinearly(1, -0.04); // hacia centro
            printf("waiting to move up (4cm)...\n");
            getchar();
            moveTrayLinearly(2, 0.04); // hacia arriba
            printf("waiting to move down (4cm)...\n");
            getchar();
            moveTrayLinearly(2, -0.04); // hacia abajo
            getchar();
        }

    }
}

/************************************************************************/

}  // namespace teo

