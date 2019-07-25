// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "BalanceTray.hpp"

#include <algorithm>

namespace teo
{

/************************************************************************/

bool BalanceTray::configure(yarp::os::ResourceFinder &rf)
{   
    robot = rf.check("robot",yarp::os::Value(DEFAULT_ROBOT),"name of /robot to be used").asString();
    mode = rf.check("mode",yarp::os::Value(DEFAULT_MODE),"the /mode to be used").asString();
    rdsxaa.resize(7);
    ldsxaa.resize(7);

    printf("--------------------------------------------------------------\n");
    if (rf.check("help"))
    {
        printf("BalanceTray options:\n");        
        printf("\t--help (this help)\t--from [file.ini]\t--context [path]\n");
        printf("\t--robot: %s [%s]\n",robot.c_str(),DEFAULT_ROBOT);
        printf("\t--mode: %s [%s] (select: jr3Balance / keyboard / jr3ToCsv )\n", mode.c_str(), DEFAULT_MODE);
        printf("\t--speak \n""");
        ::exit(0);
    }

    // operation modes
    balanceJr3 = false;
    keyboard   = false;
    jr3ToCsv    = false;

    // speak sentences
    speak = false;

    // initialice variables
    rightArmTrajThread = 0;
    leftArmTrajThread = 0;
    fp = 0;

    if(mode == "jr3Balance")
    {
        CD_INFO_NO_HEADER("Mode JR3 with balance tray [activated]\n");
        balanceJr3 = true;
    }
    else if(mode == "keyboard")
    {
        CD_INFO_NO_HEADER("Mode KEYBOARD [activated]\n");
        keyboard = true;
    }
    else if(mode == "jr3ToCsv")
    {
        CD_INFO_NO_HEADER("Mode test positions and JR3 values to CSV file [activated]\n");
        jr3ToCsv = true;
    }
    else {
        CD_ERROR_NO_HEADER("no mode recognised to work\n");
        return false;
    }

    if(rf.check("speak"))
    {
        CD_INFO_NO_HEADER("Mode SPEAK [activated]\n");
        speak = true;
    }
    else CD_INFO_NO_HEADER("Mode SPEAK [deactivated]\n");


    std::string balanceTrayStr("/balanceTray");

    // ------ ANALOG SENSOR ------   
    if(balanceJr3 || jr3ToCsv){
        yarp::os::Property options;
            options.put("device","Jr3");

        if(!jr3card.open(options)) {
              std::printf("Device not available.\n");
              jr3card.close();
              yarp::os::Network::fini();
              return 1;
        }


        if (!jr3card.view(iAnalogSensor) )
            {
                CD_ERROR("Problems acquiring JR3 interface\n");
                return 1;
            }

        CD_SUCCESS("Acquired JR3 interface [ok]\n");
    }

    // ------ RIGHT ARM -------

    yarp::os::Property rightArmOptions;
    rightArmOptions.put("device","remote_controlboard");
    rightArmOptions.put("remote","/"+robot+"/rightArm");
    rightArmOptions.put("local",balanceTrayStr+"/"+robot+"/rightArm");
    rightArmDevice.open(rightArmOptions);
    if(!rightArmDevice.isValid()) {
      CD_ERROR("robot rightArm device not available.\n");
      rightArmDevice.close();
      yarp::os::Network::fini();
      return false;
    }    

    // connecting our device with "IEncoders" interface
    if (!rightArmDevice.view(rightArmIEncoders) ) {
        CD_ERROR("Problems acquiring rightArmIEncoders interface\n");
        return false;
    }
    else
    {
        CD_INFO("Acquired leftArmIEncoders interface\n");
        if(!rightArmIEncoders->getAxes(&numRightArmJoints))
            CD_ERROR("Problems acquiring numRightArmJoints\n");
    }

    // connecting our device with "control mode" interface, initializing which control mode we want (position)
    if (!rightArmDevice.view(rightArmIControlMode) ) {
        CD_ERROR("Problems acquiring rightArmIControlMode interface\n");
        return false;
    } else
        CD_INFO("Acquired rightArmIControlMode interface\n");    

    // connecting our device with "PositionControl" interface
    // connecting our device with "position control" interface (configuring our device: speed, acceleration... and sending joint positions)
    if (!rightArmDevice.view(rightArmIPositionControl) ) {
        CD_ERROR("Problems acquiring rightArmIPositionControl interface\n");
        return false;
    } else
        CD_INFO("Acquired rightArmIPositionControl interface\n");

    // connecting our device with "PositionDirect" interface
    if (!rightArmDevice.view(rightArmIPositionDirect) ) {
        CD_ERROR("Problems acquiring rightArmIPositionDirect interface\n");
        return false;
    } else
        CD_INFO("Acquired rightArmIPositionDirect interface\n");

    // -- connecting our device with "RemoteVariables" interface
    /*    This interface is used to test external reference mode (but it's not used)
    if (!rightArmDevice.view(rightArmIRemoteVariables) ) {
        CD_ERROR("Problems acquiring rightArmIRemoteVariables interface\n");
        return false;
    } else
        CD_INFO("Acquired rightArmIRemoteVariables interface\n");
    */

    // ------ LEFT ARM -------

    yarp::os::Property leftArmOptions;
    leftArmOptions.put("device","remote_controlboard");
    leftArmOptions.put("remote","/"+robot+"/leftArm");
    leftArmOptions.put("local",balanceTrayStr+"/"+robot+"/leftArm");
    leftArmDevice.open(leftArmOptions);
    if(!leftArmDevice.isValid()) {
      CD_ERROR("robot leftArm device not available.\n");
      leftArmDevice.close();
      yarp::os::Network::fini();
      return false;
    }

    // connecting our device with "IEncoders" interface   
    if (!leftArmDevice.view(leftArmIEncoders) ) {
        CD_ERROR("Problems acquiring leftArmIEncoders interface\n");
        return false;
    } else {
        CD_INFO("Acquired leftArmIEncoders interface\n");
        if(!leftArmIEncoders->getAxes(&numLeftArmJoints))
            CD_ERROR("Problems acquiring numLeftArmJoints\n");
    }

    // connecting our device with "control mode" interface, initializing which control mode we want (position)
    if (!leftArmDevice.view(leftArmIControlMode) ) {
        CD_ERROR("Problems acquiring leftArmIControlMode interface\n");
        return false;
    } else
        CD_INFO("Acquired leftArmIControlMode interface\n");

    // connecting our device with "position control" interface (configuring our device: speed, acceleration... and sending joint positions)
    if (!leftArmDevice.view(leftArmIPositionControl) ) {
        CD_ERROR("Problems acquiring leftArmIPositionControl interface\n");
        return false;
    } else
        CD_INFO("Acquired leftArmIPositionControl interface\n");

    // -- connecting our device with "PositionDirect" interface
    if (!leftArmDevice.view(leftArmIPositionDirect) ) {
        CD_ERROR("Problems acquiring leftArmIPositionDirect interface\n");
        return false;
    } else
        CD_INFO("Acquired leftArmIPositionDirect interface\n");

    // -- connecting our device with "RemoteVariables" interface
    /*  This interface is used to test external reference mode (but it's not used for now)
    if (!leftArmDevice.view(leftArmIRemoteVariables) ) {
        CD_ERROR("Problems acquiring leftArmIRemoteVariables interface\n");
        return false;
    } else
        CD_INFO("Acquired leftArmIRemoteVariables interface\n");

    */

    // ----- Configuring KDL Solver for right-arm -----

    if( ! rightArmDevice.view(rightArmIControlLimits) ) {
        CD_ERROR("Could not view iControlLimits in rightArmDevice\n");
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
            CD_INFO_NO_HEADER("Joint %d limits: [%f,%f]\n",joint,min,max);
        }

    yarp::os::Property rightArmSolverOptions;
    std::string rightKinPath = rf.findFileByName("fixedTrunk-rightArm-fetch-kinematics.ini");;
    rightArmSolverOptions.fromConfigFile(rightKinPath);
    rightArmSolverOptions.put("device","KdlSolver");
    //rightArmSolverOptions.put("maxIter", 1000); // iterator configuration
    //rightArmSolverOptions.put("eps", 1e-9); // precision
    rightArmSolverOptions.put("mins", yarp::os::Value::makeList(qrMin.toString().c_str()));
    rightArmSolverOptions.put("maxs", yarp::os::Value::makeList(qrMax.toString().c_str()));
    //rightArmSolverOptions.put("HN", yarp::os::Value::makeList("-0.00984635  0.9989744  -0.04419502  0.23194013 -0.00484966 -0.04424435 -0.99900897 -0.01027258 -0.99993976 -0.00962226  0.00528033 -0.00223408 0 0 0 1"));
    rightArmSolverDevice.open(rightArmSolverOptions);

    if( ! rightArmSolverDevice.isValid() )
    {
        CD_ERROR("KDLSolver solver device for right-arm is not valid \n");
        return false;
    }

    if( ! rightArmSolverDevice.view(rightArmICartesianSolver) )
    {
        CD_ERROR("Could not view iCartesianSolver in KDLSolver \n");
        return false;
    } else
        CD_SUCCESS("Acquired rightArmICartesianSolver interface\n");


    // ----- Configuring KDL Solver for left-arm -----

    if( ! leftArmDevice.view(leftArmIControlLimits) ) {
        CD_ERROR("Could not view iControlLimits in leftArmDevice\n");
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
            CD_INFO_NO_HEADER("Joint %d limits: [%f,%f]\n",joint,min,max);
        }

    yarp::os::Property leftArmSolverOptions;    
    std::string leftKinPath = rf.findFileByName("fixedTrunk-leftArm-fetch-kinematics.ini");;
    leftArmSolverOptions.fromConfigFile(leftKinPath);
    leftArmSolverOptions.put("device", "KdlSolver");
    leftArmSolverOptions.put("mins", yarp::os::Value::makeList(qlMin.toString().c_str()));
    leftArmSolverOptions.put("maxs", yarp::os::Value::makeList(qlMax.toString().c_str()));
    //leftArmSolverOptions.put("ik", "st"); // to use screw theory IK
    //leftArmSolverOptions.put("HN", yarp::os::Value::makeList("-0.00984635  -0.9989744  -0.04419502  0.23194013 0.00484966 -0.04424435 0.99900897 0.01027258 -0.99993976 0.00962226  0.00528033 -0.00223408 0 0 0 1"));
    leftArmSolverDevice.open(leftArmSolverOptions);

    if( ! leftArmSolverDevice.isValid() )
    {
        CD_ERROR("KDLSolver solver device for left-arm is not valid \n");
        return false;
    } else
        CD_SUCCESS("Acquired rightArmICartesianSolver interface\n");

    if( ! leftArmSolverDevice.view(leftArmICartesianSolver) )
    {
        CD_ERROR("Could not view iCartesianSolver in KDLSolver\n");
        return false;
    } else
        CD_SUCCESS("Acquired leftArmICartesianSolver interface\n");


    // ** Unify TCP of right-arm and left-arm: apppending link to the tray's centroid
    /** Transformation matrix between the gripper and the tray **/
    // -- teoSim:     
    double rightArmTeoSim[] = {1.73545231e-03, -3.80284497e-04, 2.28085790e-01, 1.57220542e+00, 5.08799928e-02, 3.90038185e-02};
    double leftArmTeoSim[]  = {1.73545231e-03,  3.80284497e-04, 2.28085790e-01,-1.57220542e+00, 5.08799928e-02,-3.90038185e-02};
    // -- teo    
    double rightArmTeoRobot[] = {-3.47946146e-03, 1.80942075e-04, 2.28993589e-01, 1.56922560e+00, 2.77594895e-02, 5.15844922e-02}; //right
    double leftArmTeoRobot[]  = {-0.00332034, -0.00249526,  0.22617789, -1.55904518,  0.02679647, -0.04939248}; //left

    std::vector<double> twist_right_N_T;
    std::vector<double> twist_left_N_T;

    if(robot=="teoSim"){
        for(int i; i<6; i++)
            twist_right_N_T.push_back(rightArmTeoSim[i]);
        for(int i; i<6; i++)
            twist_left_N_T.push_back(leftArmTeoSim[i]);
        CD_SUCCESS("\n");
    }
    else if(robot=="teo")
    {
        for(int i; i<6; i++)
            twist_right_N_T.push_back(rightArmTeoRobot[i]);
        for(int i; i<6; i++)
            twist_left_N_T.push_back(leftArmTeoRobot[i]);
        CD_SUCCESS("\n");
    }
    else
    {
        CD_ERROR("\n");
        return false;
    }

    rightArmICartesianSolver->appendLink(twist_right_N_T);
    leftArmICartesianSolver->appendLink(twist_left_N_T);

    // ----- Configuring Speech -----
    if(speak){
        dialogueManager = new DialogueManager("spanish");
        dialogueManager->ttsSay("Demostracion de bandeja iniciada");
    }

    // Start operations:
    if(homePosition())
    {
        CD_SUCCESS("Home position [OK]\n");
        printFKinAA();
    }

    if (speak) dialogueManager->ttsSay("Por favor, coloca la bandeja en mis manos, y cuando este lista, pulsa cualquier tecla para comenzar");

    CD_INFO_NO_HEADER("Put the tray and press a Key...\n");
    getchar();

    // Calibrate sensor ... JR3
    if(balanceJr3 || jr3ToCsv){
        CD_INFO("\n");
        int ret = iAnalogSensor->calibrateSensor();        
        if(ret!=0){
            CD_ERROR("Calibrating sensors...\n");
            return false;
        }
        else CD_SUCCESS("JR3 sensors calibrated\n");
        CD_INFO("\n");
    }

    if (speak) dialogueManager->ttsSay("un momento por favor");

    if(configArmsToPositionDirect()) {
        CD_SUCCESS("Configured to Position Direct\n");
    }
    else {
        CD_ERROR("Configuring drivers to Position Direct\n");
        return false;
    }

    if (speak) dialogueManager->ttsSay("Sensores de fuerza par calibrados. Que comience el juego");

    // start reading and sending information thread: JR3/keyboard (10ms) -> BalanceThread (50ms)
    this->start();

    if(jr3ToCsv) // we are going to create a offline saved trajectories to test the jr3 sensors
    {       
       fp = fopen("../data.csv","w+");
       fprintf(fp, "time, axis_x, axis_y, axis_z, rotation angle, right_jr3_+y, left_jr3_-y\n");
       double increment = PI/180; // 1 degree = 0,0174533 rad

       for(int i=1; i<=5; i++){ // menos de 5º
            //CD_INFO_NO_HEADER("Rotate tray: (%f) rad / (%i) degrees\n", increment*i, i); // max value: increment*i = 0.1
            // axis, angle, duration, maxvel
            rotateTrayByTrajectory(0,increment,3,10);
            yarp::os::Time::delay(0.5);
       }

       for(int i=1; i<=10; i++){ // menos de 5º
            //CD_INFO_NO_HEADER("Rotate tray: (%f) rad / (%i) degrees\n", increment*i, i); // max value: increment*i = 0.1
            // axis, angle, duration, maxvel
            rotateTrayByTrajectory(0,-increment,3,10);
            yarp::os::Time::delay(0.5);
       }

       for(int i=1; i<=5; i++){ // menos de 5º
            //CD_INFO_NO_HEADER("Rotate tray: (%f) rad / (%i) degrees\n", increment*i, i); // max value: increment*i = 0.1
            // axis, angle, duration, maxvel
            rotateTrayByTrajectory(0,increment,3,10);
            yarp::os::Time::delay(0.5);
       }


    }

    else
    {

        // Initialice threads of arms
        rightArmBalThread = new BalanceThread(rightArmIEncoders, rightArmICartesianSolver, rightArmIPositionDirect, PT_MODE_MS );
        leftArmBalThread = new BalanceThread(leftArmIEncoders, leftArmICartesianSolver, leftArmIPositionDirect, PT_MODE_MS );

        // start BalanceThread
        rightArmBalThread->start();
        leftArmBalThread->start();
    }


    return true;
}

/************************************************************************/

bool BalanceTray::interruptModule()
{
    this->stop(); // stop JR3/keyboard reading thread
    rightArmBalThread->stop(); // stop BalanceThread of rightArm
    leftArmBalThread->stop();  // stop BalanceThread of leftArm
    rightArmDevice.close();
    leftArmDevice.close();
    return true;
}

/************************************************************************/

double BalanceTray::getPeriod()
{
    return 15.0; // Fixed, in seconds, the slow thread that calls updateModule below
}

/************************************************************************/

bool BalanceTray::updateModule()
{
   if (speak) dialogueManager->talkTrayStatus(sensorValues,rdsxaa,ldsxaa);
   return true;
}

/************************************************************************/

bool BalanceTray::threadInit(){
    initTime = Time::now();
    sensorValues.zero();
    return true;
}

/**************** Sending Thread *************************/

void BalanceTray::run()
{    
    std::vector<double> rdx, ldx;

    // sensor reading
        if(balanceJr3 || jr3ToCsv){
            int ret = iAnalogSensor->read(sensorValues);
            if(ret!=yarp::dev::IAnalogSensor::AS_OK)
            {
                CD_ERROR("Reading JR3\n");
                return;
            }
            else
                if(balanceJr3)
                {
                    // reading JR3 sensor
                    printJr3(sensorValues);

                    if(!calculatePointOpposedToForce(sensorValues, &rdx, &ldx)){
                        CD_ERROR("Calculating next point\n");
                        return;
                    }
                }
            else
                if(jr3ToCsv)
                {
                    std::vector<double> axisRotation;
                    if(!getAxisRotation(&axisRotation))
                    {
                        CD_ERROR("Getting tray rotation\n");
                        return;
                    }
                    writeInfo2Csv(Time::now()-initTime, axisRotation, sensorValues);
                    return;
                }

        }

    // reading keyboard
        else
        {
            // if the iteration of the thread is the first, copy the position.
            // this fixs the warning messages: "KdlVectorConverter.cpp:13 vectorToFrame(): Size mismatch; expected: 6, was: 0"
            if( this->getIterations()!=0 )
                calculatePointPressingKeyboard(&rdx, &ldx);
            else getRefPosition(&rdx,&ldx);
        }

    rightArmBalThread->setCartesianPosition(rdx);
    leftArmBalThread->setCartesianPosition(ldx);
}

/****************************************************/

bool BalanceTray::getRightArmFwdKin(std::vector<double> *currentX)
{
    /** ----- Obtain current joint position ----- **/    
    std::vector<double> currentQ(numRightArmJoints);
    if ( ! rightArmIEncoders->getEncoders( currentQ.data() ) ){
        CD_ERROR("ForgetEncoders failed\n");
        return false;
    }


    /** ----- Obtain current cartesian position ---------- **/
    if ( ! rightArmICartesianSolver->fwdKin(currentQ, *currentX) )    {
        CD_ERROR("Forward Kinematic failed.\n");
        return false;
    }

    return true;
}

bool BalanceTray::getLeftArmFwdKin(std::vector<double> *currentX)
{
    /** ----- Obtain current joint position ----- **/
    std::vector<double> currentQ(numLeftArmJoints);
    if ( ! leftArmIEncoders->getEncoders( currentQ.data() ) ){
        CD_ERROR("getEncoders failed\n");
        return false;
    }

    /** ----- Obtain current cartesian position ---------- **/
    if ( ! leftArmICartesianSolver->fwdKin(currentQ, *currentX) )    {
        CD_ERROR("Forward Kinematic failed.\n");
        return false;
    }

    return true;
}

/************ CONFIGURATION MODES FOR DEVICES *******/

bool BalanceTray::configArmsToPosition(double sp, double acc){

    // -- Speed and acceleration for 7 joints
    std::vector<double> armSpeeds(7, sp); // 7,30.0
    std::vector<double> armAccelerations(7, acc); // 7,30.0

    // -- Configuring Devices to Position Mode
    std::vector<int> rightArmControlModes(numRightArmJoints,VOCAB_CM_POSITION);
    if(! rightArmIControlMode->setControlModes(rightArmControlModes.data())){
        CD_ERROR("Problems setting position control mode of: right-arm\n");
        return false;
    }


    std::vector<int> leftArmControlModes(numLeftArmJoints,VOCAB_CM_POSITION);
    if(! leftArmIControlMode->setControlModes( leftArmControlModes.data() )){
        CD_ERROR("Problems setting position control mode of: left-arm\n");
        return false;
    }

    // -- Configuring speed and acceleration
    if(!rightArmIPositionControl->setRefSpeeds(armSpeeds.data())){
        CD_ERROR("Problems setting reference speed on right-arm joints.\n");
        return false;
    }
    if(!leftArmIPositionControl->setRefSpeeds(armSpeeds.data())){
        CD_ERROR("Problems setting reference speed on left-arm joints.\n");
        return false;
    }
    if(!rightArmIPositionControl->setRefAccelerations(armAccelerations.data())){
        CD_ERROR("Problems setting reference acceleration on right-arm joints.\n");
        return false;
    }
    if(!leftArmIPositionControl->setRefAccelerations(armAccelerations.data())){
        CD_ERROR("Problems setting reference acceleration on left-arm joints.\n");
        return false;
    }

    return true;
}

bool BalanceTray::configArmsToPositionDirect(){

        CD_INFO("Configuring drivers to Position Direct...\n");
        /* This is used with external reference implementation in yarp-devices
         * This APP is working well with old-pt mode
        if(robot=="teo"){ // if we are using the robot, better use PT Mode
            yarp::os::Bottle val;
            yarp::os::Bottle & b = val.addList();
            b.addInt32(PT_MODE_MS);

            if(! rightArmIRemoteVariables->setRemoteVariable("ptModeMs", val)){
                CD_ERROR("Problems setting ptModeMs remote variable of: right-arm\n");
                return false;
            }

            if(! leftArmIRemoteVariables->setRemoteVariable("ptModeMs", val)){
                CD_ERROR("Problems setting ptModeMs remote variable of: left-arm\n");
                return false;
            }
        }
        */
        // if there is no ptModeMs (=0), it's activate the external reference
        std::vector<int> rightArmControlModes(numRightArmJoints,VOCAB_CM_POSITION_DIRECT);
        if(! rightArmIControlMode->setControlModes(rightArmControlModes.data())){
            CD_ERROR("Problems setting POSITION DIRECT mode of: right-arm\n");
            return false;
        }

        std::vector<int> leftArmControlModes(numLeftArmJoints,VOCAB_CM_POSITION_DIRECT);
        if(! leftArmIControlMode->setControlModes(leftArmControlModes.data())){
            CD_ERROR("Problems setting POSITION DIRECT mode of: left-arm\n");
            return false;
        }

        return true;
}

/************ MODES TO MOVE THE JOINTS **************/

bool BalanceTray::moveJointsInPosition(std::vector<double> &rightArm, std::vector<double>& leftArm)
{
    // -- checking movement done...
    bool doneRight = false;
    bool doneLeft = false;

    // -- move to position
    if(!rightArmIPositionControl->positionMove( rightArm.data() )){
        printf("[Error: positionMove] Problems setting new reference point for right-arm axes.\n");
        return false;
    }
    if(!leftArmIPositionControl->positionMove( leftArm.data() )){
            printf("[Error: positionMove] Problems setting new reference point for left-arm axes.\n");
            return false;
    }


    while(!doneRight)
    {
        yarp::os::Time::delay(0.1);
        rightArmIPositionControl->checkMotionDone(&doneRight);
        CD_DEBUG("!doneRight\n");
    }

    while(!doneLeft)
    {
        yarp::os::Time::delay(0.1);
        leftArmIPositionControl->checkMotionDone(&doneLeft);
        CD_DEBUG("!doneLeft\n");
    }


    return true;
}

bool BalanceTray::moveJointsInPositionDirect(std::vector<double> &rightArm, std::vector<double> &leftArm){
    if(!rightArmIPositionDirect->setPositions(rightArm.data())){
        CD_ERROR("Problems setting new reference point for right-arm axes.\n");
        return false;
    }

    if(!leftArmIPositionDirect->setPositions(leftArm.data())){
        CD_ERROR("Problems setting new reference point for left-arm axes.\n");
        return false;
    }
    return true;
}

/************ EXECUTE TRAJECTORY ********************/
// Offline trajectories : This function are not used by the moment.
bool BalanceTray::executeTrajectory(std::vector<double> rx, std::vector<double> lx, std::vector<double> rxd, std::vector<double> lxd, double duration, double maxvel)
{
    // trajectory for right-arm
    KdlTrajectory rightArmTraj;
    rightArmTraj.setDuration(duration);
    rightArmTraj.setMaxVelocity(maxvel);
    rightArmTraj.addWaypoint(rx);
    rightArmTraj.addWaypoint(rxd);
    rightArmTraj.configurePath(ICartesianTrajectory::LINE);
    rightArmTraj.configureVelocityProfile(ICartesianTrajectory::RECTANGULAR);

    if (!rightArmTraj.create())
    {
        CD_ERROR("Problem creating cartesian trajectory of rightArm.\n");
        return false;
    }

    // trajectory for left-arm
    KdlTrajectory leftArmTraj;
    leftArmTraj.setDuration(duration);
    leftArmTraj.setMaxVelocity(maxvel);
    leftArmTraj.addWaypoint(lx);
    leftArmTraj.addWaypoint(lxd);
    leftArmTraj.configurePath(ICartesianTrajectory::LINE);
    leftArmTraj.configureVelocityProfile(ICartesianTrajectory::RECTANGULAR);

    if (!leftArmTraj.create())
    {
        CD_ERROR("Problem creating cartesian trajectory of leftArm.\n");
        return false;
    }

    if (rightArmTrajThread == 0)
        rightArmTrajThread = new TrajectoryThread(rightArmIEncoders, rightArmICartesianSolver, rightArmIPositionDirect, PT_MODE_MS );


    if (leftArmTrajThread == 0)
        leftArmTrajThread = new TrajectoryThread(leftArmIEncoders, leftArmICartesianSolver, leftArmIPositionDirect, PT_MODE_MS );

    rightArmTrajThread->setICartesianTrajectory(&rightArmTraj);
    leftArmTrajThread->setICartesianTrajectory(&leftArmTraj);

    if (rightArmTrajThread->isSuspended() && leftArmTrajThread->isSuspended())
    {
        rightArmTrajThread->resetTime();
        leftArmTrajThread->resetTime();
        rightArmTrajThread->resume();
        leftArmTrajThread->resume();
    }
    else
    {
        rightArmTrajThread->start();
        leftArmTrajThread->start();
    }

    yarp::os::Time::delay(duration);
    rightArmTrajThread->suspend();
    leftArmTrajThread->suspend();

    return true;
}

bool BalanceTray::rotateTrayByTrajectory(int axis, double angle, double duration, double maxvel){

    // first check
    if(axis<0 && axis>2){
        CD_ERROR("\n");
        return false;
    }

    std::vector<double> rx, rdx;
    std::vector<double> lx, ldx;

    if(!getRefPosition(&rx, &lx)){
        CD_ERROR("Getting last position\n");
        return false;
    }

    rdx = rx;
    ldx = lx;

    rdx[axis+3] = rdx[axis+3] + angle;
    ldx[axis+3] = ldx[axis+3] + angle;

    if(setRefPosition(rdx, ldx))
        CD_SUCCESS("Saved reference position\n");    

    if(!executeTrajectory(rx, lx, rdx, ldx, duration, maxvel)){
        CD_ERROR("Doing trajectory\n");
        return false;
    }

    return true;
}

// Online trajectories: Function used to calculate the next point in relation to the force exerted on the sensors (movement opposite to the force to balance the tray)
bool BalanceTray::calculatePointOpposedToForce(yarp::sig::Vector sensor, std::vector<double> *rdx, std::vector<double> *ldx){
    char plane ='0';
    double increment = 0;
    std::vector<double> rx, rdsx; // right current point, right destination point
    std::vector<double> lx, ldsx; // left current point, left destination point

    if(!getRefPosition(&rx, &lx)){
        CD_ERROR("Getting last position\n");
        return false;
    }

    // calculating position increment
    // -- Turning X axis (negative value)
    //    for the first condition, the force of the object on the RIGHT sensor must be greater than 0.06 and must be greater than the absolute value of the opposite side of the tray.

    if(sensor[13] > 0.06 && std::abs(sensor[13])>std::abs(sensor[19]) ){
        CD_WARNING_NO_HEADER("PRESURE DETECTED RIGHT: [%f]>[%f]\n", sensor[13], sensor[19]);
        increment=-0.00014*std::abs(sensor[13]); // increment value of the distance between points
        plane = 'x';
    }

    // -- Turning X axis (positive value)
    //    for the second condition, the force of the object on the LEFT sensor must be less than -0.6 and must be greater than the absolute value of the opposite side of the tray.

    else if(sensor[19] < -0.06 && std::abs(sensor[19])>std::abs(sensor[13]) ){
        CD_WARNING_NO_HEADER("PRESURE DETECTED LEFT: [%f]<[%f]\n", sensor[13], sensor[19]);
        increment=0.00014*std::abs(sensor[19]); // increment value of the distance between points
        plane = 'x';
    }

    // -- Turning Y axis (positive value)
    //    for the first condition, the torsional force of the object on the RIGHT sensor and LEFT sensor must be less than -0.08 and greater than 0.08 respectively
    if((sensor[17] < -0.07) || (sensor[23] > +0.07))
    {
        CD_WARNING_NO_HEADER("PRESURE DETECTED DOWN: [%f][%f]\n", sensor[17], sensor[23]);
        increment=0.0005*(std::abs(sensor[17])+std::abs(sensor[23]));// increment value of the distance between points
        plane = 'y';
    }

    // -- Turning Y axis (negative value)
    //    for the second condition, the torsional force of the object on the RIGHT sensor and LEFT sensor must be greater than 0.08 and less than -0.08 respectively
    else if((sensor[17] > 0.07) || (sensor[23] < -0.07))
    {
        CD_WARNING_NO_HEADER("PRESURE DETECTED UP: [%f][%f]\n", sensor[17], sensor[23]);
        increment=-0.0005*(std::abs(sensor[17])+std::abs(sensor[23])); // increment value of the distance between points
        plane = 'y';

    }

    // copy current to destination
    rdsx = rx;
    ldsx = lx;

    switch (plane) {
        case 'x':
            // increment rotation value in X axis
            rdsx[3] += increment;            
            ldsx[3] += increment;
            CD_WARNING_NO_HEADER("turning value of X: F[right: %f] F[left: %f]\n", rdsx[3], ldsx[3]);
            break;
        case 'y':
            // increment rotation value in Y axis
            rdsx[4] += increment;
            ldsx[4] += increment;
            CD_WARNING_NO_HEADER("turning value of Y: M[right %f] M[left %f]\n", rdsx[4], ldsx[4]);
            break;

        /* Not used by the moment
        case 'z':
            // increment rotation value in Z axis
            rdsx[5] = rdsx[5] + increment;
            ldsx[5] = ldsx[5] + increment;
            break;
        */
        case '0':
            CD_INFO("Repose position\n");
            break;
    }

    // transformation: Axis Angle Scaled -> Axis Angle
    //std::vector<double> rdsxaa, ldsxaa; -> global
    KinRepresentation::decodePose(rdsx, rdsxaa, KinRepresentation::CARTESIAN, KinRepresentation::AXIS_ANGLE, KinRepresentation::DEGREES );
    KinRepresentation::decodePose(ldsx, ldsxaa, KinRepresentation::CARTESIAN, KinRepresentation::AXIS_ANGLE, KinRepresentation::DEGREES );

    // Checks the joint limits!
    CD_DEBUG_NO_HEADER("R-POSS: [");
    for(int i=0; i<rdsxaa.size(); i++){
        CD_DEBUG_NO_HEADER("%f ",rdsxaa[i]);
    }
    CD_DEBUG_NO_HEADER("]\n");

    CD_DEBUG_NO_HEADER("L-POSS: [");
    for(int i=0; i<ldsxaa.size(); i++){
        CD_DEBUG_NO_HEADER("%f ",ldsxaa[i]);
    }
    CD_DEBUG_NO_HEADER("]\n");

    if(rdsxaa[6]>5 || ldsxaa[6]>5){
        CD_WARNING("Turning STOP (> 5º)!!\n");
        return false;
    }

    if(!setRefPosition(rdsx, ldsx)){
        CD_ERROR("Saving reference position\n");
        return false;
    }

    // send to the pointer
    *rdx = rdsx;
    *ldx = ldsx;

    return true;
}

// -- test functions
bool BalanceTray::calculatePointPressingKeyboard(std::vector<double> *rdx, std::vector<double> *ldx){
    int cKey;
    char plane ='0';
    double x_increment= 0.001;
    double y_increment= 0.001;
    std::vector<double> rx, rdsx; // right current point, right destination point
    std::vector<double> lx, ldsx; // left current point, left destination point

    if(!getRefPosition(&rx, &lx)){
        CD_ERROR("Getting last position\n");
        return false;
    }
    else CD_SUCCESS("Got reference position\n");

    // Read the keyboard
    cKey = StaticLibrary::getch();

    // copy current to destination
    rdsx = rx;
    ldsx = lx;

    switch(cKey)
    {
    case 65:
        printf("Presiono flecha Arriba\n");
        rdsx[4] = rdsx[4] + y_increment;
        ldsx[4] = ldsx[4] + y_increment;
        break;

    case 66:
        printf("Presiono flecha Abajo\n");       
        rdsx[4] = rdsx[4] - y_increment;
        ldsx[4] = ldsx[4] - y_increment;
        break;

    case 68:
        printf("Presiono flecha izquierda\n");        
        rdsx[3] = rdsx[3] - x_increment;
        ldsx[3] = ldsx[3] - x_increment;
        break;

    case 67:
        printf("Presiono flecha derecha\n");        
        rdsx[3] = rdsx[3] + x_increment;
        ldsx[3] = ldsx[3] + x_increment;
        break;

    default:
        CD_DEBUG_NO_HEADER("Repose position\n");
        break;
    }

        // transformation: Axis Angle Scaled -> Axis Angle
        KinRepresentation::decodePose(rdsx, rdsxaa, KinRepresentation::CARTESIAN, KinRepresentation::AXIS_ANGLE, KinRepresentation::DEGREES );
        KinRepresentation::decodePose(ldsx, ldsxaa, KinRepresentation::CARTESIAN, KinRepresentation::AXIS_ANGLE, KinRepresentation::DEGREES );

        // Checks the joint limits!
        CD_DEBUG_NO_HEADER("R-POSS: [");
        for(int i=0; i<rdsxaa.size(); i++){
            CD_DEBUG_NO_HEADER("%f ",rdsxaa[i]);
        }
        CD_DEBUG_NO_HEADER("]\n");

        CD_DEBUG_NO_HEADER("L-POSS: [");
        for(int i=0; i<ldsxaa.size(); i++){
            CD_DEBUG_NO_HEADER("%f ",ldsxaa[i]);
        }
        CD_DEBUG_NO_HEADER("]\n");

        if(rdsxaa[6]>6 || ldsxaa[6]>6){
            CD_WARNING("Turning STOP (> 6º)!!\n");
            // send to the pointer
            *rdx = rdsx;
            *ldx = ldsx;
            return false;
        }

        if(!setRefPosition(rdsx, ldsx)){
            CD_ERROR("Saving reference position\n");
            return false;
        }

        // send to the pointer
        *rdx = rdsx;
        *ldx = ldsx;

        return true;
}

/************ REF POSITIONS *************************/

bool BalanceTray::homePosition(){
    // Prepare the last position        
        CD_INFO("Preparing homing position...\n");
        configArmsToPosition(10,10);
        double rightArmPoss[7] = {-27.0, -25.5,  28.6, -78.7,  57.5, -70.6};
        double leftArmPoss[7]  = {-27.0,  25.5, -28.6, -78.7, -57.5, -70.6};
        std::vector<double> rightArm(&rightArmPoss[0], &rightArmPoss[0]+7); //teoSim (+6) teo (+7)
        std::vector<double> leftArm(&leftArmPoss[0], &leftArmPoss[0]+7);
        if(!moveJointsInPosition(rightArm, leftArm)){
            CD_ERROR("\n");
            return false;
        }

        std::vector<double> rightArmFK(6);        
        if(! getRightArmFwdKin(&rightArmFK))
            CD_ERROR("Doing Forward Kinematic of right-arm...\n");

        std::vector<double> leftArmFK(6);        
        if(! getLeftArmFwdKin(&leftArmFK))
            CD_ERROR("Doing Forward Kinematic of left-arm...\n");

        if(setRefPosition(rightArmFK, leftArmFK))
            CD_SUCCESS("Reference position saved\n");

        return true;
}

bool BalanceTray::setRefPosition(std::vector<double> rx, std::vector<double> lx){;
    rightArmRefpos = rx;
    leftArmRefpos = lx;
    if(!rightArmRefpos.empty() && !leftArmRefpos.empty())
        return true;
    else
        return false;
}

bool BalanceTray::getRefPosition(std::vector<double> *rx, std::vector<double> *lx){;
    *rx = rightArmRefpos;
    *lx = leftArmRefpos;
    if(!rightArmRefpos.empty() && !leftArmRefpos.empty())
        return true;
    else
        return false;
}

/************ SHOWING DIFFERENT VALUES *******************/

void BalanceTray::printFKinAAS(){
    printf("R-arm pose : [");
    std::vector<double> rightArmPoint(6);
    if(! getRightArmFwdKin(&rightArmPoint))
        CD_ERROR("Doing Forward Kinematic of right-arm...\n");
    for(int i=0; i<rightArmPoint.size(); i++)
        printf("%f ",rightArmPoint[i]);
    printf("]\n");

    printf("L-arm pose: [");
    std::vector<double> leftArmPoint(6);
    if(! getLeftArmFwdKin(&leftArmPoint))
        CD_ERROR("Doing Forward Kinematic of left-arm...\n");
    for(int i=0; i<leftArmPoint.size(); i++)
        printf("%f ",leftArmPoint[i]);
    printf("]\n");
}

void BalanceTray::printFKinAA(){
    std::vector<double> rightArmPoint(6);
    if(! getRightArmFwdKin(&rightArmPoint))
        CD_ERROR("Doing Forward Kinematic of right-arm...\n");
    std::vector<double> rightArmPointInAxisAngle(7); // axis angle
    KinRepresentation::decodePose(rightArmPoint, rightArmPointInAxisAngle, KinRepresentation::CARTESIAN, KinRepresentation::AXIS_ANGLE, KinRepresentation::DEGREES );
    printf("R-arm pose: [");
    for(int i=0; i<rightArmPointInAxisAngle.size(); i++)
        printf("%f ",rightArmPointInAxisAngle[i]);
    printf("]\n");

    std::vector<double> leftArmPoint(6);
    if(! getLeftArmFwdKin(&leftArmPoint))
        CD_ERROR("Doing Forward Kinematic of left-arm...\n");
    std::vector<double> leftArmPointInAxisAngle(7); // axis angle
    KinRepresentation::decodePose(leftArmPoint, leftArmPointInAxisAngle, KinRepresentation::CARTESIAN, KinRepresentation::AXIS_ANGLE, KinRepresentation::DEGREES );
    printf("L-arm pose: [");
    for(int i=0; i<leftArmPointInAxisAngle.size(); i++)
        printf("%f ",leftArmPointInAxisAngle[i]);
    printf("]\n");
}

void BalanceTray::printJr3(yarp::sig::Vector values)
{
    CD_INFO_NO_HEADER("JR3-R: ( ");
    for(int i=12; i<18; i++)
        CD_INFO_NO_HEADER("%f ",values[i]);
    CD_INFO_NO_HEADER(")\n");

    CD_INFO_NO_HEADER("JR3-L: ( ");
    for(int i=18; i<24; i++)
        CD_INFO_NO_HEADER("%f ",values[i]);
    CD_INFO_NO_HEADER(")\n");
}

/************** GET rotation information *****************/

bool BalanceTray::getAxisRotation(std::vector<double> *axisRotation){
    std::vector<double> rightArmPoint(6);
    if(! getRightArmFwdKin(&rightArmPoint))
        CD_ERROR("Doing Forward Kinematic of right-arm...\n");

    std::vector<double> rightArmPointInAxisAngle(7); // axis angle
    KinRepresentation::decodePose(rightArmPoint, rightArmPointInAxisAngle, KinRepresentation::CARTESIAN, KinRepresentation::AXIS_ANGLE, KinRepresentation::DEGREES );

    axisRotation->resize(4);
    std::copy(rightArmPointInAxisAngle.begin() + 3, rightArmPointInAxisAngle.end(), axisRotation->begin());
    /*
    for(std::vector<double>::iterator it = axisRotation->begin(); it != axisRotation->end(); ++it)
        printf("%f ",*it);
    printf("]\n");
    */
    return true;
}

/************** Write information in CSV file ***************/

bool BalanceTray::writeInfo2Csv(double timeStamp, std::vector<double> axisRotation, yarp::sig::Vector jr3Values)
{
    CD_WARNING_NO_HEADER("%.4f ", timeStamp);
    CD_WARNING_NO_HEADER("%.4f %.4f %.4f %.4f ", axisRotation[0], axisRotation[1], axisRotation[2], axisRotation[3]); // axis rotation
    CD_WARNING_NO_HEADER("%.4f %.4f\n", jr3Values[13], jr3Values[19]); // +y -y
    fprintf(fp,"%.4f, ", timeStamp);
    fprintf(fp,"%.4f, %.4f, %.4f, %.4f, ", axisRotation[0], axisRotation[1], axisRotation[2], axisRotation[3]); // axis rotation
    fprintf(fp,"%.4f, %.4f\n", jr3Values[13], jr3Values[19]); // +y -y
}

}  // namespace teo

