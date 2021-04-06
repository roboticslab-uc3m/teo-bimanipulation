// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "BalanceTray.hpp"

#include <algorithm>

#include <kdl/path_line.hpp>
#include <kdl/rotational_interpolation_sa.hpp>
#include <kdl/trajectory_segment.hpp>
#include <kdl/velocityprofile_rect.hpp>
#include <kdl/velocityprofile_trap.hpp>

#include <yarp/os/LogStream.h>
#include <KdlVectorConverter.hpp>

using namespace roboticslab::KinRepresentation;
using namespace roboticslab::KdlVectorConverter;
using namespace teo;

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
        printf("\t--mode: %s [%s] (select: jr3Balance / keyboard / testMov )\n", mode.c_str(), DEFAULT_MODE);
        printf("\t--jr3ToCsv: to export data read by JR3 sensors\n");
        printf("\t--speak \n""");
        ::exit(0);
    }

    // operation modes
    jr3Balance = false;
    keyboard   = false;
    testMov    = false;
    jr3ToCsv   = false;

    // speak sentences
    speak = false;

    // initialice variables
    //rightArmTrajThread = 0;
    //leftArmTrajThread = 0;
    //fp = 0;

    /** Configure different MODES of the application **/
    if(mode == "jr3Balance")
    {
        yInfo() << "Mode JR3 with balance tray [activated]";
        jr3Balance = true;
    }
    else if(mode == "keyboard")
    {
        yInfo() << "Mode KEYBOARD [activated]";
        keyboard = true;
    }
    else if(mode == "testMov")
    {
        yInfo() << "Mode TESTMOV with defined trajectories [activated]";
        testMov = true;
    }
    else {
        yInfo() << "MODE not recognised to work";
        return false;
    }

    // checks options: speak & jr3ToCsv

    if(rf.check("jr3ToCsv"))
    {
        yInfo() << "Mode JR3TOCSV [activated]";
        jr3ToCsv = true;
    }

    if(rf.check("speak"))
    {
        yInfo() << "Mode SPEAK [activated]";
        speak = true;
    }
    else yInfo() << "Mode SPEAK [deactivated]";

    std::string balanceTrayStr("/balanceTray");

    // ------ ANALOG SENSOR ------   
    if(jr3Balance || jr3ToCsv){
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
            yError() << "Problems acquiring JR3 interface";
            return 1;
        }

        yInfo() << "Acquired JR3 interface";
    }

    // ------ RIGHT ARM -------

    yarp::os::Property rightArmOptions;
    rightArmOptions.put("device","remote_controlboard");
    rightArmOptions.put("remote","/"+robot+"/rightArm");
    rightArmOptions.put("local",balanceTrayStr+"/"+robot+"/rightArm");
    rightArmDevice.open(rightArmOptions);
    if(!rightArmDevice.isValid()) {
        yError() << "Robot rightArm device not available";
        rightArmDevice.close();
        yarp::os::Network::fini();
        return false;
    }    

    // connecting our device with "IEncoders" interface
    if (!rightArmDevice.view(rightArmIEncoders) ) {
        yError() << "Problems acquiring rightArmIEncoders interface";
        return false;
    }
    else
    {
        yInfo() << "Acquired leftArmIEncoders interface";
        if(!rightArmIEncoders->getAxes(&numRightArmJoints))
            yError() << "Problems acquiring numRightArmJoints";
        else yWarning() << "Number of joints:" << numRightArmJoints;
    }

    // connecting our device with "control mode" interface, initializing which control mode we want (position)
    if (!rightArmDevice.view(rightArmIControlMode) ) {
        yError() << "Problems acquiring rightArmIControlMode interface";
        return false;
    } else
        yInfo() << "Acquired rightArmIControlMode interface";

    // connecting our device with "PositionControl" interface
    if (!rightArmDevice.view(rightArmIPositionControl) ) {
        yError() << "Problems acquiring rightArmIPositionControl interface";
        return false;
    } else
        yInfo() << "Acquired rightArmIPositionControl interface";

    // connecting our device with "PositionDirect" interface
    if (!rightArmDevice.view(rightArmIPositionDirect) ) {
        yError() << "Problems acquiring rightArmIPositionDirect interface";
        return false;
    } else
        yInfo() << "Acquired rightArmIPositionDirect interface";


    // ------ LEFT ARM -------

    yarp::os::Property leftArmOptions;
    leftArmOptions.put("device","remote_controlboard");
    leftArmOptions.put("remote","/"+robot+"/leftArm");
    leftArmOptions.put("local",balanceTrayStr+"/"+robot+"/leftArm");
    leftArmDevice.open(leftArmOptions);
    if(!leftArmDevice.isValid()) {
        yError() << "Robot leftArm device not available";
        leftArmDevice.close();
        yarp::os::Network::fini();
        return false;
    }

    // connecting our device with "IEncoders" interface   
    if (!leftArmDevice.view(leftArmIEncoders) ) {
        yError() << "Problems acquiring leftArmIEncoders interface";
        return false;
    } else {
        yInfo() << "Acquired leftArmIEncoders interface";
        if(!leftArmIEncoders->getAxes(&numLeftArmJoints))
            yError() << "Problems acquiring numLeftArmJoints";
    }

    // connecting our device with "control mode" interface, initializing which control mode we want (position)
    if (!leftArmDevice.view(leftArmIControlMode) ) {
        yError() << "Problems acquiring leftArmIControlMode interface";
        return false;
    } else
        yInfo() << "Acquired leftArmIControlMode interface";

    // connecting our device with "position control" interface (configuring our device: speed, acceleration... and sending joint positions)
    if (!leftArmDevice.view(leftArmIPositionControl) ) {
        yError() << "Problems acquiring leftArmIPositionControl interface";
        return false;
    } else
        yInfo() << "Acquired leftArmIPositionControl interface";

    // -- connecting our device with "PositionDirect" interface
    if (!leftArmDevice.view(leftArmIPositionDirect) ) {
        yError() << "Problems acquiring leftArmIPositionDirect interface";
        return false;
    } else
        yInfo() << "Acquired leftArmIPositionDirect interface";


    // ----- Configuring KDL Solver for right-arm -----

    if( ! rightArmDevice.view(rightArmIControlLimits) ) {
        yError() << "Could not view iControlLimits in rightArmDevice";
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
            yInfo("Joint %d limits: [%f,%f]",joint,min,max);
        }

    yarp::os::Property rightArmSolverOptions;
    std::string rightKinPath = rf.findFileByName("teo-fixedTrunk-rightArm-fetch.ini");;
    rightArmSolverOptions.fromConfigFile(rightKinPath);
    rightArmSolverOptions.put("device","KdlSolver");
    rightArmSolverOptions.put("mins", yarp::os::Value::makeList(qrMin.toString().c_str()));
    rightArmSolverOptions.put("maxs", yarp::os::Value::makeList(qrMax.toString().c_str()));
    rightArmSolverOptions.put("ik", "st"); // to use screw theory IK
    rightArmSolverDevice.open(rightArmSolverOptions);

    if( ! rightArmSolverDevice.isValid() )
    {
        yError() << "KDLSolver solver device for right-arm is not valid";
        return false;
    }

    if( ! rightArmSolverDevice.view(rightArmICartesianSolver) )
    {
        yError() << "Could not view iCartesianSolver in KDLSolver";
        return false;
    }

    yInfo() << "Acquired rightArmICartesianSolver interface";

    // ----- Configuring KDL Solver for left-arm -----

    if( ! leftArmDevice.view(leftArmIControlLimits) ) {
        yError() << "Could not view iControlLimits in leftArmDevice";
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
            yInfo("Joint %d limits: [%f,%f]",joint,min,max);
        }

    yarp::os::Property leftArmSolverOptions;    
    std::string leftKinPath = rf.findFileByName("teo-fixedTrunk-leftArm-fetch.ini");;
    leftArmSolverOptions.fromConfigFile(leftKinPath);
    leftArmSolverOptions.put("device", "KdlSolver");
    leftArmSolverOptions.put("mins", yarp::os::Value::makeList(qlMin.toString().c_str()));
    leftArmSolverOptions.put("maxs", yarp::os::Value::makeList(qlMax.toString().c_str()));
    leftArmSolverOptions.put("ik", "st"); // to use screw theory IK
    leftArmSolverDevice.open(leftArmSolverOptions);

    if( ! leftArmSolverDevice.isValid() )
    {
        yError() << "KDLSolver solver device for left-arm is not valid";
        return false;
    }

    yInfo() << "Acquired rightArmICartesianSolver interface";

    if( ! leftArmSolverDevice.view(leftArmICartesianSolver) )
    {
        yError() << "Could not view iCartesianSolver in KDLSolver";
        return false;
    }

    yInfo() << "Acquired leftArmICartesianSolver interface";

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
    }
    else if(robot=="teo")
    {
        for(int i; i<6; i++)
            twist_right_N_T.push_back(rightArmTeoRobot[i]);
        for(int i; i<6; i++)
            twist_left_N_T.push_back(leftArmTeoRobot[i]);
    }
    else
    {
        yError() << "Unsupported robot:" << robot;
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
        yInfo() << "Home position [OK]";
        printFKinAA();
    }

    if (speak) dialogueManager->ttsSay("Por favor, coloca la bandeja en mis manos, y cuando este lista, pulsa cualquier tecla para comenzar");

    yWarning() << "Press a key to CALIBRATE SENSORS...";
    getchar();

    // Calibrate sensor ... JR3
    if(jr3Balance || jr3ToCsv){
        int ret = iAnalogSensor->calibrateSensor();        
        if(ret!=0){
            yError() << "Calibrating sensors...";
            return false;
        }
        else yInfo() << "JR3 sensors calibrated";
    }

    yWarning() << "Put the tray or and object and press a key...";
    getchar();

    if (speak) dialogueManager->ttsSay("un momento por favor");

    if(!configArmsToPositionDirect()) {
        yError() << "Configuring drivers to Position Direct";
        return false;
    }

    yInfo() << "Configured to Position Direct";

    if (speak) dialogueManager->ttsSay("Sensores de fuerza par calibrados. Que comience el juego");

    // start reading and sending information thread: JR3/keyboard (10ms) -> BalanceThread (50ms)
    this->start();

    if(jr3ToCsv) // we are going to create a offline saved trajectories to test the jr3 sensors
    {
       fp = fopen("../data.csv","w+");
       fprintf(fp, "time, iteration, axis_x, axis_y, axis_z, rotation angle, rfx, rfy, rfz, rmx, rmy, rmz, lfx, lfy, lfz, lmx, lmy, lmz\n");       
    }

    if(testMov)
    {
       double increment = (PI/180); // 1 degree = 0,0174533 rad

       //rotateTrayByTrajectory(0,increment,1,10);

       for(i=1; i<=5; i++){ // menos de 5º
            printf("-> Iteration: %d\n", i);
            // axis, angle, duration, maxvel
            rotateTrayByTrajectory(0,increment,2,10);
            yarp::os::Time::delay(4);
       }

       for(i=4; i>=-5; i--){ // menos de 5º
            printf("-> Iteration: %d\n", i);
            // axis, angle, duration, maxvel
            rotateTrayByTrajectory(0,-increment,2,10);
            yarp::os::Time::delay(4);
       }

       for(i=-4; i<=0; i++){ // menos de 5º
            printf("-> Iteration: %d\n", i);
            // axis, angle, duration, maxvel
            rotateTrayByTrajectory(0,increment,2,10);
            yarp::os::Time::delay(4);
       }

    }

    // else: mode keyboard/jr3Balance
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
        if(jr3Balance || jr3ToCsv){
            int ret = iAnalogSensor->read(sensorValues);
            if(ret!=yarp::dev::IAnalogSensor::AS_OK)
            {
                yError() << "Reading JR3";
                return;
            }
        }

        if(jr3ToCsv)
        {
            std::vector<double> axisRotation;
            if(!getAxisRotation(&axisRotation))
            {
                yError() << "Getting tray rotation";
                return;
            }
            writeInfo2Csv(Time::now()-initTime, axisRotation, sensorValues);
        }

        if(jr3Balance)
        {
            // reading JR3 sensor
            printJr3(sensorValues);

            if(!calculatePointOpposedToForce(sensorValues, &rdx, &ldx)){
                yError() << "Calculating next point";
                return;
            }

            rightArmBalThread->setCartesianPosition(rdx);
            leftArmBalThread->setCartesianPosition(ldx);
        }


        else if(keyboard)
        // reading keyboard
        {
            // if the iteration of the thread is the first, copy the position.
            // this fixs the warning messages: "KdlVectorConverter.cpp:13 vectorToFrame(): Size mismatch; expected: 6, was: 0"
            if( this->getIterations()!=0 )
                calculatePointPressingKeyboard(&rdx, &ldx);
            else getRefPosition(&rdx,&ldx);

            rightArmBalThread->setCartesianPosition(rdx);
            leftArmBalThread->setCartesianPosition(ldx);
        }
}

/****************************************************/

bool BalanceTray::getRightArmFwdKin(std::vector<double> *currentX)
{
    /** ----- Obtain current joint position ----- **/    
    std::vector<double> currentQ(numRightArmJoints);
    if ( ! rightArmIEncoders->getEncoders( currentQ.data() ) ){
        yError() << "getEncoders() failed (right arm)";
        return false;
    }


    /** ----- Obtain current cartesian position ---------- **/
    if ( ! rightArmICartesianSolver->fwdKin(currentQ, *currentX) )    {
        yError() << "fwdKin() failed (right arm)";
        return false;
    }

    return true;
}

bool BalanceTray::getLeftArmFwdKin(std::vector<double> *currentX)
{
    /** ----- Obtain current joint position ----- **/
    std::vector<double> currentQ(numLeftArmJoints);
    if ( ! leftArmIEncoders->getEncoders( currentQ.data() ) ){
        yError() << "getEncoders() failed (left arm)";
        return false;
    }

    /** ----- Obtain current cartesian position ---------- **/
    if ( ! leftArmICartesianSolver->fwdKin(currentQ, *currentX) )    {
        yError() << "fwdKin() failed (left arm)";
        return false;
    }

    return true;
}

/************ CONFIGURATION MODES FOR DEVICES *******/

bool BalanceTray::configArmsToPosition(double sp, double acc){

    // -- Speed and acceleration for 7 joints
    std::vector<double> armSpeeds(numRightArmJoints, sp); // 7,30.0
    std::vector<double> armAccelerations(numRightArmJoints, acc); // 7,30.0

    // -- Configuring Devices to Position Mode
    std::vector<int> rightArmControlModes(numRightArmJoints,VOCAB_CM_POSITION);
    if(! rightArmIControlMode->setControlModes(rightArmControlModes.data())){
        yError() << "Problems setting position control mode of: right-arm";
        return false;
    }


    std::vector<int> leftArmControlModes(numLeftArmJoints,VOCAB_CM_POSITION);
    if(! leftArmIControlMode->setControlModes( leftArmControlModes.data() )){
        yError() << "Problems setting position control mode of: left-arm";
        return false;
    }

    // -- Configuring speed and acceleration
    if(!rightArmIPositionControl->setRefSpeeds(armSpeeds.data())){
        yError() << "Problems setting reference speed on right-arm joints";
        return false;
    }
    if(!leftArmIPositionControl->setRefSpeeds(armSpeeds.data())){
        yError() << "Problems setting reference speed on left-arm joints";
        return false;
    }
    if(!rightArmIPositionControl->setRefAccelerations(armAccelerations.data())){
        yError() << "Problems setting reference acceleration on right-arm joints";
        return false;
    }
    if(!leftArmIPositionControl->setRefAccelerations(armAccelerations.data())){
        yError() << "Problems setting reference acceleration on left-arm joints";
        return false;
    }

    return true;
}

bool BalanceTray::configArmsToPositionDirect()
{
    yInfo() << "Configuring drivers to Position Direct...";
    /* This is used with external reference implementation in yarp-devices
     * This APP is working well with old-pt mode
    if(robot=="teo"){ // if we are using the robot, better use PT Mode
        yarp::os::Bottle val;
        yarp::os::Bottle & b = val.addList();
        b.addInt32(PT_MODE_MS);

        if(! rightArmIRemoteVariables->setRemoteVariable("ptModeMs", val)){
            yError() << "Problems setting ptModeMs remote variable of: right-arm";
            return false;
        }

        if(! leftArmIRemoteVariables->setRemoteVariable("ptModeMs", val)){
            yError() << "Problems setting ptModeMs remote variable of: left-arm";
            return false;
        }
    }
    */
    // if there is no ptModeMs (=0), it's activate the external reference
    std::vector<int> rightArmControlModes(numRightArmJoints,VOCAB_CM_POSITION_DIRECT);
    if(! rightArmIControlMode->setControlModes(rightArmControlModes.data())){
        yError() << "Problems setting POSITION DIRECT mode of: right-arm";
        return false;
    }

    std::vector<int> leftArmControlModes(numLeftArmJoints,VOCAB_CM_POSITION_DIRECT);
    if(! leftArmIControlMode->setControlModes(leftArmControlModes.data())){
        yError() << "Problems setting POSITION DIRECT mode of: left-arm";
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
        yDebug() << "!doneRight";
    }

    while(!doneLeft)
    {
        yarp::os::Time::delay(0.1);
        leftArmIPositionControl->checkMotionDone(&doneLeft);
        yDebug() << "!doneLeft";
    }

    return true;
}

/************ EXECUTE TRAJECTORY ********************/
// Offline trajectories : This function are not used by the moment.
bool BalanceTray::executeTrajectory(std::vector<double> rx, std::vector<double> lx, std::vector<double> rxd, std::vector<double> lxd, double duration, double maxvel)
{
    // trajectory for right-arm
    auto * pathRA = new KDL::Path_Line(vectorToFrame(rx), vectorToFrame(rxd), new KDL::RotationalInterpolation_SingleAxis(), 1.0);
    auto * profileRA = new KDL::VelocityProfile_Trap(maxvel, 10.0);
    auto * trajectoryRA = new KDL::Trajectory_Segment(pathRA, profileRA, duration);

    // trajectory for left-arm
    auto * pathLA = new KDL::Path_Line(vectorToFrame(lx), vectorToFrame(lxd), new KDL::RotationalInterpolation_SingleAxis(), 1.0);
    auto * profileLA = new KDL::VelocityProfile_Trap(maxvel, 10.0);
    auto * trajectoryLA = new KDL::Trajectory_Segment(pathLA, profileLA, duration);

    if (rightArmTrajThread == 0)
        rightArmTrajThread = new TrajectoryThread(rightArmIEncoders, rightArmICartesianSolver, rightArmIPositionDirect, PT_MODE_MS );


    if (leftArmTrajThread == 0)
        leftArmTrajThread = new TrajectoryThread(leftArmIEncoders, leftArmICartesianSolver, leftArmIPositionDirect, PT_MODE_MS );

    rightArmTrajThread->setICartesianTrajectory(trajectoryRA);
    leftArmTrajThread->setICartesianTrajectory(trajectoryLA);

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

    delete trajectoryRA;
    delete trajectoryLA;

    return true;
}

bool BalanceTray::rotateTrayByTrajectory(int axis, double angle, double duration, double maxvel){

    // first check
    if(axis<0 && axis>2){
        yError() << "Axis check failed";
        return false;
    }

    std::vector<double> rx, rdx;
    std::vector<double> lx, ldx;

    if(!getRefPosition(&rx, &lx)){
        yError() << "Getting last position";
        return false;
    }

    rdx = rx;
    ldx = lx;

    rdx[axis+3] = rdx[axis+3] + angle;
    ldx[axis+3] = ldx[axis+3] + angle;

    if(setRefPosition(rdx, ldx))
        yInfo() << "Saved reference position";

    if(!executeTrajectory(rx, lx, rdx, ldx, duration, maxvel)){
        yError() << "Doing trajectory";
        return false;
    }

    return true;
}

// Online trajectories: Function used to calculate the next point in relation to the force exerted on the sensors (movement opposite to the force to balance the tray)
bool BalanceTray::calculatePointOpposedToForce(yarp::sig::Vector sensor, std::vector<double> *rdx, std::vector<double> *ldx){
    char plane ='0';
    double offset= 0.0;
    double increment = 0;
    std::vector<double> rx, rdsx; // right current point, right destination point
    std::vector<double> lx, ldsx; // left current point, left destination point

    if(!getRefPosition(&rx, &lx)){
        yError() << "Getting last position";
        return false;
    }

    // calculating position increment
    // -- Turning X axis (negative value)
    //    for the first condition, the force of the object on the RIGHT sensor must be greater than 0.06 and must be greater than the absolute value of the opposite side of the tray.

    if(sensor[13] > 0.06 && (std::abs(sensor[13])+offset)>(std::abs(sensor[19])) ){
        yWarning("PRESURE DETECTED RIGHT: [%f]>[%f]", (std::abs(sensor[13])+offset), sensor[19]);
        increment=-0.00015*std::abs(sensor[13]); // increment value of the distance between points (-0.00014)
        plane = 'x';
    }

    // -- Turning X axis (positive value)
    //    for the second condition, the force of the object on the LEFT sensor must be less than -0.06 and must be greater than the absolute value of the opposite side of the tray.

    else if(sensor[19] < -0.06 && std::abs(sensor[19])>(std::abs(sensor[13])+offset) ){
        yWarning("PRESURE DETECTED LEFT: [%f]<[%f]", (std::abs(sensor[13])+offset), sensor[19]);
        increment=0.00015*std::abs(sensor[19]); // increment value of the distance between points
        plane = 'x';
    }

    // -- Turning Y axis (positive value)
    //    for the first condition, the torsional force of the object on the RIGHT sensor and LEFT sensor must be less than -0.08 and greater than 0.08 respectively

    if((sensor[17] < -0.15) || (sensor[23] > +0.15))
    {
        yWarning("PRESURE DETECTED DOWN: [%f][%f]", sensor[17], sensor[23]);
        increment=0.0005*(std::abs(sensor[17])+std::abs(sensor[23]));// increment value of the distance between points
        plane = 'y';
    }

    // -- Turning Y axis (negative value)
    //    for the second condition, the torsional force of the object on the RIGHT sensor and LEFT sensor must be greater than 0.08 and less than -0.08 respectively
    else if((sensor[17] > 0.1) || (sensor[23] < -0.1))
    {
        yWarning("PRESURE DETECTED UP: [%f][%f]", sensor[17], sensor[23]);
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
            yWarning("turning value of X: F[right: %f] F[left: %f]", rdsx[3], ldsx[3]);
            break;
        case 'y':
            // increment rotation value in Y axis
            rdsx[4] += increment;
            ldsx[4] += increment;
            yWarning("turning value of Y: M[right %f] M[left %f]", rdsx[4], ldsx[4]);
            break;

        /* Not used by the moment
        case 'z':
            // increment rotation value in Z axis
            rdsx[5] = rdsx[5] + increment;
            ldsx[5] = ldsx[5] + increment;
            break;
        */
        case '0':
            yInfo() << "Repose position";
            break;
    }

    // transformation: Axis Angle Scaled -> Axis Angle
    //std::vector<double> rdsxaa, ldsxaa; -> global
    decodePose(rdsx, rdsxaa, coordinate_system::CARTESIAN, orientation_system::AXIS_ANGLE, angular_units::DEGREES );
    decodePose(ldsx, ldsxaa, coordinate_system::CARTESIAN, orientation_system::AXIS_ANGLE, angular_units::DEGREES );

    // Checks the joint limits!
    yDebug() << "R-POSS:" << rdsxaa;
    yDebug() << "L-POSS:" << ldsxaa;

    if(rdsxaa[6]>5 || ldsxaa[6]>5){
        yWarning() << "Turning STOP (> 5º)!!";
        return false;
    }

    if(!setRefPosition(rdsx, ldsx)){
        yDebug() << "Saving reference position";
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
        yError() << "Getting last position";
        return false;
    }
    else yInfo() << "Got reference position";

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
        yDebug() << "Repose position";
        break;
    }

        // transformation: Axis Angle Scaled -> Axis Angle
        decodePose(rdsx, rdsxaa, coordinate_system::CARTESIAN, orientation_system::AXIS_ANGLE, angular_units::DEGREES );
        decodePose(ldsx, ldsxaa, coordinate_system::CARTESIAN, orientation_system::AXIS_ANGLE, angular_units::DEGREES );

        // Checks the joint limits!
        yDebug() << "R-POSS:" << rdsxaa;
        yDebug() << "L-POSS:" << ldsxaa;

        if(rdsxaa[6]>6 || ldsxaa[6]>6){
            yWarning() << "Turning STOP (> 6º)!!";
            // send to the pointer
            *rdx = rdsx;
            *ldx = ldsx;
            return false;
        }

        if(!setRefPosition(rdsx, ldsx)){
            yError() << "Saving reference position";
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
        yInfo() << "Preparing homing position...";
        configArmsToPosition(10,10);
        double rightArmPoss[6] = {-27.0, -25.5,  28.6, -78.7,  57.5, -70.6};
        double leftArmPoss[6]  = {-27.0,  25.5, -28.6, -78.7, -57.5, -70.6};
        std::vector<double> rightArm(&rightArmPoss[0], &rightArmPoss[0]+6); //teoSim (+6) teo (+7)
        std::vector<double> leftArm(&leftArmPoss[0], &leftArmPoss[0]+6);
        if(!moveJointsInPosition(rightArm, leftArm)){
            yError() << "moveJointsInPosition() failed";
            return false;
        }

        std::vector<double> rightArmFK(6);        
        if(! getRightArmFwdKin(&rightArmFK))
            yError() << "Doing Forward Kinematic of right-arm...";

        std::vector<double> leftArmFK(6);        
        if(! getLeftArmFwdKin(&leftArmFK))
            yError() << "Doing Forward Kinematic of left-arm...";

        if(setRefPosition(rightArmFK, leftArmFK))
            yInfo() << "Reference position saved";

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
        yError() << "Doing Forward Kinematic of right-arm...";
    for(int i=0; i<rightArmPoint.size(); i++)
        printf("%f ",rightArmPoint[i]);
    printf("]\n");

    printf("L-arm pose: [");
    std::vector<double> leftArmPoint(6);
    if(! getLeftArmFwdKin(&leftArmPoint))
        yError() << "Doing Forward Kinematic of left-arm...";
    for(int i=0; i<leftArmPoint.size(); i++)
        printf("%f ",leftArmPoint[i]);
    printf("]\n");
}

void BalanceTray::printFKinAA(){
    std::vector<double> rightArmPoint(6);
    if(! getRightArmFwdKin(&rightArmPoint))
        yError() << "Doing Forward Kinematic of right-arm...";
    std::vector<double> rightArmPointInAxisAngle(7); // axis angle
    decodePose(rightArmPoint, rightArmPointInAxisAngle, coordinate_system::CARTESIAN, orientation_system::AXIS_ANGLE, angular_units::DEGREES );
    printf("R-arm pose: [");
    for(int i=0; i<rightArmPointInAxisAngle.size(); i++)
        printf("%f ",rightArmPointInAxisAngle[i]);
    printf("]\n");

    std::vector<double> leftArmPoint(6);
    if(! getLeftArmFwdKin(&leftArmPoint))
        yError() << "Doing Forward Kinematic of left-arm...";
    std::vector<double> leftArmPointInAxisAngle(7); // axis angle
    decodePose(leftArmPoint, leftArmPointInAxisAngle, coordinate_system::CARTESIAN, orientation_system::AXIS_ANGLE, angular_units::DEGREES );
    printf("L-arm pose: [");
    for(int i=0; i<leftArmPointInAxisAngle.size(); i++)
        printf("%f ",leftArmPointInAxisAngle[i]);
    printf("]\n");
}

void BalanceTray::printJr3(yarp::sig::Vector values)
{
    yInfo() << "JR3-R:" << values.subVector(12, 17).toString();
    yInfo() << "JR3-L:" << values.subVector(18, 23).toString();
}


/************** GET rotation information *****************/

bool BalanceTray::getAxisRotation(std::vector<double> *axisRotation){
    std::vector<double> rightArmPoint(6);
    if(! getRightArmFwdKin(&rightArmPoint))
        yError() << "Doing Forward Kinematic of right-arm...";

    std::vector<double> rightArmPointInAxisAngle(7); // axis angle
    decodePose(rightArmPoint, rightArmPointInAxisAngle, coordinate_system::CARTESIAN, orientation_system::AXIS_ANGLE, angular_units::DEGREES );

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
    //CD_WARNING_NO_HEADER("%.4f ", timeStamp);
    //CD_WARNING_NO_HEADER("%.4f %.4f %.4f %.4f ", axisRotation[0], axisRotation[1], axisRotation[2], axisRotation[3]); // axis rotation
    //CD_WARNING_NO_HEADER("%.4f %.4f\n", jr3Values[13], jr3Values[19]); // +y -y
    fprintf(fp,"%.4f, %d, ", timeStamp, i);
    fprintf(fp,"%.8f, %.8f, %.8f, %.8f, ", axisRotation[0], axisRotation[1], axisRotation[2], axisRotation[3]); // axis rotation
    fprintf(fp,"%.8f, %.8f, %.8f, %.8f, %.8f, %.8f, %.8f, %.8f, %.8f, %.8f, %.8f, %.8f\n", jr3Values[12], jr3Values[13], jr3Values[14], jr3Values[15], jr3Values[16], jr3Values[17], jr3Values[18], jr3Values[19], jr3Values[20], jr3Values[21], jr3Values[22], jr3Values[23]); // +y -y
}

