// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "BalanceTray.hpp"

namespace teo
{

/************************************************************************/

bool BalanceTray::configure(yarp::os::ResourceFinder &rf)
{
    printf("inicia el configure\n");
    std::string robot = rf.check("robot",yarp::os::Value(DEFAULT_ROBOT),"name of /robot to be used").asString();

    printf("--------------------------------------------------------------\n");
    if (rf.check("help"))
    {
        printf("BalanceTray options:\n");
        printf("\t--help (this help)\t--from [file.ini]\t--context [path]\n");
        printf("\t--robot: %s [%s]\n",robot.c_str(),DEFAULT_ROBOT);
        ::exit(0);
    }

    std::string balanceTrayStr("/balanceTray");

    // ------ RIGHT ARM -------

    yarp::os::Property rightArmOptions;
    rightArmOptions.put("device","remote_controlboard");
    rightArmOptions.put("remote",robot+"/rightArm");
    rightArmOptions.put("local",balanceTrayStr+robot+"/rightArm");
    rightArmDevice.open(rightArmOptions);
    if(!rightArmDevice.isValid()) {
      CD_ERROR("robot rightArm device not available.\n");
      rightArmDevice.close();
      yarp::os::Network::fini();
      return false;
    }    

    if (!rightArmDevice.view(rightArmIControlMode2) ) { // connecting our device with "control mode 2" interface, initializing which control mode we want (position)
        CD_ERROR("Problems acquiring rightArmIControlMode2 interface\n");
        return false;
    } else
        CD_INFO("Acquired rightArmIControlMode2 interface\n");


    if (!rightArmDevice.view(rightArmIPositionControl2) ) { // connecting our device with "position control 2" interface (configuring our device: speed, acceleration... and sending joint positions)
        CD_ERROR("Problems acquiring rightArmIPositionControl2 interface\n");
        return false;
    } else
        CD_INFO("Acquired rightArmIPositionControl2 interface\n");

    // connecting our device with "IEncoders" interface

    if (!rightArmDevice.view(rightArmIEncoders) ) {
        CD_ERROR("Problems acquiring rightArmIEncoders interface\n");
        return false;
    }
    else
    {
        CD_INFO("Acquired leftArmIEncoders interface\n");
        if(!rightArmIPositionControl2->getAxes(&numRightArmJoints))
            CD_ERROR("Problems acquiring numRightArmJoints\n");
    }

    // ------ LEFT ARM -------

    yarp::os::Property leftArmOptions;
    leftArmOptions.put("device","remote_controlboard");
    leftArmOptions.put("remote",robot+"/leftArm");
    leftArmOptions.put("local",balanceTrayStr+robot+"/leftArm");
    leftArmDevice.open(leftArmOptions);
    if(!leftArmDevice.isValid()) {
      CD_ERROR("robot leftArm device not available.\n");
      leftArmDevice.close();
      yarp::os::Network::fini();
      return false;
    }

    if (!leftArmDevice.view(leftArmIControlMode2) ) { // connecting our device with "control mode 2" interface, initializing which control mode we want (position)
        CD_ERROR("Problems acquiring leftArmIControlMode2 interface\n");
        return false;
    } else
        CD_INFO("Acquired leftArmIControlMode2 interface\n");

    if (!leftArmDevice.view(leftArmIPositionControl2) ) { // connecting our device with "position control 2" interface (configuring our device: speed, acceleration... and sending joint positions)
        CD_ERROR("Problems acquiring leftArmIPositionControl2 interface\n");
        return false;
    } else
        CD_INFO("Acquired leftArmIPositionControl2 interface\n");

    // connecting our device with "IEncoders" interface

    if (!leftArmDevice.view(leftArmIEncoders) ) { // connecting our device with "IEncoders" interface
        CD_ERROR("Problems acquiring leftArmIEncoders interface\n");
        return false;
    } else {
        CD_INFO("Acquired leftArmIEncoders interface\n");
        if(!leftArmIPositionControl2->getAxes(&numLeftArmJoints))
            CD_ERROR("Problems acquiring numLeftArmJoints\n");
    }


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
    rightArmSolverOptions.fromConfigFile("/usr/local/share/teo-configuration-files/contexts/kinematics/rightArmKinematics.ini");
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
    leftArmSolverOptions.fromConfigFile("/usr/local/share/teo-configuration-files/contexts/kinematics/leftArmKinematics.ini");
    leftArmSolverOptions.put("device", "KdlSolver");
    leftArmSolverOptions.put("mins", yarp::os::Value::makeList(qlMin.toString().c_str()));
    leftArmSolverOptions.put("maxs", yarp::os::Value::makeList(qlMax.toString().c_str()));
    //leftArmSolverOptions.put("HN", yarp::os::Value::makeList("-0.00984635  -0.9989744  -0.04419502  0.23194013 0.00484966 -0.04424435 0.99900897 0.01027258 -0.99993976 0.00962226  0.00528033 -0.00223408 0 0 0 1"));
    leftArmSolverDevice.open(leftArmSolverOptions);

    if( ! leftArmSolverDevice.isValid() )
    {
        CD_ERROR("KDLSolver solver device for left-arm is not valid \n");
        return false;
    }

    if( ! leftArmSolverDevice.view(leftArmICartesianSolver) )
    {
        CD_ERROR("Could not view iCartesianSolver in KDLSolver\n");
        return false;
    } else
        CD_SUCCESS("Acquired leftArmICartesianSolver interface\n");


    // ** Unify TCP of right-arm and left-arm: apppending link to the tray's centroid
    /** Transformation matrix between the gripper and the tray **/
    // -- teoSim:     
    //double twist_right_N_T[] = {2.31983894e-01,-1.51740342e-04, -2.22628149e-03, 1.20036222e+00,   1.21121959e+00,-1.21975782e+00};
    //double twist_left_N_T[]  = {2.31983894e-01, 1.51740342e-04, -2.22628149e-03, -1.20036222e+00,  1.21121959e+00, 1.21975782e+00};
    // -- teo    
    double twist_right_N_T[] = {2.26569395e-01, 9.62965268e-02, 9.81463895e-04, 8.95534782e-01, 1.34767916e+00, -9.14452916e-01}; //right
    double twist_left_N_T[]  = {2.30405787e-01,-1.53655717e-03, 6.52832174e-04,-1.20043758e+00, 1.20502640e+00,  1.20553506e+00}; //left

    std::vector<double> vtwist_right_N_T(&twist_right_N_T[0], &twist_right_N_T[0]+sizeof(twist_right_N_T));
    std::vector<double> vtwist_left_N_T(&twist_left_N_T[0], &twist_left_N_T[0]+sizeof(twist_left_N_T));

    rightArmICartesianSolver->appendLink(vtwist_right_N_T);
    leftArmICartesianSolver->appendLink(vtwist_left_N_T);

    // prepare position
    preparePosition();

    CD_INFO_NO_HEADER("Current FK of TCP in both arms:\n");

    // * left-arm
    std::vector<double> leftArmCurrentPose(6); // origin pose
    if(! getLeftArmFwdKin(&leftArmCurrentPose))
        CD_ERROR("Doing Forward Kinematic of left-arm\n");

    CD_INFO_NO_HEADER(" Left-arm: [");
    for(int i=0; i<leftArmCurrentPose.size(); i++)
        CD_INFO_NO_HEADER("%f ",leftArmCurrentPose[i]);
    printf("]\n ");

    // * right-arm
    std::vector<double> rightArmCurrentPose(6); // origin pose
    if(! getRightArmFwdKin(&rightArmCurrentPose))
        CD_ERROR("Doing Forward Kinematic of right-arm\n");

    CD_INFO_NO_HEADER("Right-arm: [");
    for(int i=0; i<rightArmCurrentPose.size(); i++)
        CD_INFO_NO_HEADER("%f ",rightArmCurrentPose[i]);
    printf("]\n ");

    return true;
}

/************************************************************************/

bool BalanceTray::interruptModule()
{
    this->stop(); // stop the thread
    rightArmDevice.close();
    leftArmDevice.close();
    return true;
}

/************************************************************************/

double BalanceTray::getPeriod()
{
    return 4.0; // Fixed, in seconds, the slow thread that calls updateModule below
}

/************************************************************************/

bool BalanceTray::updateModule()
{
   //printf("Entered updateModule...\n");
   return true;
}

/************************************************************************/

std::vector<std::vector<double> > BalanceTray::interpolate(std::vector<double> startPose, std::vector<double> endPose, int res)
{
    //printf("------ Interpolation results ------\n");
    // vector de vectores: se trata de un vector que contiene el conjunto de vectores
    int vsize = startPose.size(); // 6 for AXIS_ANGLE_SCALED / 7 for AXIS_ANGLE
    std::vector<std::vector<double> > path (res, std::vector<double>(vsize)); //path (poses)
    std::vector<double> factor(vsize);
    for(int i=0; i<vsize;i++){
        factor[i] = (endPose[i]-startPose[i])/res;
    }

    for(int v=0; v<res; v++){
        //CD_INFO_NO_HEADER("pose (%d): (", v+1);
        for(int i=0; i<vsize; i++){
            path[v][i]=((v+1)*factor[i]) + startPose[i];
            //CD_INFO_NO_HEADER("%f ",path[v][i]);
        }
        //CD_INFO_NO_HEADER(")\n ");
    }

    //CD_INFO_NO_HEADER("------------------------------------\n");
    return path;
}

/************************************************************************/

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
        CD_ERROR("ForgetEncoders failed\n");
        return false;
    }

    /** ----- Obtain current cartesian position ---------- **/
    if ( ! leftArmICartesianSolver->fwdKin(currentQ, *currentX) )    {
        CD_ERROR("Forward Kinematic failed.\n");
        return false;
    }

    return true;
}

/***************** CONFIGURATION MODES FOR DEVICES *********************/

bool BalanceTray::configArmsToPosition(double sp, double acc){

    // -- Speed and acceleration for 7 joints
    std::vector<double> armSpeeds(7, sp); // 7,30.0
    std::vector<double> armAccelerations(7, acc); // 7,30.0

    // -- Configuring Devices to Position Mode

    if (!rightArmDevice.view(rightArmIPositionControl2) ) { // connecting our device with "position control 2" interface (configuring our device: speed, acceleration... and sending joint positions)
        CD_ERROR("Problems acquiring rightArmIPositionControl2 interface\n");
        return false;
    } else
        CD_INFO("Acquired rightArmIPositionControl2 interface\n");

    if (!leftArmDevice.view(leftArmIPositionControl2) ) { // connecting our device with "position control 2" interface (configuring our device: speed, acceleration... and sending joint positions)
        CD_ERROR("Problems acquiring leftArmIPositionControl2 interface\n");
        return false;
    } else
        CD_INFO("Acquired leftArmIPositionControl2 interface\n");


    std::vector<int> rightArmControlModes(numRightArmJoints,VOCAB_CM_POSITION);
    if(! rightArmIControlMode2->setControlModes(rightArmControlModes.data())){
        CD_ERROR("Problems setting position control mode of: right-arm\n");
        return false;
    }


    std::vector<int> leftArmControlModes(numLeftArmJoints,VOCAB_CM_POSITION);
    if(! leftArmIControlMode2->setControlModes( leftArmControlModes.data() )){
        CD_ERROR("Problems setting position control mode of: left-arm\n");
        return false;
    }

    // -- Configuring speed and acceleration
    if(!rightArmIPositionControl2->setRefSpeeds(armSpeeds.data())){
            CD_ERROR("Problems setting reference speed on right-arm joints.\n");
            return false;
        }
        if(!leftArmIPositionControl2->setRefSpeeds(armSpeeds.data())){
            CD_ERROR("Problems setting reference speed on left-arm joints.\n");
            return false;
        }
        if(!rightArmIPositionControl2->setRefAccelerations(armAccelerations.data())){
            CD_ERROR("Problems setting reference acceleration on right-arm joints.\n");
            return false;
        }
        if(!leftArmIPositionControl2->setRefAccelerations(armAccelerations.data())){
            CD_ERROR("Problems setting reference acceleration on left-arm joints.\n");
            return false;
        }

    return true;
}

bool BalanceTray::configArmsToPositionDirect(){
        CD_INFO("Configuring drivers to Position Direct...\n");
        if (!rightArmDevice.view(rightArmIPositionDirect) ) {
                CD_ERROR("Problems acquiring rightArmIPositionDirect interface\n");
                return false;
        }

        if (!leftArmDevice.view(leftArmIPositionDirect) ) {
            CD_ERROR("Problems acquiring leftArmIPositionDirect interface\n");
            return false;
        }


        std::vector<int> rightArmControlModes(numRightArmJoints,VOCAB_CM_POSITION_DIRECT);
        if(! rightArmIControlMode2->setControlModes(rightArmControlModes.data())){
            CD_ERROR("Problems setting POSITION DIRECT mode of: right-arm\n");
            return false;
        }

        std::vector<int> leftArmControlModes(numLeftArmJoints,VOCAB_CM_POSITION_DIRECT);
        if(! leftArmIControlMode2->setControlModes(leftArmControlModes.data())){
            CD_ERROR("Problems setting POSITION DIRECT mode of: left-arm\n");
            return false;
        }
        CD_SUCCESS("Configured to Position Direct\n");
        return true;
}

/******************* MODES TO MOVE THE JOINTS **************************/

bool BalanceTray::moveJointsInPosition(std::vector<double> &rightArm, std::vector<double>& leftArm)
{
    // -- checking movement done...
    bool doneRight = false;
    bool doneLeft = false;

    // -- move to position
    if(!rightArmIPositionControl2->positionMove( rightArm.data() )){
        printf("[Error: positionMove] Problems setting new reference point for right-arm axes.\n");
        return false;
    }
    if(!leftArmIPositionControl2->positionMove( leftArm.data() )){
            printf("[Error: positionMove] Problems setting new reference point for left-arm axes.\n");
            return false;
    }

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

/******************* MODES TO MOVE THE TRAY ****************************/

/* int axis:
    axis = 0 -> x
    axis = 1 -> y
    axis = 2 -> z
*/

/************************************************************************/

/* int axis:
    axis = 0 -> x
    axis = 1 -> y
    axis = 2 -> z
*/

bool BalanceTray::moveTrayLinearlyInPosDirect(int axis, double dist, int points, double delay)
{

    std::vector<double> rightArmOrig(6); // origin pose of right-arm
    std::vector<double> rightArmDest(6); // destination pose of right-arm
    std::vector<double> leftArmOrig(6); // origin pose of left-arm
    std::vector<double> leftArmDest(6); // destination pose of left-arm

    // Resulting paths:
    std::vector<std::vector<double> > rightArmPath(points, std::vector<double>(6));
    std::vector<std::vector<double> > leftArmPath(points, std::vector<double>(6));

    // Getting fordward kinematic
    if(! getRightArmFwdKin(&rightArmOrig))
        printf("[ERROR] Doing Forward Kinematic of right-arm...\n");

    if(! getLeftArmFwdKin(&leftArmOrig))
        printf("[ERROR] Doing Forward Kinematic of left-arm...\n");

    // Calculating new destination point of right-arm and left-arm
    rightArmDest = rightArmOrig;
    rightArmDest[axis] = rightArmOrig[axis] + dist; // pto destination = pto origin + dist
    leftArmDest = leftArmOrig;
    leftArmDest[axis] = leftArmOrig[axis] + dist; // pto destination = pto origin + dist

    // interpolation
    rightArmPath = interpolate(rightArmOrig, rightArmDest, points);
    leftArmPath = interpolate(leftArmOrig, leftArmDest, points);

    std::vector<double> rightArmCurrentQ(numRightArmJoints);
    std::vector<double> rightArmDesireQ(numRightArmJoints);
    std::vector<double> leftArmCurrentQ(numRightArmJoints);
    std::vector<double> leftArmDesireQ(numRightArmJoints);

    // MAIN LOOP
    for(int i=0; i<points; i++){

        // initialize to 0
        rightArmCurrentQ.clear();
        rightArmDesireQ.clear();
        leftArmCurrentQ.clear();
        leftArmDesireQ.clear();

        // -- Right-Arm
        // currentQ: pequeña ayuda que le das al solver sobre la posición inicial en posicion articular
        if ( ! rightArmIEncoders->getEncoders( rightArmCurrentQ.data() ) ){
            printf("[ERROR] Failed getEncoders of right-arm\n");
            return false;
        }
        // inverse kinematic
        if ( ! rightArmICartesianSolver->invKin(rightArmPath[i], rightArmCurrentQ, rightArmDesireQ) )    {
            printf("[ERROR] invKin failed.\n");
            return false;
        }

        // -- Left-Arm
        if ( ! leftArmIEncoders->getEncoders( leftArmCurrentQ.data() ) ){
            printf("[ERROR] Failed getEncoders of left-arm\n");
            return false;
        }
        // inverse kinematic
        if ( ! leftArmICartesianSolver->invKin(leftArmPath[i], leftArmCurrentQ, leftArmDesireQ) )    {
            printf("[ERROR] invKin failed.\n");
            return false;
        }

        /*   [[[[[ Print values in AXIS_ANGLE]]]]]
         *   -----------------------------------------  */
        std::vector<double> currentPoint(6);
        if(! getRightArmFwdKin(&currentPoint))
            CD_ERROR("Doing Forward Kinematic of right-arm...\n");
        std::vector<double> currentPointInAxisAngle(7); // axis angle
        KinRepresentation::decodePose(currentPoint, currentPointInAxisAngle, KinRepresentation::CARTESIAN, KinRepresentation::AXIS_ANGLE, KinRepresentation::DEGREES );
        printf("Current point R-arm (%d): [", i+1);
        for(int i=0; i<currentPointInAxisAngle.size(); i++)
            printf("%f ",currentPointInAxisAngle[i]);
        printf("]\n ");

        moveJointsInPositionDirect(rightArmDesireQ, leftArmDesireQ);
        yarp::os::Time::delay(delay);
    }
    return true;
}

bool BalanceTray::rotateTrayInPosDirect(int axis, double angle, int points, double delay){

    if(axis>2 || axis<0) return false;
    else axis = axis+3; //angulos de rotación

    std::vector<double> rightArmOrig(6); // origin pose of right-arm
    std::vector<double> rightArmDest(6); // destination pose of right-arm
    std::vector<double> leftArmOrig(6); // origin pose of left-arm
    std::vector<double> leftArmDest(6); // destination pose of left-arm


    // Resulting paths:
    std::vector<std::vector<double> > rightArmPath(points, std::vector<double>(6));
    std::vector<std::vector<double> > leftArmPath(points, std::vector<double>(6));

    // Getting fordward kinematic
    if(! getRightArmFwdKin(&rightArmOrig))
        CD_ERROR("Doing Forward Kinematic of right-arm...\n");
    if(! getLeftArmFwdKin(&leftArmOrig))
        CD_ERROR("Doing Forward Kinematic of left-arm...\n");

    //Converting units more readable:  AXIS_ANGLE_SCALED ->  AXIS_ANGLE
    //std::vector<double> axisAngleArmOrig(7); // axis angle
    //KinRepresentation::decodePose(poseArmOrig, axisAngleArmOrig, KinRepresentation::CARTESIAN, KinRepresentation::AXIS_ANGLE, KinRepresentation::DEGREES );

    // Calculating new destination point of right-arm and left-arm
    rightArmDest = rightArmOrig;
    rightArmDest[axis] = rightArmOrig[axis] + angle; // pto destination = pto origin + dist
    leftArmDest = leftArmOrig;
    leftArmDest[axis] = leftArmOrig[axis] + angle; // pto destination = pto origin + dist

    // Interpolation: nos devolverá un vector (path) con todos los vectores de poses
    rightArmPath = interpolate(rightArmOrig, rightArmDest, points);
    leftArmPath = interpolate(leftArmOrig, leftArmDest, points);


    std::vector<double> rightArmCurrentQ(numRightArmJoints);
    std::vector<double> rightArmDesireQ(numRightArmJoints);

    std::vector<double> leftArmCurrentQ(numRightArmJoints);
    std::vector<double> leftArmDesireQ(numRightArmJoints);


    // MAIN LOOP
    for(int i=0; i<points; i++){

        // initialize to 0
        rightArmCurrentQ.clear();
        rightArmDesireQ.clear();
        leftArmCurrentQ.clear();
        leftArmDesireQ.clear();

        // -- Right-Arm
        if ( ! rightArmIEncoders->getEncoders( rightArmCurrentQ.data() ) ){
            CD_ERROR("Failed getEncoders of right-arm\n");
            return false;
        }
        // inverse kinematic
        if ( ! rightArmICartesianSolver->invKin(rightArmPath[i], rightArmCurrentQ, rightArmDesireQ) )    {
            CD_ERROR("invKin failed.\n");
            return false;
        }

        // -- Left-Arm
        if ( ! leftArmIEncoders->getEncoders( leftArmCurrentQ.data() ) ){
            CD_ERROR("Failed getEncoders of left-arm\n");
            return false;
        }
        // inverse kinematic
        if ( ! leftArmICartesianSolver->invKin(leftArmPath[i], leftArmCurrentQ, leftArmDesireQ) )    {
            CD_ERROR("invKin failed.\n");
            return false;
        }              

        // show FK in AAS
        showArmsFKinAA();

        // move it!
        moveJointsInPositionDirect(rightArmDesireQ, leftArmDesireQ);
        yarp::os::Time::delay(delay);
    }

    return true;
}

bool BalanceTray::recoverPosition(int points, double delay){
    double rightArmValues[6] = {0.316404, -0.005653, 0.124622, -0.399660, -0.007215, 0.004223};
    double leftArmValues[6] =  {0.315467, -0.001540, 0.221590, 0.000031, 0.007836, -0.004015};
    std::vector<double> rightArmInit(&rightArmValues[0], &rightArmValues[0]+sizeof (rightArmValues)); //teoSim (+6) teo (+7)
    std::vector<double> leftArmInit(&leftArmValues[0], &leftArmValues[0]+sizeof (leftArmValues));

    // Resulting paths:
    std::vector<std::vector<double> > rightArmPath(points, std::vector<double>(6));
    std::vector<std::vector<double> > leftArmPath(points, std::vector<double>(6));

    // Getting fordward kinematic
    std::vector<double> rightArmOrig(6);
    if(! getRightArmFwdKin(&rightArmOrig))
        printf("[ERROR] Doing Forward Kinematic of right-arm...\n");

    std::vector<double> leftArmOrig(6);
    if(! getLeftArmFwdKin(&leftArmOrig))
        printf("[ERROR] Doing Forward Kinematic of left-arm...\n");

    // Interpolation: nos devolverá un vector (path) con todos los vectores de poses
    rightArmPath = interpolate(rightArmOrig, rightArmInit, points);
    leftArmPath = interpolate(leftArmOrig, leftArmInit, points);

    std::vector<double> rightArmCurrentQ(numRightArmJoints);
    std::vector<double> rightArmDesireQ(numRightArmJoints);

    std::vector<double> leftArmCurrentQ(numRightArmJoints);
    std::vector<double> leftArmDesireQ(numRightArmJoints);

    // MAIN LOOP
    for(int i=0; i<points; i++){

        // initialize to 0
        rightArmCurrentQ.clear();
        rightArmDesireQ.clear();
        leftArmCurrentQ.clear();
        leftArmCurrentQ.clear();

        // -- Right-Arm
        // currentQ: pequeña ayuda que le das al solver sobre la posición inicial en posicion articular
        if ( ! rightArmIEncoders->getEncoders( rightArmCurrentQ.data() ) ){
            printf("[ERROR] Failed getEncoders of right-arm\n");
            return false;
        }
        // inverse kinematic
        if ( ! rightArmICartesianSolver->invKin(rightArmPath[i], rightArmCurrentQ, rightArmDesireQ) )    {
            printf("[ERROR] invKin failed.\n");
            return false;
        }

        // -- Left-Arm
        if ( ! leftArmIEncoders->getEncoders( leftArmCurrentQ.data() ) ){
            printf("[ERROR] Failed getEncoders of left-arm\n");
            return false;
        }
        // inverse kinematic
        if ( ! leftArmICartesianSolver->invKin(leftArmPath[i], leftArmCurrentQ, leftArmDesireQ) )    {
            printf("[ERROR] invKin failed.\n");
            return false;
        }

        std::vector<double> rightArm(&rightArmDesireQ[0], &rightArmDesireQ[0]+sizeof (rightArmDesireQ)); //teoSim (+6) teo (+7)
        std::vector<double> leftArm(&leftArmDesireQ[0], &leftArmDesireQ[0]+sizeof (leftArmDesireQ));
        moveJointsInPositionDirect(rightArm, leftArm);

        yarp::os::Time::delay(delay);
    }
    return true;
}

void BalanceTray::preparePosition(){
    // Prepare the last position        
        CD_INFO("Preparing position...\n");
        configArmsToPosition(25,25);
        double rightArmPoss[7] = {31.283699, -14.04759, -10.804402, 60.000894, -72.5, 88.479388};
        double leftArmPoss[7] = {-31.283699, 14.04759, 10.804402, -60.000894, 72.5, -88.479388};
        std::vector<double> rightArm(&rightArmPoss[0], &rightArmPoss[0]+7); //teoSim (+6) teo (+7)
        std::vector<double> leftArm(&leftArmPoss[0], &leftArmPoss[0]+7);
        moveJointsInPosition(rightArm, leftArm);
        configArmsToPositionDirect();
        printf("Press a Key to move -Y");
        getchar();
        moveTrayLinearlyInPosDirect(0, -0.06, 50, 0.05);
        showArmsFKinAAS();

        while(1){

            getchar();
            printf("turning -0.04\n");
            rotateTrayInPosDirect(0, -0.04, 10, 0.05); // -0.0488

            getchar();
            printf("turning 0.04\n");
            rotateTrayInPosDirect(0, 0.04, 10, 0.05);

            getchar();
            printf("turning 0.04\n");
            rotateTrayInPosDirect(0, 0.04, 10, 0.05);

            getchar();
            printf("turning -0.04\n");
            rotateTrayInPosDirect(0, -0.04, 10, 0.05);
       }

}

// FK in AXIS_ANGLE_SCALED
void BalanceTray::showArmsFKinAAS(){
    printf("R-arm pose : [");
    std::vector<double> rightArmPoint(6);
    if(! getRightArmFwdKin(&rightArmPoint))
        CD_ERROR("Doing Forward Kinematic of right-arm...\n");
    for(int i=0; i<rightArmPoint.size(); i++)
        printf("%f ",rightArmPoint[i]);
    printf("]\n ");

    printf("L-arm pose: [");
    std::vector<double> leftArmPoint(6);
    if(! getLeftArmFwdKin(&leftArmPoint))
        CD_ERROR("Doing Forward Kinematic of left-arm...\n");
    for(int i=0; i<leftArmPoint.size(); i++)
        printf("%f ",leftArmPoint[i]);
    printf("]\n ");
}

// FK in AXIS_ANGLE
void BalanceTray::showArmsFKinAA(){
    std::vector<double> rightArmPoint(6);
    if(! getRightArmFwdKin(&rightArmPoint))
        CD_ERROR("Doing Forward Kinematic of right-arm...\n");
    std::vector<double> rightArmPointInAxisAngle(7); // axis angle
    KinRepresentation::decodePose(rightArmPoint, rightArmPointInAxisAngle, KinRepresentation::CARTESIAN, KinRepresentation::AXIS_ANGLE, KinRepresentation::DEGREES );
    printf("R-arm pose: [");
    for(int i=0; i<rightArmPointInAxisAngle.size(); i++)
        printf("%f ",rightArmPointInAxisAngle[i]);
    printf("]\n ");

    std::vector<double> leftArmPoint(6);
    if(! getLeftArmFwdKin(&leftArmPoint))
        CD_ERROR("Doing Forward Kinematic of left-arm...\n");
    std::vector<double> leftArmPointInAxisAngle(7); // axis angle
    KinRepresentation::decodePose(leftArmPoint, leftArmPointInAxisAngle, KinRepresentation::CARTESIAN, KinRepresentation::AXIS_ANGLE, KinRepresentation::DEGREES );
    printf("L-arm pose: [");
    for(int i=0; i<leftArmPointInAxisAngle.size(); i++)
        printf("%f ",leftArmPointInAxisAngle[i]);
    printf("]\n ");
}

void BalanceTray::run()
{

}

}  // namespace teo

