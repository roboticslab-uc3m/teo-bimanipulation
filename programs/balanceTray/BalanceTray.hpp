// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __BALANCE_TRAY_HPP__
#define __BALANCE_TRAY_HPP__

#include <string>
#include <vector>

#include <yarp/conf/version.h>

#include <yarp/os/PeriodicThread.h>
#include <yarp/os/RFModule.h>

#include <yarp/dev/IAnalogSensor.h>
#include <yarp/dev/IEncoders.h>
#include <yarp/dev/IPositionControl.h>
#include <yarp/dev/IPositionDirect.h>
#include <yarp/dev/IControlLimits.h>
#include <yarp/dev/IControlMode.h>
#include <yarp/dev/IRemoteVariables.h>
#include <yarp/dev/PolyDriver.h>

#include <ICartesianSolver.h>
#include <KinematicRepresentation.hpp>

#include "DialogueManager.hpp"
#include "TrajectoryThread.hpp"
#include "BalanceThread.hpp"

constexpr auto DEFAULT_ROBOT = "teo"; // "teo" or "teoSim"
constexpr auto DEFAULT_MODE = "keyboard";
constexpr auto PT_MODE_MS = 50;
constexpr auto INPUT_READING_MS = 10;

namespace teo
{

/**
 * @ingroup teo-bimanipulation_programs
 * @brief Balance Tray Core.
 */
class BalanceTray : public yarp::os::RFModule,
                    public yarp::os::PeriodicThread
{
public:
    BalanceTray() : yarp::os::PeriodicThread(INPUT_READING_MS * 0.001) {} // constructor
    bool configure(yarp::os::ResourceFinder & rf) override;

    /** current vector position of the tray centroid **/
    std::vector<double> rdsxaa;
    std::vector<double> ldsxaa;

private:
    /** robot used (teo/teoSim) **/
    std::string robot;

    /** control mode: jr3/keyboard **/
    bool jr3Balance {false};
    bool testMov {false};
    bool keyboard {false};
    bool jr3ToCsv {false};

    /** with speech **/
    bool speak {false};

    /** Operating mode: jr3Balance / keyboard / jr3Check2Csv **/
    std::string mode;

    /** RFModule interruptModule. */
    bool interruptModule() override;
    /** RFModule getPeriod. */
    double getPeriod() override;
    /** RFModule updateModule. */
    bool updateModule() override;

    /*-- Right Arm Device --*/
    /** Axes number **/
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    std::size_t numLeftArmJoints {0};
#else
    int numLeftArmJoints {0};
#endif
    /** Device **/
    yarp::dev::PolyDriver rightArmDevice;
    /** Encoders **/
    yarp::dev::IEncoders * rightArmIEncoders {nullptr};
    /** Right Arm ControlMode2 Interface */
    yarp::dev::IControlMode * rightArmIControlMode {nullptr};
    /** Right Arm PositionControl2 Interface */
    yarp::dev::IPositionControl * rightArmIPositionControl {nullptr};
    /** Right Arm PositionDirect Interface */
    yarp::dev::IPositionDirect * rightArmIPositionDirect {nullptr};
    /** Right Arm ControlLimits2 Interface */
    yarp::dev::IControlLimits * rightArmIControlLimits {nullptr};
    /** Right Arm RemoteVariables **/
    yarp::dev::IRemoteVariables * rightArmIRemoteVariables {nullptr};

    /** Solver device **/
    yarp::dev::PolyDriver rightArmSolverDevice;
    roboticslab::ICartesianSolver * rightArmICartesianSolver {nullptr};
    /** Thread of right-arm KDL trajectory generator **/
    TrajectoryThread * rightArmTrajThread {nullptr};
    /** Thread of right-arm Point2Point movement **/
    BalanceThread * rightArmBalThread {nullptr};
    /** Forward Kinematic function **/
    bool getRightArmFwdKin(std::vector<double> & currentX);

    /*-- Left Arm Device --*/
    /** Axes number **/
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    std::size_t numRightArmJoints {0};
#else
    int numRightArmJoints {0};
#endif
    /** Device **/
    yarp::dev::PolyDriver leftArmDevice;
    /** Encoders **/
    yarp::dev::IEncoders * leftArmIEncoders {nullptr};
    /** Left Arm ControlMode2 Interface */
    yarp::dev::IControlMode * leftArmIControlMode {nullptr};
    /** Left Arm PositionControl2 Interface */
    yarp::dev::IPositionControl * leftArmIPositionControl {nullptr};
    /** Left Arm PositionDirect Interface */
    yarp::dev::IPositionDirect * leftArmIPositionDirect {nullptr};
    /** Left Arm ControlLimits2 Interface */
    yarp::dev::IControlLimits * leftArmIControlLimits {nullptr};
    /** Left Arm RemoteVariables **/
    yarp::dev::IRemoteVariables * leftArmIRemoteVariables {nullptr};

    /** Solver device **/
    yarp::dev::PolyDriver leftArmSolverDevice;
    roboticslab::ICartesianSolver * leftArmICartesianSolver {nullptr};
    /** Thread of left-arm KDL trajectory generator **/
    TrajectoryThread * leftArmTrajThread {nullptr};
    /** Thread of left-arm Point2Point movement **/
    BalanceThread * leftArmBalThread {nullptr};
    /** Forward Kinematic function **/
    bool getLeftArmFwdKin(std::vector<double> & currentX);

    /** JR3 device **/
    yarp::dev::PolyDriver jr3card;
    yarp::dev::IAnalogSensor * iAnalogSensor {nullptr};
    yarp::sig::Vector sensorValues;

    /** Reference position functions **/
    std::vector<double> rightArmRefpos;
    std::vector<double> leftArmRefpos;
    bool setRefPosition(const std::vector<double> & rx, const std::vector<double> & lx);
    bool getRefPosition(std::vector<double> & rx, std::vector<double> & lx);
    bool homePosition(); // initial pos

    /****** FUNCTIONS ******/

    /** Execute trajectory using a thread and KdlTrajectory**/
    bool executeTrajectory(const std::vector<double> & rx, const std::vector<double> & lx, const std::vector<double> & rxd, const std::vector<double> & lxd, double duration, double maxvel);
    bool rotateTrayByTrajectory(int axis, double angle, double duration, double maxvel);
    bool passJr3ValuesToCsv();

    /** Configure functions **/
    bool configArmsToPosition(double sp, double acc);
    bool configArmsToPositionDirect();

    /** Modes to move the joins **/
    bool moveJointsInPosition(const std::vector<double> & rightArm, const std::vector<double> & leftArm);
    bool moveJointsInPositionDirect(const std::vector<double> & rightArm, const std::vector<double> & leftArm);


    /** calculate next point in relation to the forces readed by the sensor or key pressed **/
    bool calculatePointOpposedToForce(const yarp::sig::Vector & sensor, std::vector<double> & rdx, std::vector<double> & ldx);
    bool calculatePointPressingKeyboard(std::vector<double> & rdx, std::vector<double> & ldx);


    /** Check movements functions */
    void checkLinearlyMovement();

    /** Get axis rotation of the tray **/
    bool getAxisRotation(std::vector<double> & axisRotation);

    /** Write information in CSV file **/
    FILE * fp {nullptr};
    bool writeInfo2Csv(double timeStamp, const std::vector<double> & axisRotation, const yarp::sig::Vector & jr3Values);
    // ireration
    int i {0};

    /** Show information **/
    void printFKinAAS();
    void printFKinAA();
    void printJr3(const yarp::sig::Vector & values);

    /** movement finished */
    bool done {false};

    /** Current time **/
    double initTime {0.0};

    /** Dialogue manager */
    DialogueManager * dialogueManager {nullptr};

    /** Thread run */
    bool threadInit() override;
    void run() override;
};

} // namespace teo

#endif // __BALANCE_TRAY_HPP__
