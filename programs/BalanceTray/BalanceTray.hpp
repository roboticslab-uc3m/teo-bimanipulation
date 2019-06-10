// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "StaticLibrary.hpp"
#include <yarp/os/all.h>
#include <yarp/dev/all.h>
#include <yarp/dev/IAnalogSensor.h>
#include "ColorDebug.h"
#include "ICartesianSolver.h"
#include <ICartesianTrajectory.hpp>
#include <KdlTrajectory.hpp>
#include "KinematicRepresentation.hpp"

#include "TrajectoryThread.hpp"
#include "BalanceThread.hpp"
#include <yarp/os/Semaphore.h>


#define DEFAULT_ROBOT "teo" // teo or teoSim
#define PT_MODE_MS 50 // default 50 ms (funciona bien con 25.0)
#define INPUT_READING_MS 10

// ini configuration files for kinematic
#define RIGHTARM_KIN "/usr/local/share/teo-configuration-files/contexts/kinematics/fixedTrunk-rightArm-fetch-kinematics.ini"
#define LEFTARM_KIN "/usr/local/share/teo-configuration-files/contexts/kinematics/fixedTrunk-leftArm-fetch-kinematics.ini"

using namespace yarp::os;
using namespace roboticslab;

namespace teo
{

/**
 * @ingroup teo-bimanipulation_programs
 *
 * @brief Balance Tray Core.
 *
 */
   class BalanceTray : public yarp::os::RFModule, public yarp::os::RateThread
    {        
        public:
        BalanceTray() :  yarp::os::RateThread(INPUT_READING_MS) {} // constructor
        virtual bool configure(yarp::os::ResourceFinder &rf);

        private:

            /** robot used (teo/teoSim) **/
            std::string robot;

            /** control mode: jr3/keyboard **/
            bool useJr3;

            /** RFModule interruptModule. */
            virtual bool interruptModule();
            /** RFModule getPeriod. */
            virtual double getPeriod();
            /** RFModule updateModule. */
            virtual bool updateModule();

            /*-- Right Arm Device --*/
            /** Axes number **/
            int numLeftArmJoints;
            /** Device **/
            yarp::dev::PolyDriver rightArmDevice;
            /** Encoders **/
            yarp::dev::IEncoders *rightArmIEncoders;
            /** Right Arm ControlMode2 Interface */
            yarp::dev::IControlMode *rightArmIControlMode;
            /** Right Arm PositionControl2 Interface */
            yarp::dev::IPositionControl *rightArmIPositionControl;
            /** Right Arm PositionDirect Interface */
            yarp::dev::IPositionDirect *rightArmIPositionDirect;
            /** Right Arm ControlLimits2 Interface */
            yarp::dev::IControlLimits *rightArmIControlLimits;
            /** Right Arm RemoteVariables **/
            yarp::dev::IRemoteVariables *rightArmIRemoteVariables;

            /** Solver device **/
            yarp::dev::PolyDriver rightArmSolverDevice;
            ICartesianSolver *rightArmICartesianSolver;
            /** Thread of right-arm KDL trajectory generator **/
            TrajectoryThread *rightArmTrajThread;
            /** Thread of right-arm Point2Point movement **/
            BalanceThread *rightArmBalThread;
            /** Forward Kinematic function **/
            bool getRightArmFwdKin(std::vector<double> *currentX);

            /*-- Left Arm Device --*/
            /** Axes number **/
            int numRightArmJoints;
            /** Device **/
            yarp::dev::PolyDriver leftArmDevice;
            /** Encoders **/
            yarp::dev::IEncoders *leftArmIEncoders;
            /** Left Arm ControlMode2 Interface */
            yarp::dev::IControlMode *leftArmIControlMode;
            /** Left Arm PositionControl2 Interface */
            yarp::dev::IPositionControl *leftArmIPositionControl;
            /** Left Arm PositionDirect Interface */
            yarp::dev::IPositionDirect *leftArmIPositionDirect;
            /** Left Arm ControlLimits2 Interface */
            yarp::dev::IControlLimits *leftArmIControlLimits;
            /** Left Arm RemoteVariables **/
            yarp::dev::IRemoteVariables *leftArmIRemoteVariables;

            /** Solver device **/
            yarp::dev::PolyDriver leftArmSolverDevice;
            ICartesianSolver *leftArmICartesianSolver;
            /** Thread of left-arm KDL trajectory generator **/
            TrajectoryThread *leftArmTrajThread;
            /** Thread of left-arm Point2Point movement **/
            BalanceThread *leftArmBalThread;
            /** Forward Kinematic function **/
            bool getLeftArmFwdKin(std::vector<double> *currentX);

            /** JR3 device **/
            yarp::dev::PolyDriver jr3card;
            yarp::dev::IAnalogSensor *iAnalogSensor;
            yarp::sig::Vector sensorValues;

            /** Reference position functions **/
            std::vector<double> rightArmRefpos;
            std::vector<double>  leftArmRefpos;            
            bool setRefPosition(std::vector<double> rx, std::vector<double> lx);
            bool getRefPosition(std::vector<double> *rx, std::vector<double> *lx);
            std::vector<double> rightArmHomepos;
            std::vector<double>  leftArmHomepos;
            bool homePosition(); // initial pos
            bool setHomePosition(std::vector<double> rx, std::vector<double> lx);
            bool getHomePosition(std::vector<double> *rx, std::vector<double> *lx);

            /****** FUNCTIONS ******/            

            /** Execute trajectory using a thread and KdlTrajectory**/
            bool executeTrajectory(std::vector<double> rx, std::vector<double> lx, std::vector<double> rxd, std::vector<double> lxd, double duration, double maxvel);

            /** Configure functions **/
            bool configArmsToPosition(double sp, double acc);
            bool configArmsToPositionDirect();

            /** Modes to move the joins **/
            bool moveJointsInPosition(std::vector<double> &rightArm, std::vector<double>& leftArm);
            bool moveJointsInPositionDirect(std::vector<double> &rightArm, std::vector<double> &leftArm);


            /** Moving the tray calculating a trajectory **/
            bool rotateTrayByTrajectory(int axis, double angle, double duration, double maxvel);

            /** calculate next point in relation to the forces read by the sensor **/
            bool calculatePointFollowingForce(yarp::sig::Vector sensor, std::vector<double> *rdx, std::vector<double> *ldx);
            bool calculatePointOpposedToForce(yarp::sig::Vector sensor, std::vector<double> *rdx, std::vector<double> *ldx);
            bool calculatePointPressingKeyboard(std::vector<double> *rdx, std::vector<double> *ldx);


            /** Check movements functions */
            void checkLinearlyMovement();

            /** Show information **/
            void printFKinAAS();
            void printFKinAA();
            void printJr3(yarp::sig::Vector values);

            /** movement finished */
            bool done;

            /** Input port from dialogue manager */
            yarp::os::RpcServer inDialogPort;

            /** Thread run */
            virtual bool threadInit();
            virtual void run();


     }; // class BalanceTray
}
