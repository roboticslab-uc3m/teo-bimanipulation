// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
#include <yarp/os/all.h>
#include <yarp/dev/all.h>
#include "ColorDebug.h"
#include "ICartesianSolver.h"


#define DEFAULT_ROBOT "/teoSim" // teo or teoSim

using namespace yarp::os;
namespace teo
{

/**
 * @ingroup teo-bimanipulation_programs
 *
 * @brief Balance Tray Core.
 *
 */
   class BalanceTray : public yarp::os::RFModule, public yarp::os::PortReader, public yarp::os::Thread
    {
        public:
             virtual bool configure(yarp::os::ResourceFinder &rf);
        private:

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
            yarp::dev::IControlMode2 *rightArmIControlMode2;
            /** Right Arm PositionControl2 Interface */
            yarp::dev::IPositionControl2 *rightArmIPositionControl2;
            /** Right Arm ControlLimits2 Interface */
            yarp::dev::IControlLimits *rightArmIControlLimits;
            /** Solver device **/
            yarp::dev::PolyDriver rightArmSolverDevice;
            roboticslab::ICartesianSolver *rightArmICartesianSolver;
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
            yarp::dev::IControlMode2 *leftArmIControlMode2;
            /** Left Arm PositionControl2 Interface */
            yarp::dev::IPositionControl2 *leftArmIPositionControl2;
            /** Left Arm ControlLimits2 Interface */
            yarp::dev::IControlLimits *leftArmIControlLimits;
            /** Solver device **/
            yarp::dev::PolyDriver leftArmSolverDevice;
            roboticslab::ICartesianSolver *leftArmICartesianSolver;
            /** Forward Kinematic function **/
            bool getLeftArmFwdKin(std::vector<double> *currentX);


            /** State */
            int state;

            /** movement finished */
            bool done;

            /** Input port from dialogue manager */
            yarp::os::RpcServer inDialogPort;

            /** Treats data received from input port from speech recognition */
            //virtual bool read(yarp::os::ConnectionReader& connection);

            /** Thread run */
            virtual void run();



     }; // class BalanceTray
}
