// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __FOLLOW_ME_ARM_SWING_HPP__
#define __FOLLOW_ME_ARM_SWING_HPP__

#include <yarp/os/all.h>
#include <yarp/dev/all.h>
#include <fstream>
#include "ICartesianSolver.h"

#define DEFAULT_ROBOT "/teoSim" // teo or teoSim


using namespace yarp::os;

namespace teo
{

/**
 * @ingroup teo-self-presentation_programs
 *
 * @brief Body Execution Core.
 *
 */
class BodyExecution : public yarp::os::RFModule, public yarp::os::PortReader, public yarp::os::Thread
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

     /** Left Arm Device */
     yarp::dev::PolyDriver leftArmDevice;
     /** Encoders **/
     yarp::dev::IEncoders *leftArmIEncoders;
     /** Left Arm ControlMode2 Interface */
     yarp::dev::IControlMode2 *leftArmIControlMode2;
     /** Left Arm PositionControl2 Interface */
     yarp::dev::IPositionControl2 *leftArmIPositionControl2;
     /** Solver device **/
     yarp::dev::PolyDriver leftArmSolverDevice;
     roboticslab::ICartesianSolver *leftArmICartesianSolver;
     /** Forward Kinematic function **/
     bool getLeftArmFwdKin(std::vector<double> *currentX);

     /** Right Arm Device */
     yarp::dev::PolyDriver rightArmDevice;
     /** Encoders **/
     yarp::dev::IEncoders *rightArmIEncoders;
     /** Right Arm ControlMode2 Interface */
     yarp::dev::IControlMode2 *rightArmIControlMode2;
     /** Right Arm PositionControl2 Interface */
     yarp::dev::IPositionControl2 *rightArmIPositionControl2;
     /** Solver device **/
     yarp::dev::PolyDriver rightArmSolverDevice;
     roboticslab::ICartesianSolver *rightArmICartesianSolver;
     /** Forward Kinematic function **/
     bool getRightArmFwdKin(std::vector<double> *currentX);

     /** Head Device */
     yarp::dev::PolyDriver headDevice;
     /** Head ControlMode2 Interface */
     yarp::dev::IControlMode2 *headIControlMode2;
     /** Head PositionControl2 Interface */
     yarp::dev::IPositionControl2 *headIPositionControl2;

     /** Trunk Device */
     yarp::dev::PolyDriver trunkDevice;
     /** Trunk ControlMode2 Interface */
     yarp::dev::IControlMode2 *trunkIControlMode2;
     /** Trunk PositionControl2 Interface */
     yarp::dev::IPositionControl2 *trunkIPositionControl2;



     /** Arm Joints Move And Wait */
     bool jointsMoveAndWait(std::vector<double>& leftArm, std::vector<double>& rightArm, std::vector<double> &head);

     std::vector<double> interpolate(double pta, double ptb, int res);
     bool moveTrayLinearly(int axis, double dist);

     /** State */
     int state;

     /** movement finished */
     bool done;

     /** Input port from dialogue manager */
     yarp::os::RpcServer inDialogPort;

     /** Treats data received from input port from speech recognition */
     virtual bool read(yarp::os::ConnectionReader& connection);

     /** Thread run */
     virtual void run();

};

}  // namespace teo

#endif  // __FOLLOW_ME_ARM_SWING_HPP__
