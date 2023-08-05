// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __BALANCE_THREAD_HPP__
#define __BALANCE_THREAD_HPP__

#include <mutex>
#include <vector>

#include <yarp/os/PeriodicThread.h>

#include <yarp/dev/IEncoders.h>
#include <yarp/dev/IPositionDirect.h>
#include <yarp/dev/IPositionControl.h>

#include <ICartesianSolver.h>

class BalanceThread : public yarp::os::PeriodicThread
{
public:
    BalanceThread(yarp::dev::IEncoders *iEncoders,
                  roboticslab::ICartesianSolver *iCartesianSolver,
                  yarp::dev::IPositionDirect *iPositionDirect,
                  int period)
        : yarp::os::PeriodicThread(period * 0.001),
          iEncoders(iEncoders),
          iCartesianSolver(iCartesianSolver),
          iPositionDirect(iPositionDirect),
          axes(0)
    {}

    void setCartesianPosition(const std::vector<double> & position)
    {
       std::lock_guard lock(positionMutex);
       this->position = position;
    }

    void getCartesianPosition(std::vector<double> *position) const
    {
        std::lock_guard lock(positionMutex);
        *position = this->position;
    }

protected:
    bool threadInit() override;
    void run() override;

private:
    mutable std::mutex positionMutex;
    yarp::dev::IEncoders *iEncoders;
    roboticslab::ICartesianSolver *iCartesianSolver;
    yarp::dev::IPositionDirect *iPositionDirect;
    std::vector<double> position;
    int axes;
};

#endif  // __BALANCE_THREAD_HPP__
