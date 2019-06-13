// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __DIALOGUE_MANAGER_HPP__
#define __DIALOGUE_MANAGER_HPP__

#include <cmath>
#include <yarp/os/all.h>
#include <yarp/dev/all.h>
#include <ColorDebug.h>

#define DEFAULT_LANGUAGE "spanish"

namespace teo
{

/**
 * @ingroup BalanceTray_programs
 *
 * @brief Dialogue Manager.
 */
class DialogueManager
{
    public:

    // constructor
      DialogueManager(std::string language);

      void talkTrayStatus(yarp::sig::Vector sensor, std::vector<double> rdsxaa, std::vector<double> ldsxaa);
      void ttsSay(std::string sayString);

    private:
        yarp::os::RpcClient outTtsPort; // tts port

};

}  // namespace teo

#endif  // __DIALOGUE_MANAGER_HPP__
