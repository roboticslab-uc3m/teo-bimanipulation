// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __DIALOGUE_MANAGER_HPP__
#define __DIALOGUE_MANAGER_HPP__

#include <string>
#include <vector>

#include <yarp/os/RpcClient.h>
#include <yarp/sig/Vector.h>

constexpr auto DEFAULT_LANGUAGE = "spanish";

namespace teo
{

/**
 * @ingroup BalanceTray_programs
 * @brief Dialogue Manager.
 */
class DialogueManager
{
public:
    // constructor
    DialogueManager(const std::string & language);

    void talkTrayStatus(yarp::sig::Vector sensor, std::vector<double> rdsxaa, std::vector<double> ldsxaa);
    void ttsSay(const std::string & sayString);

private:
    yarp::os::RpcClient outTtsPort; // tts port
    int sentence;

};

}  // namespace teo

#endif  // __DIALOGUE_MANAGER_HPP__
