// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "DialogueManager.hpp"

namespace teo
{

/************************************************************************/

bool DialogueManager::configureTts(std::string language) {

    outTtsPort.open("/balanceTray/tts/rpc:c");

    /*
    while(1){
        if(outTtsPort.getOutputCount() > 0) break;
        printf("Waiting for \"/dialogueManager/tts/rpc:c\" to be connected to something...\n");
        yarp::os::Time::delay(0.5);
    }
    */

    while(!yarp::os::Network::connect("/balanceTray/tts/rpc:c","/tts/rpc:s")){
        CD_WARNING("TTS port not detected! Please, run: yarpdev --Espeak --name /tts\n");
        yarp::os::Time::delay(1);
    }

    yarp::os::Bottle bConf;
    bConf.clear();

    //-- speech configuration
    if( language == "english" )
    {

        bConf.addString("setLanguage");
        bConf.addString("mb-us2");
        outTtsPort.write(bConf);
        printf("[info] Configured bottle: %s\n", bConf.toString().c_str());
        bConf.clear();

        bConf.addString("setSpeed");
        bConf.addInt(150);
        outTtsPort.write(bConf);
        printf("[info] Configured bottle: %s\n", bConf.toString().c_str());
        bConf.clear();
    }
    else if ( language == "spanish" )
    {
        bConf.addString("setLanguage");
        bConf.addString("mb-es1");
        outTtsPort.write(bConf);
        printf("[info] Configured bottle: %s\n", bConf.toString().c_str());
        bConf.clear();

        bConf.addString("setSpeed");
        bConf.addInt(170);
        outTtsPort.write(bConf);
        printf("[info] Configured bottle: %s\n", bConf.toString().c_str());
        bConf.clear();
    }
    else
    {
        printf("Language not found. Please use '--language english' or '--language spanish'");
        return false;
    }
}

/************************************************************************/

void DialogueManager::ttsSay(std::string sayString) {

    yarp::os::Bottle bOut, bRes;
    bOut.addString("say");
    bOut.addString(sayString);
    outTtsPort.write(bOut,bRes);
    printf("[StateMachine] Said: %s [%s]\n", sayString.c_str(), bRes.toString().c_str());
    yarp::os::Time::delay(0.5);
}

}  // namespace teo
