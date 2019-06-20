// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "DialogueManager.hpp"

namespace teo
{

/************************************************************************/

DialogueManager::DialogueManager(std::string language) {

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
    sentence = 0;

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
    }
}

/************************************************************************/

void DialogueManager::talkTrayStatus(yarp::sig::Vector sensor, std::vector<double> rdsxaa, std::vector<double> ldsxaa){
    printf("[%f] [%f]\n", rdsxaa[6], ldsxaa[6]);

    // centrado entre sensor[13] y sensor[19]
    if ( (std::abs(sensor[13])>0.1 && std::abs(sensor[13])<0.2)  && (std::abs(sensor[19])>0.1  && std::abs(sensor[19])<0.2))
        switch(sentence)
        {
        case 0:
            ttsSay("mis sensores me dicen que el objeto esta centrado");
            break;
        case 1:
            ttsSay("lo tengo, al parecer soy bueno en esto");
            break;
        case 2:
            ttsSay("ahi esta de nuevo, soy el mejor en esto");
            break;
        case 3:
            ttsSay("si, si, si, lo consegui. Soy un crack, jaajaajaajaajaaa");
            break;
        case 4:
            ttsSay("ahi lo tienes, no creo que tu fueses capaz de hacerlo mejor");
            break;
        case 5:
            ttsSay("ya la tienes de nuevo centrada");
            break;
        case 6:
            ttsSay("en el centro de la diana, no puedo ser mejor en esto");
            break;
        }

    else{
        // deteccion parte izquierda de la bandeja
        if(sensor[13] > 0.06 && std::abs(sensor[13])>std::abs(sensor[19]))
            switch(sentence)
            {
            case 0:
                ttsSay("detecto presion en la parte izquierda");
                break;
            case 1:
                ttsSay("necesito girarla un poco a mi izquierda.");
                break;
            case 2:
                ttsSay("se me esta escapando del centro");
                break;
            case 3:
                ttsSay("la tengo muy a derecha");
                break;
            case 4:
                ttsSay("uy uy uy ... se me va a la derecha");
                break;
            case 5:
                ttsSay("no no y no ... otra vez a la derecha");
                break;
            case 6:
                ttsSay("otra vez se me van a la derecha ... me estoy poniendo nervioso");
                break;
            }

        // deteccion parte derecha de la bandeja
        else if(sensor[19] < -0.06 && std::abs(sensor[19])>std::abs(sensor[13]))
            switch(sentence)
            {
            case 0:
                ttsSay("detecto presion en la parte derecha");
                break;
            case 1:
                ttsSay("necesito girarla un poco a mi izquierda.");
                break;
            case 2:
                ttsSay("necesito girarla un poco a mi derecha.");
                break;
            case 3:
                ttsSay("parece que no le gusta irse a la derecha");
                break;
            case 4:
                ttsSay("lo que me faltaba... se me va a la izquierda");
                break;
            case 5:
                ttsSay("uy uy uy ... a la izquierda no quiero que te vayas");
                break;
            case 6:
                ttsSay("estoy concentrandome para moverla a la derecha");
                break;
            case 7:
                ttsSay("mentalizate Teo... tengo que moverla a la derecha");
                break;
            }

        // deteccion parte atras de la bandeja
        if((sensor[17] < -0.08) || (sensor[23] > +0.08))
            switch(sentence)
            {
            case 0:
                ttsSay("detecto presion en la parte atras de la bandeja");
                break;
            case 1:
                ttsSay("necesito girarla un poco hacia adelante");
                break;
            case 2:
                ttsSay("parece que la tengo que mover hacia adelante");
                break;
            case 3:
                ttsSay("la detecto demasiado atras, voy a moverla hacia adelante");
                break;
            case 4:
                ttsSay("uy uy uy ... otra vez atras no me gusta");
                break;
            case 5:
                ttsSay("estoy concentrandome para moverla hacia adelante");
                break;
            case 6:
                ttsSay("adelante, vamos, adelante");
                break;
            }

        // deteccion parte alante de la bandeja
        else if((sensor[17] > 0.08) || (sensor[23] < -0.08))
            switch(sentence)
            {
            case 0:
                ttsSay("detecto presion en la parte adelante de la bandeja");
                break;
            case 1:
                ttsSay("necesito girarla un poco hacia atras");
                break;
            case 2:
                ttsSay("parece que la tengo que mover hacia atras");
                break;
            case 3:
                ttsSay("esta demasiado arriba. voy a moverla un poco hacia atras");
                break;
            case 4:
                ttsSay("detecto presion en la parte delantera de la bandeja");
                break;
            case 5:
                ttsSay("estoy concentrandome para moverla hacia atras");
                break;
            case 6:
                ttsSay("un poquito hacia atras seria estupendo");
                break;
            }

        if(rdsxaa[6]>4.9 || ldsxaa[6]>4.9)
            ttsSay("esto esta demasiado torcido");

        else if ((rdsxaa[6]<4.9 && rdsxaa[6]>3) || (ldsxaa[6]<4.9 && ldsxaa[6]>3))
            ttsSay("casi lo tengo, necesito concentracion");

        else if ((rdsxaa[6]<3 && rdsxaa[6]>2) || (ldsxaa[6]<3 && ldsxaa[6]>2))
            ttsSay("esta muy cerca, solo un poco más");
    }

    if(sentence<7) sentence++;
    else sentence = 0;

    return;
}

/************************************************************************/

void DialogueManager::ttsSay(std::string sayString) {

    yarp::os::Bottle bOut, bRes;
    bOut.addString("say");
    bOut.addString(sayString);
    outTtsPort.write(bOut,bRes);
    printf("[DialogueManager] Teo said: %s [%s]\n", sayString.c_str(), bRes.toString().c_str());
    yarp::os::Time::delay(0.5);
}

}  // namespace teo
