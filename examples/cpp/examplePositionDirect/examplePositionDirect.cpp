#include <stdio.h>
#include <stdlib.h>

#include <vector>

#include <yarp/os/all.h>
#include <yarp/dev/all.h>

#define JOINT 0


using namespace yarp::os;
using namespace yarp::dev;

int main(int argc, char *argv[]) {

    printf("WARNING: requires a running instance of OneCanBusOneWrapper\n");
    Network yarp;
    if (!Network::checkNetwork()) {
        printf("Please start a yarp name server first\n");
        return(-1);
    }
    Property options;
    options.put("device","remote_controlboard"); //remote_controlboard
    options.put("remote","/teo/rightArm"); // /wrapper0
    options.put("local","/local");

    PolyDriver rightArm(options);
    if(!rightArm.isValid()) {
      printf("RightArm device not available.\n");
      rightArm.close();
      Network::fini();
      return 1;
    }

    IPositionDirect *posdir;
    IPositionControl *pos;
    IVelocityControl *vel;
    IEncoders *enc;
    IControlMode *mode;

    bool ok = true;

    ok &= rightArm.view(posdir);
    ok &= rightArm.view(pos);
    ok &= rightArm.view(enc);
    ok &= rightArm.view(mode);

    if (!ok) {
        printf("[warning] Problems acquiring robot interface\n");
        return false;
    } else printf("[success] acquired robot interface\n");

    /** Axes number **/
    int numJoints;

    printf("-- testing POSITION MODE --\n");
    pos->getAxes(&numJoints);
    std::vector<int> positionMode(numJoints,VOCAB_CM_POSITION);
    if(! mode->setControlModes(positionMode.data())){
        printf("[warning] Problems setting position control: POSITION \n");
        return false;
    }

    printf("moving joint 0 to 10 degrees...\n");
    pos->positionMove(JOINT, 10);
    getchar();

    printf("-- testing POSITION DIRECT --\n");
    posdir->getAxes(&numJoints);
    std::vector<int> positionDirectMode(numJoints,VOCAB_CM_POSITION_DIRECT);
    if(! mode->setControlModes(positionDirectMode.data())){
        printf("[warning] Problems setting position control: POSITION_DIRECT \n");
        return false;
    }

    double encValue;
    if ( ! enc->getEncoder(JOINT,&encValue) ){
        printf("[ERROR] Failed getEncoders of right-arm\n");
        return false;
    }

    printf("Current ENC value: %f\n", encValue);

    getchar();

    while(encValue<=20)
    {
        encValue+=0.1;
        posdir->setPosition(JOINT, encValue);
        yarp::os::Time::delay(0.05); //0.05
    }

    return 0;
}
