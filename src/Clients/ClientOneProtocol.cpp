#include "ClientOneProtocol.h"

using namespace std;

namespace AeroRobot {

void writeDouble6(ostream & os, const string & pre, double * data)
{
    os << pre;
    for(size_t i = 0; i < 6; ++i){
        os << " " << data[i];
    }
}

ostream & operator<<(ostream & os, RobotStatus & data)
{
    os << "AxisNum: " << data.axisNum;
    writeDouble6(os, " CurrentJoint: ", data.currentJoint);
    writeDouble6(os, " CurrentPoseEuler: ", data.currentPoseEuler);
    writeDouble6(os, " CurrentPoseEulerTCP: ", data.currentPoseEulerTCP);
    os << " JogSpeedJoint: " << data.JogSpeedJoint;
    os << " JogSpeedPose: " << data.JogSpeedPose;
    os << " JogSpeedEuler: " << data.JogSpeedEuler;
    os << " JogInchingPose: " << data.JogInchingPose;
    os << " JogInchingEuler: " << data.JogInchingEuler;
    writeDouble6(os, " CurrentSpeedJoint: ", data.currentSpeedJoint);
    writeDouble6(os, " CurrentAmpereJoint: ", data.currentAmpereJoint);

    return os;
}

}

