//
// Created by andrea on 22/04/16.
//

#ifndef COMMON_H
#define COMMON_H



namespace common
{

    enum states
    {
        PARAMETERS,
        PRE_TAKE_OFF,
        TAKE_OFF,
        MOVE,
        ROTATE,
        LAND,
        IDLE
    };

    enum fail_states
    {
        STOP,
        RECOVERY,
        KEEP_ON,
        GO_TO,
        LAND_F
    };


}


#endif //COMMON_H
