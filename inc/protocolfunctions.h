#pragma once

#include "protocol.h"

int setup_protocol();


extern PROTOCOL_STAT sSoftwareSerial;
extern PROTOCOL_STAT sUSART2;
extern PROTOCOL_STAT sUSART3;


//////////////////////////////////////////
// structures exclusively used in protocol

#pragma pack(push, 4)  // all used data types are 4 byte
typedef struct tag_POSN_DATA {
    // these get set
    long wanted_posn_mm[2];

    // configurations/constants
    int posn_max_speed; // max speed in this mode
    int posn_min_speed; // minimum speed (to get wheels moving)

    // just so it can be read back
    long posn_diff_mm[2];
    long posn_speed_demand[2];
} POSN_DATA;
#pragma pack(pop)

extern POSN_DATA PosnData;


#pragma pack(push, 4)  // all used data types are 4 byte
typedef struct tag_SPEED_DATA {
    // these get set
    long wanted_speed_mm_per_sec[2];

    // configurations/constants
    int speed_max_power; // max speed in this mode
    int speed_min_power; // minimum speed (to get wheels moving)
    int speed_minimum_speed; // below this, we don't ask it to do anything

    // just so it can be read back
    long speed_diff_mm_per_sec[2];
    long speed_power_demand[2];
} SPEED_DATA;
#pragma pack(pop)

extern SPEED_DATA SpeedData;


#pragma pack(push, 4)  // all used data types are 4 byte
typedef struct tag_PWM_DATA {
    // these get set
    long pwm[2];

    // configurations/constants
    int speed_max_power; // max speed in this mode
    int speed_min_power; // minimum speed (to get wheels moving)
    int speed_minimum_pwm; // below this, we don't ask it to do anything
} PWM_DATA;
#pragma pack(pop)

extern PWM_DATA PWMData;


#pragma pack(push, 4)  // all used data types are 4 byte
typedef struct tag_POSN {
    long LeftAbsolute;
    long RightAbsolute;
    long LeftOffset;
    long RightOffset;
} POSN;
#pragma pack(pop)

#pragma pack(push, 4)  // all used data types are 4 byte
typedef struct tag_POSN_INCR {
    long Left;
    long Right;
} POSN_INCR;
#pragma pack(pop)

#pragma pack(push, 1)
typedef struct {
    uint8_t buzzerFreq;
    uint8_t buzzerPattern;
    uint16_t buzzerLen;
} BUZZER_DATA;
#pragma pack(pop)

