//
// Created by Andrew Ward on 11/12/22.
//

#ifndef PRE_KAYAK_RC_CLIENT_SHARED_YAK_MESSAGING_H
#define PRE_KAYAK_RC_CLIENT_SHARED_YAK_MESSAGING_H
#include <vector>


typedef enum gear { NEUTRAL, FORWARD, REVERSE } GEAR_t;

typedef enum { YAK=0xA1, COMMS=0xB1 } MSG_t;
typedef enum { PING=0, MOTOR_CONTROL=1, BATTERY_CHECK=2, BATTERY_MESSAGING=3, NEW_CLIENT_MAC=4, BLUETOOTH_PAIR=5 } ACTION_t;
typedef enum { SEND_OK=0x10000, SEND_FAIL=0x20000, RESPONSE_OK=0x40000, RESPONSE_FAIL=0x80000 } ACTION_RESULT_t;

typedef struct _yakMessage {
    MSG_t       msgType;   // uniqueify it a bit  ("YAK")
    uint32_t    msgID;
    ACTION_t    action;
    GEAR_t      gear;
    uint8_t     speed;  // (0-100)
} yakMessage_t;

// doesn't work since the value is not the position:   std::vector<String> MSG_t_v { "YAK" };
std::vector<String> ACTION_t_v { "PING", "MOTOR_CONTROL", "BATTERY_CHECK", "BATTERY_MESSAGING", "NEW_CLIENT_MAC", "BLUETOOTH_PAIR"};
std::vector<String> GEAR_t_v { "NEUTRAL", "FORWARD", "REVERSE" };


typedef enum { CONNECTED } YAK_COMMS_t;

typedef struct _yakCommsMessage {
    MSG_t       msgType;   // uniqueify it a bit  ("COMMS")
    uint32_t    msgID;
    YAK_COMMS_t commState;
    uint32_t    rssi;
} yakCommsMessage_t;


#endif //PRE_KAYAK_RC_CLIENT_SHARED_YAK_MESSAGING_H
