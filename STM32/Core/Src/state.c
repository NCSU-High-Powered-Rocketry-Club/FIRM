#include "state.h"

static State current_state = STANDBY;

typedef struct {
    void (*update)(void);
    void (*next_state)(void);
} StateHandlers;

void next_state_standby(void) {
    current_state = MOTOR_BURN;
}
void next_state_motor_burn(void) {
    current_state = COAST;
}
void next_state_coast(void) {
    current_state = FREE_FALL;
}
void next_state_free_fall(void) {
    current_state = LANDED;
}
void next_state_landed(void) {
    serialPrintStr("cannot go to next state");
}



void update_standby(void) {
    serialPrintStr("standby");
}
void update_motor_burn(void) {
    serialPrintStr("motor burn");
}
void update_coast(void) {
    serialPrintStr("coast");
}
void update_free_fall(void) {
    serialPrintStr("freefall");
}
void update_landed(void) {
    serialPrintStr("landed");
}



static const StateHandlers stateTable[NUM_STATES] = {
    [STANDBY] = {update_standby, next_state_standby},
    [MOTOR_BURN] = {update_motor_burn, next_state_motor_burn},
    [COAST] = {update_coast, next_state_coast},
    [FREE_FALL] = {update_free_fall, next_state_free_fall},
    [LANDED] = {update_landed, next_state_landed},
};

void update_state() {
    stateTable[current_state].update();
}

