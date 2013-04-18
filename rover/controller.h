#define CONTROLLER_DIRECTION_FORWARD 0
#define CONTROLLER_DIRECTION_HALT 1
#define CONTROLLER_DIRECTION_REVERSE 2

void initializeController();

int getControllerAngle();
int getControllerDirection();
int getControllerSpeed();
bool isKillSwitch();
