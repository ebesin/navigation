std_msgs/Header header
string tm

# Powertrain
int32 drivemode
int32 steeringmode
int32 MODE_MANUAL=0
int32 MODE_AUTO=1

mpc_msgs/Gear current_gear

float64 speed # vehicle velocity [km/s]
int32 drivepedal
int32 brakepedal

float64 angle # vehicle steering (tire) angle [rad]

# Body
int32 lamp
int32 LAMP_LEFT=1
int32 LAMP_RIGHT=2
int32 LAMP_HAZARD=3

int32 light
