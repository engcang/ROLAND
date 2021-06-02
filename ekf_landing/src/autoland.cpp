#include "autoland.h"


int main(int argc, char **argv)
{
    ros::init(argc, argv, "autolanding_node");
    ros::NodeHandle nh("~");

    drone drone(nh);

    signal(SIGINT, signal_handler); // to exit program when ctrl+c

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(25.0);

    // wait for FCU connection
    while(ros::ok() && !drone.current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    while(ros::ok()){
        drone.run();
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}