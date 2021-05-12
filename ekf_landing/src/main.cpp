#include "main.h"


int main(int argc, char **argv)
{
    ros::init(argc, argv, "ekf_land_node");
    ros::NodeHandle n("~");

    ekf_land ekfland(n);

    signal(SIGINT, signal_handler); // to exit program when ctrl+c

    ros::AsyncSpinner spinner(3); // Use 7 threads -> 3 callbacks + 0 Timer callbacks
    spinner.start();
    ros::waitForShutdown();

    return 0;
}