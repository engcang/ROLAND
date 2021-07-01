#include "main.h"


int main(int argc, char **argv)
{
    ros::init(argc, argv, "ekf_land_node");
    ros::NodeHandle n("~");

    ekf_land ekfland(n);

    signal(SIGINT, signal_handler); // to exit program when ctrl+c

    ros::AsyncSpinner spinner(6); // Use 6 threads -> 5 callbacks + 1 Timer callbacks
    spinner.start();
    ros::waitForShutdown();

    return 0;
}