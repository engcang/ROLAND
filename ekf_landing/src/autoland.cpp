#include "autoland.h"


int main(int argc, char **argv)
{
    ros::init(argc, argv, "autolanding_node");
    ros::NodeHandle n("~");

    drone drone(n);

    signal(SIGINT, signal_handler); // to exit program when ctrl+c

    ros::AsyncSpinner spinner(4); // Use 7 threads -> 3 callbacks + 0 Timer callbacks
    spinner.start();
    ros::waitForShutdown();

    return 0;
}