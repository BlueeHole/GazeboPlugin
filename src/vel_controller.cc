//
// Created by mofei on 2022/12/10.
//

#include <ros/ros.h>
#include <std_msgs/Int8.h>
#include <stdio.h>

/////////////////////////////////////////////////
int main(int argc, char** argv) {
    // Create our node for communication
    ros::init(argc, argv, "controller_mf");
    ros::NodeHandle nh;

    // Publish to my topic
    ros::Publisher pub = nh.advertise<std_msgs::Int8>("/my_suv_controller/vel_cmd", 1);

    char c;
    std::cout << "Q to quit, w,s加减线速度；a,d加减角速度"
                 "type Enter each time."
              << std::endl;
	system("stty raw");   /*使终端驱动处于一次一字符模式*/
    while (true) {
        c = getchar();
        if (c == 'q')
            break;
        if (c == '\n')
            continue;
        std_msgs::Int8 msg;
        msg.data = c;
        pub.publish(msg);
        //        std::cout << "sent" << std::endl;
    }
 	system("stty cooked");   /*使终端驱动回到一次一行模式*/
}