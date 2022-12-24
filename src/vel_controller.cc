//
// Created by mofei on 2022/12/10.
//

#include <gazebo/gazebo_config.h>
#include <geometry_msgs/Twist.h>

#include <gazebo/gazebo_client.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>

/////////////////////////////////////////////////
int main(int _argc, char** _argv) {
    gazebo::client::setup(_argc, _argv);

    // Create our node for communication
    gazebo::transport::NodePtr node(new gazebo::transport::Node());
    node->Init();

    // Publish to my topic
    gazebo::transport::PublisherPtr pub =
        node->Advertise<gazebo::msgs::Int>("/my_suv_controller/vel_cmd");

    // Wait for a subscriber to connect to this publisher
    pub->WaitForConnection();

    int c;
    std::cout << "Q to quit, w/a/s/d to move, q/e to rotate. "
                 "type Enter each time."
              << std::endl;
    while (true) {
        c = getchar();
        if (c == 'Q')
            break;
        if (c == '\n')
            continue;
        //		char *data = &c;
        gazebo::msgs::Int msg;
        msg.set_data(c);
        pub->Publish(msg);
        //        std::cout << "sent" << std::endl;
    }

    gazebo::client::shutdown();
}