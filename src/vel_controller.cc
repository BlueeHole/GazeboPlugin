//
// Created by mofei on 2022/12/10.
//

#include <gazebo/gazebo_config.h>
#include <geometry_msgs/Twist.h>

#include <gazebo/gazebo_client.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>

/////////////////////////////////////////////////
int main(int _argc, char **_argv) {
	gazebo::client::setup(_argc, _argv);

	// Create our node for communication
	gazebo::transport::NodePtr node(new gazebo::transport::Node());
	node->Init();

	// Publish to my topic
	gazebo::transport::PublisherPtr pub =
		node->Advertise<gazebo::msgs::GzString>("/my_suv_controller/vel_cmd");

	// Wait for a subscriber to connect to this publisher
	pub->WaitForConnection();

	char c;
	while (true) {
		c = getchar();
		if (c == 'q') break;
		char *data = &c;
		gazebo::msgs::GzString msg;
		msg.set_data(data);
		pub->Publish(msg);
		std::cout << "sent" << std::endl;
	}
	//	gazebo::msgs::Twist msg;
	//	gazebo::msgs::Vector3d linear;
	//	linear.set_x(1);
	//	linear.set_y(0);
	//	linear.set_z(0);
	//	msg.set_allocated_linear(&linear);

	gazebo::client::shutdown();
}