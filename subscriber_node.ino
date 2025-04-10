#include <ros.h>

#include <std_msgs/Int32.h>




// Create ROS node handle

ros::NodeHandle nh;

// ros::init(argc, argv, "subscriber");



// Callback function to handle received data

void messageCallback(const std_msgs::Int32& msg) {

  // Print received data

  Serial.print("Received: ");

  Serial.println(msg.data);

}



// Create subscriber object

ros::Subscriber<std_msgs::Int32> sub("arduino_1_topic", &messageCallback);



void setup() {

  // Initialize Serial for debug output

  Serial.begin(9600);

  

  // Initialize ROS node handle

  nh.initNode();

  nh.subscribe(sub);

}



void loop() {

  // Spin to process incoming messages
  
  
  nh.spinOnce();

  delay(100);  // Small delay for stability

}

