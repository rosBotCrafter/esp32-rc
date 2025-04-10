#include <ros.h>

#include <std_msgs/Int32.h>



// Create ROS node handle

ros::NodeHandle nh;



// Create a publisher object

std_msgs::Int32 msg;

ros::Publisher pub("arduino_1_topic", &msg);



// Data to send

int sent_value = 0;



void setup() {

  // Initialize ROS node handle
  
  Serial.begin(9600);

  nh.initNode();

  nh.advertise(pub);

}



void loop() {

  // Increment and publish integer data

  msg.data = sent_value;

  pub.publish(&msg);



  // Update data value

  sent_value++;

  if (sent_value > 100) sent_value = 0;  // Reset after 100

  
  Serial.print(sent_value);
  
  nh.spinOnce();

  delay(100);  // Delay to control publishing rate

}

