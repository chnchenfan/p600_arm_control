#include <iostream>
#include <ros/ros.h>
#include <ros/time.h>
#include <ros/duration.h>
#include <std_msgs/Float64.h>
class Sub_data{
public:
    double pos_value;
    bool updata_flag; 
    Sub_data(ros::NodeHandle nh);
    ~Sub_data();
private:
    ros::Subscriber pos_sub; 
    void pos_cb(const boost::shared_ptr<const std_msgs::Float64>& msg);
};
