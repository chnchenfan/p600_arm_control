#include "linux_serial/Sub_data.h"
Sub_data::Sub_data(ros::NodeHandle nh){
    updata_flag=false;
    pos_value=0;
    pos_sub=nh.subscribe<std_msgs::Float64>("/pose_data",10,&Sub_data::pos_cb, this);
}
Sub_data::~Sub_data(){

}
void Sub_data::pos_cb(const boost::shared_ptr<const std_msgs::Float64>& msg){
    pos_value=msg->data;   
    updata_flag=true;
}
