#include <rag2_trajectory/GoalInfo.h>
#include <ros/ros.h>

void cb(const rag2_trajectory::GoalInfoConstPtr & msg)
{

}

int main(int ac, char **av)
{
    ros::init(ac, av, "moveit_to_mcu_node");

    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe<rag2_trajectory::GoalInfo>("/goal_pub", 10, cb);

    while (ros::ok())
    {
    }
}