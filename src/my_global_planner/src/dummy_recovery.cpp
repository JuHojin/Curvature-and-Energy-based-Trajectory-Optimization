#include <nav_core/recovery_behavior.h>
#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>

namespace my_global_planner {

class DummyRecovery : public nav_core::RecoveryBehavior {
public:
    DummyRecovery() {}

    void initialize(std::string name, tf2_ros::Buffer* tf, 
                    costmap_2d::Costmap2DROS* global_costmap, 
                    costmap_2d::Costmap2DROS* local_costmap) override {
        ROS_INFO("DummyRecovery initialized");
    }

    void runBehavior() override {
        ROS_INFO("DummyRecovery behavior is running");
    }

    ~DummyRecovery() {}
};

}  // namespace my_global_planner

PLUGINLIB_EXPORT_CLASS(my_global_planner::DummyRecovery, nav_core::RecoveryBehavior)
