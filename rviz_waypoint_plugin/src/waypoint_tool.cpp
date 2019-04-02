
#include <tf/transform_listener.h>

#include <geometry_msgs/PoseStamped.h>

#include <rviz/display_context.h>
#include <rviz/properties/string_property.h>

#include "waypoint_tool.h"

namespace rviz
{

    WaypointTool::WaypointTool()
    {
        shortcut_key_ = 'w';

        topic_property_ = new StringProperty( "Topic", "clicked_pose",
                "The topic on which to publish waypoint.",
                getPropertyContainer(), SLOT( updateTopic()  ), this );

    }

    void WaypointTool::onInitialize()
    {
        PoseTool::onInitialize();
        setName( "Insert Waypoint"  );
        updateTopic();

    }

    void WaypointTool::updateTopic()
    {
        pub_ = nh_.advertise<geometry_msgs::PoseStamped>( topic_property_->getStdString(), 1  );
    }

    void WaypointTool::onPoseSet(double x, double y, double theta)
    {
        std::string fixed_frame = context_->getFixedFrame().toStdString();
        geometry_msgs::PoseStamped pose;
        pose.header.frame_id = fixed_frame;
        pose.header.stamp = ros::Time::now();
        pose.pose.position.x = x;
        pose.pose.position.y = y;

        tf::Quaternion quat;
        quat.setRPY(0.0, 0.0, theta);
        tf::quaternionTFToMsg(quat,
                pose.pose.orientation);
        ROS_INFO("Setting waypoint: %.3f %.3f %.3f [frame=%s]", x, y, theta, fixed_frame.c_str());
        pub_.publish(pose);
    }

} // end namespace rviz_waypoint_plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS( rviz::WaypointTool, rviz::Tool  )
