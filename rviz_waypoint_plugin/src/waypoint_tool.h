#ifndef PLANT_FLAG_TOOL_H
#define PLANT_FLAG_TOOL_H

#include <rviz/tool.h>

#ifndef Q_MOC_RUN  // See: https://bugreports.qt-project.org/browse/QTBUG-22829
# include <QObject>

# include <ros/ros.h>

# include <rviz/default_plugin/tools/pose_tool.h>
#endif

namespace rviz
{
    class Arrow;
    class DisplayContext;
    class StringProperty;

    class WaypointTool: public PoseTool
    {
        Q_OBJECT
        public:
            WaypointTool();
            virtual ~WaypointTool() {}
            virtual void onInitialize();

        protected:
            virtual void onPoseSet(double x, double y, double theta);

            private Q_SLOTS:
                void updateTopic();

        private:
            ros::NodeHandle nh_;
            ros::Publisher pub_;

            StringProperty* topic_property_;
    };
}

#endif
