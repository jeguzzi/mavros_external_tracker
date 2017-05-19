#include <mavros/mavros_plugin.h>
#include <sensor_msgs/Range.h>


namespace mavros {
namespace extra_plugins {

class RangeFinderPlugin : public plugin::PluginBase
{
public:
RangeFinderPlugin() : PluginBase(),
        mp_nh("~rangefinder")
{
}

void initialize(UAS &uas_)
{
        PluginBase::initialize(uas_);
        range_pub = mp_nh.advertise<sensor_msgs::Range>("range", 1);
        mp_nh.param<std::string>("frame_id", range_msg.header.frame_id, "range_finder");
        mp_nh.param<float>("min_range", range_msg.min_range, 0.0);
        mp_nh.param<float>("max_range", range_msg.max_range, 10.0);
        mp_nh.param<float>("field_of_view", range_msg.field_of_view, 0.052);
        std::string type;
        mp_nh.param<std::string>("radiation_type", type, "infrared");
        if(type.compare("infrared") == 0)
        {
                range_msg.radiation_type = sensor_msgs::Range::INFRARED;
        }
        else if(type.compare("ultrasound") == 0)
        {
                range_msg.radiation_type = sensor_msgs::Range::ULTRASOUND;
        }
        else{
                ROS_ERROR("Wrong radiation_type parameter. Should be one of infrared or ultrasound");
        }
        UAS_DIAG(m_uas).add("Range finder", this, &RangeFinderPlugin::update_diagnostics);
}

Subscriptions get_subscriptions()
{
        return {
                       make_handler(&RangeFinderPlugin::handle_rangefinder),
        };
}

private:
ros::NodeHandle mp_nh;
ros::Publisher range_pub;
sensor_msgs::Range range_msg;


void update_diagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
   if((ros::Time::now() - range_msg.header.stamp).toSec() < 1.0)
   {
      stat.summary(0, "Ok");
      stat.addf("Range", "%.2f m", range_msg.range);
   }
   else
   {
      stat.summary(2, "Not alive");
   }
}

/**
 * Receive distance sensor data from FCU.
 */
void handle_rangefinder(const mavlink::mavlink_message_t *msg, mavlink::ardupilotmega::msg::RANGEFINDER &rangefinder_msg)
{
        range_msg.header.stamp = ros::Time::now();
        range_msg.range = rangefinder_msg.distance;
        range_pub.publish(range_msg);
}
};
}    // namespace extra_plugins
}    // namespace mavros

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mavros::extra_plugins::RangeFinderPlugin, mavros::plugin::PluginBase)
