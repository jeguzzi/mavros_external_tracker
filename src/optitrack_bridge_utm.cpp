/**
 * @brief MocapPoseEstimate plugin
 * @file mocap_pose_estimate.cpp
 * @author Tony Baltovski <tony.baltovski@gmail.com>
 * @author Vladimir Ermakov <vooon341@gmail.com>
 *
 * @addtogroup plugin
 * @{
 */
/*
 * Copyright 2014,2015,2016 Vladimir Ermakov, Tony Baltovski.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */

#include <mavros/mavros_plugin.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/Vector3.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/NavSatStatus.h>
#include <sensor_msgs/Range.h>
#include <mavros/gps_conversions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <dynamic_reconfigure/server.h>
#include <mavros_external_tracker/TrackerConfig.h>


namespace mavros {
namespace extra_plugins{
/**
 * @brief MocapPoseEstimate plugin
 *
 * Sends motion capture data to FCU.
 */
class OptitrackPlugin : public plugin::PluginBase
{
public:
    OptitrackPlugin() : PluginBase(),
        mp_nh("~mocap"),
        r_nh("~mocap/range"),
        utm_zone_("32T")
      //   server(mp_nh)
    { }

    void initialize(UAS &uas_)
    {
        PluginBase::initialize(uas_);

        range_is_valid = false;
        range_stamp = ros::Time::now();
        r_nh.param("timeout", range_timeout, 1.0);  //Dynamic
        r_nh.param("enable", range_enable, true); //Dynamic
        r_nh.param("maximum", range_max, 100.0); //Dynamic
        mp_nh.param("gps_id", gps_id, 0);
      //   mp_nh.param<std::string>("utm_zone", utm_zone_, "32T");
        mp_nh.param("publish_fix", publish_fix, false); //Dynamic
        mp_nh.param("enable_logs", enable_logs, false); //Dynamic
        mp_nh.param("s_error", s_error, 0.01); //Dynamic
        mp_nh.param("h_error", h_error, 0.01); //Dynamic
        mp_nh.param("v_error", v_error, 0.01); //Dynamic
        mp_nh.param("include_altitude", include_altitude, true); //Dynamic
        mp_nh.param("include_gps_altitude", include_gps_altitude, false);
        // Used to correct a bug. The mavlink message altitude field is meant to be in meters BUT
        // is interpreted as cm
        mp_nh.param("altitude_factor", altitude_factor, 100.0);
        mocap_pose_sub = mp_nh.subscribe("pose", 1, &OptitrackPlugin::mocap_pose_cb, this);
        mocap_odom_sub = mp_nh.subscribe("odom", 1, &OptitrackPlugin::mocap_odom_cb, this);
        range_sub = mp_nh.subscribe("range", 1, &OptitrackPlugin::range_cb, this);
        nav_sat_sub = mp_nh.subscribe("input_fix", 1, &OptitrackPlugin::fix_cb, this);
        nav_sat_pub = mp_nh.advertise<sensor_msgs::NavSatFix>("fix", 1);

        dynamic_reconfigure::Server<mavros_external_tracker::TrackerConfig>::CallbackType f = boost::bind(&OptitrackPlugin::dynamic_reconfigure, this, _1, _2);
        server.setCallback(f);

    }

    Subscriptions get_subscriptions()
    {
        return { /* Rx disabled */ };
    }

private:

    dynamic_reconfigure::Server<mavros_external_tracker::TrackerConfig> server;

    std::string utm_zone_;
    ros::NodeHandle mp_nh;
    ros::NodeHandle r_nh;
    int gps_id;
    ros::Subscriber mocap_pose_sub;
	 ros::Subscriber mocap_odom_sub;
    ros::Subscriber nav_sat_sub;
    ros::Subscriber range_sub;
    ros::Publisher nav_sat_pub;

    bool include_altitude, include_gps_altitude;
    double altitude_factor;
    double h_error, v_error, s_error;
    ros::Time range_stamp;
    double range;
    bool range_is_valid;
    bool range_enable;
    double range_max;
    double range_timeout;
    bool enable_logs;
    bool publish_fix;

    void dynamic_reconfigure(mavros_external_tracker::TrackerConfig &config, uint32_t level)
    {
      range_timeout = config.timeout;
      range_enable = config.enable;
      range_max = config.maximum;

      publish_fix = config.publish_fix;
      enable_logs = config.enable_logs;

      s_error = config.s_error;
      h_error = config.h_error;
      v_error = config.v_error;
      include_altitude = config.include_altitude;
    }


    bool use_range()
    {
      // ROS_INFO("use_range: %d %d %d", range_enable, range_is_valid, (ros::Time::now() - range_stamp).toSec() < range_timeout);
      return range_enable && range_is_valid && (ros::Time::now() - range_stamp).toSec() < range_timeout;
    }

    /* -*- low-level send -*- */
    void mocap_send_gps_input(uint64_t usec, double lat, double lon, double alt, double ve, double vn, bool ignore_velocity, bool ignore_altitude)
    {

      // <message id="232" name="GPS_INPUT">
      //            <description>GPS sensor input message.  This is a raw sensor value sent by the GPS. This is NOT the global position estimate of the sytem.</description>
      //            <field type="uint64_t" name="time_usec">Timestamp (micros since boot or Unix epoch)</field>
      //            <field type="uint8_t" name="gps_id">ID of the GPS for multiple GPS inputs</field>
      //            <field type="uint16_t" name="ignore_flags" enum="GPS_INPUT_IGNORE_FLAGS">Flags indicating which fields to ignore (see GPS_INPUT_IGNORE_FLAGS enum).  All other fields must be provided.</field>
      //            <field type="uint32_t" name="time_week_ms">GPS time (milliseconds from start of GPS week)</field>
      //            <field type="uint16_t" name="time_week">GPS week number</field>
      //            <field type="uint8_t" name="fix_type">0-1: no fix, 2: 2D fix, 3: 3D fix. 4: 3D with DGPS. 5: 3D with RTK</field>
      //            <field type="int32_t" name="lat">Latitude (WGS84), in degrees * 1E7</field>
      //            <field type="int32_t" name="lon">Longitude (WGS84), in degrees * 1E7</field>
      //            <field type="float" name="alt">Altitude (AMSL, not WGS84), in m (positive for up)</field>
      //            <field type="float" name="hdop">GPS HDOP horizontal dilution of position in m</field>
      //            <field type="float" name="vdop">GPS VDOP vertical dilution of position in m</field>
      //            <field type="float" name="vn">GPS velocity in m/s in NORTH direction in earth-fixed NED frame</field>
      //            <field type="float" name="ve">GPS velocity in m/s in EAST direction in earth-fixed NED frame</field>
      //            <field type="float" name="vd">GPS velocity in m/s in DOWN direction in earth-fixed NED frame</field>
      //            <field type="float" name="speed_accuracy">GPS speed accuracy in m/s</field>
      //            <field type="float" name="horiz_accuracy">GPS horizontal accuracy in m</field>
      //            <field type="float" name="vert_accuracy">GPS vertical accuracy in m</field>
      //            <field type="uint8_t" name="satellites_visible">Number of satellites visible.</field>
      // </message>
        mavlink::common::msg::GPS_INPUT fix;

        fix.time_usec = usec;
        fix.gps_id = gps_id;
		  fix.ignore_flags = 16; //ignore vertical velocity
		  if(ignore_velocity)
		  {
			  fix.ignore_flags += 40; //i.e. ignore horizontal velocity (and its error)
		  }

        if(ignore_altitude)
        {
            fix.ignore_flags += 133; //i.e. ignore altitude (and its errors)
        }
        fix.time_week_ms = 464508000; // TODO compute from UTM time
        fix.time_week = 1914; // TODO compute from UTM time
        fix.lat = round(lat * 1e7);
        fix.lon = round(lon * 1e7);
        fix.fix_type = 3;
        fix.alt = alt * altitude_factor;
        fix.hdop = h_error;
        fix.vdop = v_error;
        fix.vn = vn;
        fix.ve = ve;
        fix.speed_accuracy = s_error;
        fix.horiz_accuracy = h_error;
        fix.vert_accuracy = v_error;
        fix.satellites_visible = 12;
        UAS_FCU(m_uas)->send_message_ignore_drop(fix);
        if(enable_logs) ROS_DEBUG("Send message %s",fix.to_yaml().data());
    }

	 void send_gps_msgs(const geometry_msgs::Point &utm_point,
                       const ros::Time &stamp)
	{
		geometry_msgs::Vector3 velocity = geometry_msgs::Vector3();
		send_gps_msgs(utm_point, velocity, stamp, true);
	}

    void send_gps_msgs(const geometry_msgs::Point &utm_point,
                       const geometry_msgs::Vector3 &utm_velocity,
                       const ros::Time &stamp, bool ignore_velocity=false)
    {
        double lat, lon;
        double alt;
        bool should_use_range = use_range();
        if(should_use_range)
        {
           alt = range;
        }
        else{
           alt = utm_point.z;
        }
        //include_altitude = include altitude from pose
        //should_use_range = include altitude from range
        bool ignore_altitude = !include_altitude && !should_use_range;

        UTM::UTMtoLL(utm_point.y, utm_point.x, utm_zone_, lat, lon);
        if(enable_logs) ROS_DEBUG("pose (%.2f, %.2f) -> lat = %f,  lon = %f", utm_point.x, utm_point.y, lat, lon);
        // UTM: x axis points east
        //      y axis points north
        // that is ve = v.x and vn = v.y
        mocap_send_gps_input(stamp.toNSec() / 1000, lat, lon, alt, utm_velocity.x, utm_velocity.y,
                             ignore_velocity, ignore_altitude);
        if(publish_fix)
        {
            sensor_msgs::NavSatFix msg;
            msg.header.stamp = stamp;
            msg.header.frame_id = "base_link";
            msg.latitude = lat;
            msg.longitude = lon;
            msg.altitude = alt;
            msg.position_covariance_type = sensor_msgs::NavSatFix::COVARIANCE_TYPE_UNKNOWN;
            nav_sat_pub.publish(msg);
        }
    }

    /* -*- mid-level helpers -*- */
    void mocap_odom_cb(const nav_msgs::Odometry::ConstPtr &odom_msg)
    {
        geometry_msgs::PoseStamped utm_pose_msg;
        geometry_msgs::Vector3Stamped utm_velocity_msg;

        geometry_msgs::PoseStamped pose_msg = geometry_msgs::PoseStamped();
        pose_msg.header = odom_msg->header;
        pose_msg.pose = odom_msg->pose.pose;

        geometry_msgs::Vector3Stamped velocity_msg = geometry_msgs::Vector3Stamped();
        velocity_msg.header = odom_msg->header;
        velocity_msg.header.frame_id = odom_msg->child_frame_id;
		  velocity_msg.vector = odom_msg->twist.twist.linear;

        try {
            geometry_msgs::TransformStamped transform = m_uas->tf2_buffer.lookupTransform("utm", pose_msg.header.frame_id, ros::Time(0), ros::Duration(0.1));
            tf2::doTransform(pose_msg, utm_pose_msg, transform);
        }
        catch (tf2::TransformException &ex) {
            ROS_ERROR("%s \nFrames: %s", ex.what(), m_uas->tf2_buffer.allFramesAsString().data());
            return;
        }

        try {
            geometry_msgs::TransformStamped transform = m_uas->tf2_buffer.lookupTransform("utm", velocity_msg.header.frame_id, ros::Time(0), ros::Duration(0.1));
            tf2::doTransform(velocity_msg, utm_velocity_msg, transform);
        }
        catch (tf2::TransformException &ex) {
            ROS_ERROR("%s \nFrames: %s", ex.what(), m_uas->tf2_buffer.allFramesAsString().data());
            return;
        }
        send_gps_msgs(utm_pose_msg.pose.position, utm_velocity_msg.vector, odom_msg->header.stamp);
    }

    /* -*- mid-level helpers -*- */
    void mocap_pose_cb(const geometry_msgs::PoseStamped::ConstPtr &pose_msg)
    {
        geometry_msgs::PoseStamped utm_pose_msg;
        try {
            geometry_msgs::TransformStamped transform = m_uas->tf2_buffer.lookupTransform("utm", pose_msg->header.frame_id, ros::Time(0), ros::Duration(0.1));
            tf2::doTransform(*pose_msg, utm_pose_msg, transform);
        }
        catch (tf2::TransformException &ex) {
            ROS_ERROR("%s \nFrames: %s", ex.what(), m_uas->tf2_buffer.allFramesAsString().data());
            return;
        }
        send_gps_msgs(utm_pose_msg.pose.position, pose_msg->header.stamp);
    }

    void fix_cb(const sensor_msgs::NavSatFix::ConstPtr &fix_msg)
    {
      uint64_t usec = fix_msg->header.stamp.toNSec() / 1000;
      mavlink::common::msg::GPS_INPUT fix;
      fix.time_usec = usec;
      fix.gps_id = gps_id;
      fix.ignore_flags = 56; //ignore velocity amd its error
      // if(!include_altitude)
      // {
      //     fix.ignore_flags += 133; //i.e. ignore altitude (and its errors)
      // }
      fix.time_week_ms = 464508000; // TODO compute from UTM time
      fix.time_week = 1914; // TODO compute from UTM time
      fix.lat = round(fix_msg->latitude * 1e7);
      fix.lon = round(fix_msg->longitude * 1e7);
      if(fix_msg->status.status == sensor_msgs::NavSatStatus::STATUS_NO_FIX)
      {
         fix.fix_type = 3;
      }
      else
      {
         fix.fix_type = 0;
      }

      double alt = fix_msg->altitude;

      bool should_use_range = use_range();
      if(should_use_range)
      {
         alt = range;
         fix.vdop = v_error;
         fix.vert_accuracy = v_error;
      }

      if(!include_gps_altitude && !should_use_range)
      {
          fix.ignore_flags += 133; //i.e. ignore altitude (and its errors)
      }

      fix.alt = alt * altitude_factor;
      // TODO: get errors from fix_msg->covariance and fix_msg->position_covariance_type
      // fix.hdop = h_error;
      // fix.vdop = v_error;
      // fix.horiz_accuracy = h_error;
      // fix.vert_accuracy = v_error;
      fix.satellites_visible = 10;
      UAS_FCU(m_uas)->send_message_ignore_drop(fix);
      if(enable_logs) ROS_DEBUG("Send message %s",fix.to_yaml().data());
      if(publish_fix) nav_sat_pub.publish(fix_msg);
    }

    void range_cb(const sensor_msgs::Range::ConstPtr &range_msg)
    {
      range_stamp = range_msg->header.stamp;
      range = range_msg->range;
      range_is_valid = range < range_msg->max_range && range < range_max;
    }

};
}    // namespace extra_plugins
}    // namespace mavros

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mavros::extra_plugins::OptitrackPlugin, mavros::plugin::PluginBase)
