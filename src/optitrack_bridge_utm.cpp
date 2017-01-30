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
#include <sensor_msgs/NavSatFix.h>
#include <mavros/gps_conversions.h>
// #include <tf2_geometry_msgs/tf2_geometry_msgs.h>

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
		utm_zone_("32N")
	{ }

	void initialize(UAS &uas_)
	{
		PluginBase::initialize(uas_);
		// toUTM_defined = false;
		// tf_frame_id = "World";
		// tf_child_frame_id = "utm";
		// tf_rate = 10.0;
		// tf2_start("optitrackTF", &OptitrackPlugin::transform_cb);
		mp_nh.param("gps_id", gps_id, 0);
		mp_nh.param("publish_fix", publish_fix, false);
		mp_nh.param("h_error", h_error, 0.01);
		mp_nh.param("v_error", v_error, 0.01);
		mp_nh.param("include_altitude", include_altitude, true);
		mocap_pose_sub = mp_nh.subscribe("pose", 1, &OptitrackPlugin::mocap_pose_cb, this);
		nav_sat_pub = mp_nh.advertise<sensor_msgs::NavSatFix>("fix", 1);
	}

	Subscriptions get_subscriptions()
	{
		return { /* Rx disabled */ };
	}

private:
	// friend class TF2ListenerMixin;
	// bool toUTM_defined;
	// bool tf_listen;
	// std::string tf_frame_id;
	// std::string tf_child_frame_id;
	// double tf_rate;
	std::string utm_zone_;
	ros::NodeHandle mp_nh;
	int gps_id;
	ros::Subscriber mocap_pose_sub;
	ros::Publisher nav_sat_pub;
	bool publish_fix;
	bool include_altitude;
	double h_error;
	double v_error;
	// tf::StampedTransform toUTM;

	// void transform_cb(const geometry_msgs::TransformStamped &transform) {
	// 	ROS_INFO("Received tf -> define toUTM");
	// 	tf::transformStampedMsgToTF(transform, toUTM);
	// 	toUTM_defined = true;
	// }

	/* -*- low-level send -*- */
	void mocap_send_gps_input(uint64_t usec, double lat, double lon, double alt)
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
		fix.ignore_flags = 56; //i.e. ignore velocity informations
		if(!include_altitude)
		{
			fix.ignore_flags += 1;
		}
		fix.time_week_ms = 464508000; // TODO compute from UTM time
		fix.time_week = 1914; // TODO compute from UTM time
		fix.lat = round(lat * 1e7);
		fix.lon = round(lon * 1e7);
		fix.fix_type = 3;
		fix.alt = alt;
		fix.hdop = h_error;
		fix.vdop = v_error;
		fix.vn = 0.0;
		fix.ve = 0.0;
		fix.speed_accuracy = 0.01;
		fix.horiz_accuracy = h_error;
		fix.vert_accuracy = v_error;
		fix.satellites_visible = 12;

		UAS_FCU(m_uas)->send_message_ignore_drop(fix);
		ROS_DEBUG("Send message %s",fix.to_yaml().data());

	}

	/* -*- mid-level helpers -*- */
	void mocap_pose_cb(const geometry_msgs::PoseStamped::ConstPtr &pose_msg)
	{

		// tf::Pose pose;
		// tf::Pose utm_pose;
		// tf::poseMsgToTF(pose_msg->pose, pose);
		geometry_msgs::PoseStamped utm_pose_msg;
		try {
			geometry_msgs::TransformStamped transform = m_uas->tf2_buffer.lookupTransform("utm", pose_msg->header.frame_id, ros::Time(0), ros::Duration(0.1));
			tf2::doTransform(*pose_msg, utm_pose_msg, transform);
		}
		catch (tf2::TransformException &ex) {
			ROS_ERROR("%s \nFrames: %s", ex.what(), m_uas->tf2_buffer.allFramesAsString().data());
			return;
		}



		// except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
		//         tf2_ros.ExtrapolationException) as e:

		// if(toUTM_defined)
		// {
		// 	utm_pose = toUTM * pose;
		// }
		// else
		// {
		// 	ROS_ERROR("Cannot tranform between World and utm");
		// 	return;
		// }
		// geometry_msgs::Pose utm_pose_msg;
		// tf::poseTFToMsg(utm_pose, utm_pose_msg);
		// try{
		// 	tl.transformPose(std::string("utm"), *pose, utm_pose);
		// }
		// catch (tf::TransformException &ex) {
		// 	ROS_ERROR("%s",ex.what());
		// 	ROS_ERROR("%s", tl.allFramesAsString().data());
		// 	return;
		// }

		double lat, lon;
		double alt = utm_pose_msg.pose.position.z;
		UTM::UTMtoLL(utm_pose_msg.pose.position.y, utm_pose_msg.pose.position.x, utm_zone_, lat, lon);
		ROS_INFO("pose %f %f -> lat %f lon %f", utm_pose_msg.pose.position.x, utm_pose_msg.pose.position.y, lat, lon);
		mocap_send_gps_input(pose_msg->header.stamp.toNSec() / 1000, lat, lon, alt);
		if(publish_fix)
		{
			sensor_msgs::NavSatFix msg;
			msg.header.stamp = pose_msg->header.stamp;
			msg.header.frame_id = "base_link";
			msg.latitude = lat;
			msg.longitude = lon;
			msg.altitude = alt;
			msg.position_covariance_type = sensor_msgs::NavSatFix::COVARIANCE_TYPE_UNKNOWN;
			nav_sat_pub.publish(msg);
		}
	}


};
}	// namespace extra_plugins
}	// namespace mavros

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mavros::extra_plugins::OptitrackPlugin, mavros::plugin::PluginBase)
