#! /usr/bin/env python

# adapted from by https://github.com/jeguzzi/drone_arena/blob/master/scripts/safety_pose.py


import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import NavSatFix, NavSatStatus
from dynamic_reconfigure.server import Server
from mavros_external_tracker.cfg import LocalizationConfig
import diagnostic_updater
import diagnostic_msgs

topic_type = {'pose': PoseStamped, 'fix': NavSatFix}


class Localization(object):
    def __init__(self):
        rospy.init_node('localization')
        self.last_msg = []
        self.pub = {PoseStamped: rospy.Publisher('mavros/mocap/pose', PoseStamped, queue_size=1),
                    NavSatFix: rospy.Publisher('mavros/mocap/input_fix', NavSatFix, queue_size=1)}
        self.loc_pub = rospy.Publisher('localization', String, queue_size=1, latch=True)
        self._localization = '?'
        self.localization = None
        self.active = rospy.get_param('~active', True)
        self.timeout = rospy.get_param('~timeout', 0.5)
        max_rate = rospy.get_param('~max_rate', None)
        if max_rate is None:
            self.min_period = None
        else:
            self.min_period = 1.0 / max_rate
        self.active_index = 0
        self.updater = diagnostic_updater.Updater()
        self.updater.setHardwareID("localization sources")
        self.topics = []
        for i, data in enumerate(rospy.get_param('~in', [])):
            topic = data['topic']
            label = data['label']
            try:
                _type = topic_type[data['type']]
            except KeyError as e:
                rospy.logerr('sources init: %s', e)
                continue
            self.last_msg.append((None, None))
            self.topics.append((label, topic, _type))
            rospy.Subscriber(topic, _type, self.got_msg_from(i))
            self.updater.add('{0} [{1}]'.format(label, topic), self.diagnostics(i))
        self.updater.add('active source', self.active_diagnostics)
        rospy.Timer(rospy.Duration(1), self.update_diagnostics)
        rospy.Timer(rospy.Duration(self.timeout), self.update_localization)
        self.srv = Server(LocalizationConfig, self.reconfigure)
        rospy.spin()

    @property
    def localization(self):
        return self._localization

    @localization.setter
    def localization(self, value):
        if value != self._localization:
            rospy.logwarn('Switch to source {0} from {1}'.format(value, self._localization))
            self._localization = value
            self.loc_pub.publish(value or '')

    def update_localization(self, event):
        if self.localization:
            t, _ = self.last_msg[self.active_index]
            if not t:
                self.localization = None
            else:
                dt = (rospy.Time.now() - t).to_sec()
                if dt > 2.0 * self.timeout:
                    self.localization = None

    def update_diagnostics(self, event):
        self.updater.update()

    def active_diagnostics(self, stat):
        index = self.active_index
        label, topic, _type = self.topics[index]
        t, msg = self.last_msg[index]
        if not msg:
            stat.summary(diagnostic_msgs.msg.DiagnosticStatus.ERROR,
                         "should be {0} but was never updated".format(label))
            return stat
        dt = (rospy.Time.now() - t).to_sec()
        if dt > self.timeout:
            stat.summary(diagnostic_msgs.msg.DiagnosticStatus.ERROR,
                         "should be {0} but is not being updated".format(label))
        else:
            stat.summary(diagnostic_msgs.msg.DiagnosticStatus.OK,
                         "is {0}".format(label))
        return stat

    def diagnostics(self, index):
        OK = diagnostic_msgs.msg.DiagnosticStatus.OK
        WARN = diagnostic_msgs.msg.DiagnosticStatus.WARN
        label, topic, _type = self.topics[index]

        def f(stat):
            t, msg = self.last_msg[index]

            if not msg:
                stat.summary(WARN, "Never updated")
                return stat
            dt = (rospy.Time.now() - t).to_sec()
            if dt > self.timeout:
                stat.summary(WARN, "Last updated {0:.0f} seconds ago".format(dt))
            else:
                stat.summary(OK, "Alive")
                if _type == PoseStamped:
                    stat.add('frame_id', msg.header.frame_id)
                    stat.add('x', msg.pose.position.x)
                    stat.add('y', msg.pose.position.y)
                    stat.add('z', msg.pose.position.z)
                elif _type == NavSatFix:
                    stat.add('status', msg.status.status)
                    stat.add('latitude', msg.latitude)
                    stat.add('longitude', msg.longitude)
                    stat.add('altitude', msg.altitude)
            return stat
        return f

    def got_msg_from(self, index):
        label, topic, _type = self.topics[index]

        def f(msg):
            t, _ = self.last_msg[index]
            if(self.min_period is not None and t is not None and
               (rospy.Time.now() - t).to_sec() < self.min_period):
                return
            # if not fix we are not interessed in the message
            if _type == NavSatFix:
                if msg.status.status == NavSatStatus.STATUS_NO_FIX:
                    return
            self.last_msg[index] = (rospy.Time.now(), msg)
            if self.active:
                if index <= self.active_index:
                    self.active_index = index
                else:
                    at, _ = self.last_msg[self.active_index]
                    if not at:
                        self.active_index = index
                    else:
                        dt = (rospy.Time.now() - at).to_sec()
                        if dt > self.timeout:
                            self.active_index = index
            if index == self.active_index:
                out_msg = msg
                out_msg.header.stamp = rospy.Time.now()
                self.pub[_type].publish(out_msg)
                self.localization = label
        return f

    def reconfigure(self, config, level):
        self.active = config['active']
        if not self.active:
            config['active_index'] = min(config['active_index'], len(self.topics) - 1)
            self.active_index = config['active_index']
        return config


if __name__ == '__main__':
    Localization()
