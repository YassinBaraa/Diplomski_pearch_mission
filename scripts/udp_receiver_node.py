#!/usr/bin/env python3
import socket
import sys
import os

import rospy
from std_msgs.msg import Float32

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from udp_packet import PerchPacket


class UdpReceiverNode:
    def __init__(self):
        rospy.init_node('udp_receiver_node')

        ip = rospy.get_param('~bind_ip', '0.0.0.0')
        port = int(rospy.get_param('~bind_port', 5005))
        timeout = float(rospy.get_param('~timeout', 0.5))

        self.pub_ex = rospy.Publisher('/perch/error_x', Float32, queue_size=1)
        self.pub_ey = rospy.Publisher('/perch/error_y', Float32, queue_size=1)
        self.pub_tof = rospy.Publisher('/perch/tof', Float32, queue_size=1)

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind((ip, port))
        self.sock.settimeout(timeout)

        rospy.loginfo(f"UDP receiver bound to {ip}:{port}")

    def spin(self):
        while not rospy.is_shutdown():
            try:
                data, _ = self.sock.recvfrom(65507)
            except socket.timeout:
                continue
            except Exception as e:
                rospy.logerr(f"Socket error: {e}")
                continue

            try:
                pkt = PerchPacket.from_json(data)
            except Exception as e:
                rospy.logwarn(f"Bad packet: {e}")
                continue

            self.pub_ex.publish(Float32(pkt.error_x))
            self.pub_ey.publish(Float32(pkt.error_y))
            self.pub_tof.publish(Float32(pkt.tof_mm))

    def shutdown(self):
        self.sock.close()


def main():
    node = UdpReceiverNode()
    rospy.on_shutdown(node.shutdown)
    try:
        node.spin()
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
