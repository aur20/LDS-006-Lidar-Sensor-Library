#!/usr/bin/env python3
import rospy
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2, PointField
import std_msgs.msg
from lds006 import LDSSerialManager
import msgLDS_pb2 as msgLDS_pb2
from queue import Queue
import math

if __name__ == '__main__':
    rospy.init_node("lidar_sensor_publisher")
    cloud_pub = rospy.Publisher("lidar_pc", PointCloud2, queue_size=4)
    rate = rospy.Rate(15)

    port = rospy.get_param("port", "/dev/serial0")
    frame_id = rospy.get_param("frame_id", "fcu")
    
    print("Connecting to: " + port)
    with LDSSerialManager(port) as lds:
        header = std_msgs.msg.Header()
        header.frame_id = frame_id
        fields = [
                PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                PointField('y', 4, PointField.FLOAT32, 1),
                PointField('z', 8, PointField.FLOAT32, 1),
                #PointField('rgb', 12, PointField.UINT32, 1),
                PointField('rgb', 16, PointField.UINT32, 1),
            ]

        q_data = Queue(maxsize=3)
        def get_data(x):
            q_data.put(x)
        cb_hash = lds.registerCB(get_data)
        lds.start()
        while not rospy.is_shutdown():
            msg = msgLDS_pb2.msgLDS()
            data = q_data.get()
            msg.ParseFromString(data)

            points = []
            for i in msg.data:
                if i.distance > 400 and i.distance < 10000:
                    points.append([i.distance / 1000 * math.cos(i.angle / 180. * math.pi), i.distance / 1000 * math.sin(i.angle / 180. * math.pi), 0, 0xFFFF])

            header.stamp = rospy.Time.now()
            pc = point_cloud2.create_cloud(header, fields, points)
            cloud_pub.publish(pc)
            rate.sleep()

    rospy.spin()
