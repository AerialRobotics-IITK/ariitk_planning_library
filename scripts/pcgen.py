import rospy
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import PointField
from ast import literal_eval

pc_pub = rospy.Publisher('pcloud', PointCloud2, queue_size=1)
rospy.init_node("pc_publisher")
rate = rospy.Rate(10)

cloud = PointCloud2()
cloud.header.frame_id = "firefly/base_link"

fieldx = PointField()
fieldx.name = "x"
fieldx.offset = 0
fieldx.datatype = fieldx.FLOAT32
fieldx.count = 1
cloud.fields.append(fieldx)

fieldy = PointField()
fieldy.name = "y"
fieldy.offset = 4
fieldy.datatype = fieldy.FLOAT32
fieldy.count = 1
cloud.fields.append(fieldy)

fieldz = PointField()
fieldz.name = "z"
fieldz.offset = 8
fieldz.datatype = fieldz.FLOAT32
fieldz.count = 1
cloud.fields.append(fieldz)

fieldc = PointField()
fieldc.name = "rgb"
fieldc.offset = 16
fieldc.datatype = fieldc.FLOAT32
fieldc.count = 1
cloud.fields.append(fieldc)

cloud.is_bigendian = False
cloud.point_step = 32

cloud.height = 1
cloud.width = 7

cloud.row_step = cloud.width * cloud.point_step
cloud.is_dense = False

cloud.data = [0, 0, 128, 63
            , 0, 0, 128, 63
            , 0, 0, 128, 63
            , 0, 0, 0, 0
            , 114, 114, 114, 0
            , 0, 0, 0, 0
            , 0, 0, 0, 0
            , 0, 0, 0, 0,
            
            0, 0, 128, 63
            , 0, 0, 0, 0
            , 0, 0, 128, 63
            , 0, 0, 0, 0
            , 114, 114, 114, 0
            , 0, 0, 0, 0
            , 0, 0, 0, 0
            , 0, 0, 0, 0,
            
            0, 0, 128, 63
            , 0, 0, 128, 191
            , 0, 0, 128, 63
            , 0, 0, 0, 0
            , 114, 114, 114, 0
            , 0, 0, 0, 0
            , 0, 0, 0, 0
            , 0, 0, 0, 0,
            
            0, 0, 10, 0
            , 0, 0, 128, 191
            , 0, 0, 128, 63
            , 0, 0, 0, 0
            , 114, 114, 114, 0
            , 0, 0, 0, 0
            , 0, 0, 0, 0
            , 0, 0, 0, 0,
            
            0, 0, 0, 0
            , 0, 0, 128, 63
            , 0, 0, 128, 63
            , 0, 0, 0, 0
            , 114, 114, 114, 0
            , 0, 0, 0, 0
            , 0, 0, 0, 0
            , 0, 0, 0, 0,
            
            0, 0, 128, 191
            , 0, 0, 128, 191
            , 0, 0, 128, 63
            , 0, 0, 0, 0
            , 114, 114, 114, 0
            , 0, 0, 0, 0
            , 0, 0, 0, 0
            , 0, 0, 0, 0,
            
            0, 0, 128, 191
            , 0, 0, 128, 63
            , 0, 0, 128, 63
            , 0, 0, 0, 0
            , 114, 114, 114, 0
            , 0, 0, 0, 0
            , 0, 0, 0, 0
            , 0, 0, 0, 0]

while not rospy.is_shutdown():
    cloud.header.stamp = rospy.Time.now()
    pc_pub.publish(cloud)
    rate.sleep()

