import rospy
import rosbag
from std_msgs.msg import Int32, String

bag = rosbag.Bag('~/Desktop/test.bag', 'w')

try:
    str = String()
    str.data = 'foo'

    i = Int32()
    i.data = 42

    bag.write('/wrench', str)
    bag.write('numbers', i)
finally:
    bag.close()
