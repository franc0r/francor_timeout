import os
import collections as col
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from tf2_ros import TransformStamped
from rclpy.qos import QoSProfile, qos_profile_sensor_data, qos_profile_system_default

import wombat_waypoint.waypoint_utils as wp

from geometry_msgs.msg import Twist

class FrancorTimeoutNav(Node):

  def __init__(self):
    super().__init__('map_repub_node')

    self.target_ip = "69.69.69.222"
    self.ping_rate = 1.0
    self.rate = 10.0
    self.timeout_cnt = 3

    #params
    self.declare_parameter('target_ip', self.target_ip)
    self.declare_parameter('ping_rate', self.ping_rate)
    self.declare_parameter('rate', self.rate)
    self.declare_parameter('timeout_cnt', self.timeout_cnt)

    self.target_ip = self.get_parameter('target_ip').get_parameter_value().string_value
    self.ping_rate      = self.get_parameter('ping_rate').get_parameter_value().double_value
    self.rate           = self.get_parameter('rate').get_parameter_value().double_value
    self.timeout_cnt   = self.get_parameter('timeout_cnt').get_parameter_value().integer_value

    #print params
    self.get_logger().info("#################################")
    self.get_logger().info("Francor_timeout_nav params:")
    self.get_logger().info("target_ip: " + str(self.target_ip))
    self.get_logger().info("ping_rate: " + str(self.ping_rate))
    self.get_logger().info("rate: " + str(self.rate))
    self.get_logger().info("timeout_cnt: " + str(self.timeout_cnt))
    self.get_logger().info("#################################")

    #subs
    # qos = qos_profile_system_default
    # if self.vel_in_3_qos == "sensor_data":
    #   qos = qos_profile_sensor_data
    # self.sub_vel_in_3 = self.create_subscription(Twist, self.vel_in_3_topic, self.vel_in_3_callback, qos)

    # #pub
    # qos = qos_profile_system_default
    # if self.vel_out_qos == "sensor_data":
    #   qos = qos_profile_sensor_data
    # self.pub_vel_out = self.create_publisher(Twist, self.vel_out_topic, qos)

    #timer
    self.ping_timer = self.create_timer(1.0/self.ping_rate, self.ping_timer_callback)
    self.loop_timer = self.create_timer(1.0/self.rate, self.loop_timer_callback)

    #vars
    self.disconnect_cnt = 0

    #helper
    self.navigator: wp.NavigationExecuter = None
    self.tf: wp.TfListener = None
    self.points: col.deque = col.deque(maxlen=50)

    self.moving = False
    

  def init(self):
    self.navigator = wp.NavigationExecuter(self)
    self.tf = wp.TfListener(self)


  def to_pose_stamped(self, t:TransformStamped):
    ret = PoseStamped()
    ret.header.frame_id = t.header.frame_id
    ret.header.stamp = t.header.stamp
    ret.pose.position.x = t.transform.translation.x
    ret.pose.position.y = t.transform.translation.y
    ret.pose.position.z = t.transform.translation.z
    ret.pose.orientation.x = t.transform.rotation.x
    ret.pose.orientation.y = t.transform.rotation.y
    ret.pose.orientation.z = t.transform.rotation.z
    ret.pose.orientation.w = t.transform.rotation.w
    return ret

  def loop_timer_callback(self):
    pass

  def ping_timer_callback(self):
    if self.disconnect_cnt == 0:
      if self.moving:
        self.navigator.stop()
        self.moving = False
      #get transform
      t: TransformStamped = self.tf.get_transform("base_footprint", "map")
      if t is not None:
        #convert to pose
        p: PoseStamped = self.to_pose_stamped(t)
        #add to points
        self.points.append(p)

    con = self.ping()
    #log con 
    self.get_logger().info("ping: " + str(con))
    if con:
      self.disconnect_cnt = 0
    else:
      self.disconnect_cnt += 1
    
    if self.disconnect_cnt >= self.timeout_cnt:
      #timeout!! -> navigate to last known position
      target = self.points.popleft()
      self.navigator.start(target)
      self.moving = True
      
      pass



  def ping(self) -> bool:
    result = os.system("ping -c 1 " + self.target_ip)
    if result == 0:
      return True
    else:
      return False





def main(args=None):
  rclpy.init(args=args)

  timeout_nav_node = FrancorTimeoutNav()
  timeout_nav_node.init()

  try:
    rclpy.spin(timeout_nav_node)
  except KeyboardInterrupt:
    pass

  timeout_nav_node.destroy_node()
  rclpy.shutdown()


if __name__ == '__main__':
  main()