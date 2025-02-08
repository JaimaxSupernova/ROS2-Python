import rclpy
import math
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

Kp_ang = 0.02
Ki_ang = 0.015
Kd_ang = 0.08
Kangmultiplier = 0.0301

Kp_dist= 0.7
Ki_dist = 0.008
#was 0.008
Kd_dist = 0.35
Kdistmultiplier = 0.18

class MyRobot(Node):

    def __init__(self):
        super().__init__('myrobot_node')
        qos_policy = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
                                           history=rclpy.qos.HistoryPolicy.KEEP_LAST, depth=1)
        ## This QOS_POLICY needs to go before the laser subscription in your code. ##
        self.sub1 = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            qos_profile=qos_policy
        )
        ## The QOS_POLICY needs to be added to the call back. ##
        self.sub2 = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10
        )
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        timer_period = 0.2
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.my_x = 0
        self.my_y = 0
        self.my_th = 0
        self.err_x = 0
        self.err_y = 0
        self.err_th = 0
        self.my_ls = LaserScan()
        self.step=1
        self.velo = 0.0
        self.angvelo = 0.0

        
    def pid_angle(self,actualang, desiredang):
        previousError = 0.0
        Integral = 0.0

        Error = desiredang - actualang
        # if (Error <= -180 ):
        #     Error = Error + 180
        # elif (Error >=180):
        #     Error = Error-180
        Integral = Integral + Error
        Derivative = Error - previousError
        output = (Kp_ang*Error)+(Ki_ang*Integral)+(Kd_ang*Derivative)
        previousError=Error
        return output*Kangmultiplier

    def pid_distance(self, dx, dy, ax, ay):
        previousError = 0.0
        Integral = 0.0

        Error = math.sqrt((dx-ax)**2+ (dy-ay)**2)
        Integral = Integral + Error
        Derivative = Error - previousError
        output = (Kp_dist*Error)+(Ki_dist*Integral)+(Kd_dist*Derivative)
        previousError=Error
        return output*Kdistmultiplier


    def scan_callback(self, msg):
        self.my_ls.ranges = msg.ranges

    def odom_callback(self, msg):
        self.my_x = msg.pose.pose.position.x - self.err_x
        self.my_y = msg.pose.pose.position.y - self.err_y
        # convert quaternian to Euler angles
        q0 = msg.pose.pose.orientation.w
        q1 = msg.pose.pose.orientation.x - self.err_x
        q2 = msg.pose.pose.orientation.y - self.err_y
        q3 = msg.pose.pose.orientation.z
        self.my_th = (math.atan2(2 * (q0 * q3 + q1 * q2), (1 - 2 * (q2 * q2 + q3 * q3))) * 180 / math.pi) - self.err_th
        print ('x = {0:5.2f}, y = {1:5.2f}, th = {2:5.2f}'.format(self.my_x, self.my_y, self.my_th))
            
    def timer_callback(self):
        move = Twist()
        if(self.step==1):
            if(self.my_x>=0.99):
                move.linear.x = 0.0
                if(not (self.my_th>=88.5 and self.my_th<=91.5)):  
                    move.angular.z = self.pid_angle(float(self.my_th),90)
                    print(str(move.angular.z) + str(move.linear.x)+ "\n")
                else:
                    print(str(move.angular.z) + "\n\n\n\n\n")
                    move.angular.z = 0.0    
                    self.step=2 
                    print(str(move.angular.z) + "\n\n\n\n\n")
            else:
                move.linear.x =self.pid_distance(1.1,0,self.my_x,self.my_y)
                move.angular.z = 0.0
        elif(self.step==2):
            if(self.my_y>=0.99):
                move.linear.x = 0.0
                if(not ((self.my_th>=178.5 and self.my_th<=180) or (self.my_th<=-178.5 and self.my_th>=-180))):         
                    move.angular.z =self.pid_angle(abs(self.my_th),180)
                else:
                    move.angular.z = 0.0    
                    self.step=3 
            else:
                move.linear.x = self.pid_distance(1.1,1.1,self.my_x,self.my_y)
                move.angular.z = 0.0
        elif(self.step==3):
            if(self.my_x<=0.01):
                move.linear.x = 0.0
                if(not (self.my_th<=-88.5 and self.my_th>=-91.5)):         
                    move.angular.z = abs(self.pid_angle(float(self.my_th),-90))
                else:
                    move.angular.z = 0.0    
                    self.step=4
            else:
                move.linear.x = self.pid_distance(0,1.1,self.my_x,self.my_y)
                move.angular.z = 0.0
        elif(self.step==4):
            if(self.my_y<=0.01):
                move.linear.x = 0.0
                if( not ((self.my_th>=-1.5 and self.my_th<=0) or (self.my_th<=1.5 and self.my_th>=0))):         
                    move.angular.z = self.pid_angle(float(self.my_th),0)
                else:
                    move.angular.z = 0.0    
                    self.step=0 
            else:
                move.linear.x = self.pid_distance(0,0,self.my_x,self.my_y)
                move.angular.z = 0.0
        else:
            move.linear.x = 0.0

        self.pub.publish(move)
        self.velo = move.linear.x
        self.angvelo = move.angular.z
        self.Moving()
 
    def Moving(self):
        #clear file to initialise
        open('file.txt', 'w').close()
        # open a data file in the M-Drive, you need it to change to yours
        myfile = open("src/ce215_pkg/ce215_pkg/myodom.txt", "a")
        # your data saving code should be here
        # close the data file
        d = ", "
        myfile.write((f'{self.my_x}{d}{self.my_y}{d}{self.my_th}{d}{self.velo}{d}{self.angvelo}\n'))
        myfile.close
    def Clearfile(self):
        myfileclr = open("src/ce215_pkg/ce215_pkg/myodom.txt", "w")
        myfileclr.close



def main(args=None):
    rclpy.init(args=args)
    myrobot_node = MyRobot()
    myrobot_node.Clearfile()
    myrobot_node.Moving()
    rclpy.spin(myrobot_node)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    myrobot_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

