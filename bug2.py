import math
import rclpy
from rclpy.node import Node
import tf_transformations as transform
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import Range
from nav_msgs.msg import Odometry


# BUG2 ALGORITAM
# Martin Ključarić 0035239671 3-MiR-3



class Bug2(Node):
    #Initialization
    def __init__(self):
        super().__init__('bugx')
        self.odom_subscriber = self.create_subscription(Odometry, '/odom', self.location_callback, 10)
        self.fl_sensor_sub = self.create_subscription(Range, '/fl_range_sensor', self.fl_sensor_callback, 10)
        self.fr_sensor_sub = self.create_subscription(Range, '/fr_range_sensor', self.fr_sensor_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.target_sub = self.create_subscription(PoseStamped, '/goal_pose', self.goal_callback, 10)
        self.bug_algorithm_timer = self.create_timer(0.1, self.bug_algorithm_callback)
        self.goal_x = None
        self.goal_y = None
        self.current_x = None
        self.current_y = None
        self.current_theta = None
        self.fl_sensor_value = 0.0
        self.fr_sensor_value = 0.0
        self.isNormalized = False
        self.isPastObstacle = False
        self.corneringDone = False
        self.mAfterCornering = False
        self.normal_threshold = 0.05
        self.cornering_omega = 0.5
        self.cornering_radius = 0.6
        self.corner_theta = None
        self.cornering_vel = self.cornering_radius * self.cornering_omega
        self.cmd_vel_msg = Twist()
        self.obstacle_threshold = 0.45 # Threshold distance to consider an obstacle
        self.state = 'MOVE_TO_GOAL' #'MOVE_TO_GOAL' 'FOLLOW_OBSTACLE' 'CORNERING' 'NORMALIZING'

    #Method for goal update
    def goal_callback(self, msg):
        self.goal_x = msg.pose.position.x
        self.goal_y = msg.pose.position.y
        self.x0_m = self.current_x
        self.y0_m = self.current_y

    #Method for robot current position
    def location_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        q = (
                msg.pose.pose.orientation.x,
                msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z,
                msg.pose.pose.orientation.w)
        self.current_theta = transform.euler_from_quaternion(q)[2] #[-pi, pi]

    #Methods for updating sensor values
    def fl_sensor_callback(self, msg):
        self.fl_sensor_value = msg.range
        print(self.fl_sensor_value)   
    def fr_sensor_callback(self, msg):
        self.fr_sensor_value = msg.range
        print(self.fr_sensor_value)  
    def bug_algorithm_callback(self):
        if self.goal_x is None or self.goal_y is None or self.current_x is None or self.current_y is None:
            return     
        # telemetrija 
        print("Current X: " , self.current_x)
        print("Current Y: " , self.current_y)
        print("Current theta: " , self.current_theta)
        print("FL Sensor: " , self.fl_sensor_value)
        print("FR Sensor: " , self.fr_sensor_value)
        print("Goal X: " , self.goal_x)
        print("Goal Y: " , self.goal_y)
        print("Currently doing: ", self.state)
        print("StartM ",(self.x0_m, self.y0_m), " EndM ", (self.goal_x, self.goal_y))
        


        goal_distance = math.sqrt((self.goal_x - self.current_x) ** 2 + (self.goal_y - self.current_y) ** 2)
        goal_angle = math.atan2(self.goal_y - self.current_y, self.goal_x - self.current_x)
        # Setiranje načina rada algoritma - idi do cilja, postavi se okomito na prepreku, prati prepreku, obiđi ćošak
        if goal_distance < 0.15:        # uvjet za kraj programa
            self.cmd_vel_msg.linear.x = 0.0
            self.cmd_vel_msg.angular.z = 0.0
            self.cmd_pub.publish(self.cmd_vel_msg)
            self.isNormalized = False
            self.isPastObstacle = False
            self.corneringDone = False
            self.mAfterCornering = False
            self.state = 'MOVE_TO_GOAL'
            print("Goal reached!")
            return

        if self.state == 'MOVE_TO_GOAL' :   # uđi u normalising kada senzor pređe threshold i robot gleda u smjeru cilja (1. praćenje prepreke)
            if (self.fl_sensor_value < self.obstacle_threshold or self.fr_sensor_value < self.obstacle_threshold) and abs(self.current_theta - goal_angle) < 0.05:
                self.cmd_vel_msg.linear.x = 0.0
                self.cmd_vel_msg.angular.z = 0.0
                self.cmd_pub.publish(self.cmd_vel_msg)
                self.state = 'NORMALIZING'
                self.normalize()            # uđi u normalising kada senzor pređe threshold i aktiviran je flag (2.+ praćenje prepreke)
            elif (self.fl_sensor_value < self.obstacle_threshold or self.fr_sensor_value < self.obstacle_threshold) and self.mAfterCornering:
                self.cmd_vel_msg.linear.x = 0.0
                self.cmd_vel_msg.angular.z = 0.0
                self.cmd_pub.publish(self.cmd_vel_msg)
                self.state = 'NORMALIZING'
                self.normalize()
            else:                                                                               # idi prema cilju
                self.move_to_goal(goal_angle)

        if self.state == 'NORMALIZING':
            if self.isNormalized == True:                                                       # ako si normaliziran gasi sve motore i prelazi u praćenje prepreke
                self.cmd_vel_msg.linear.x = 0.0
                self.cmd_vel_msg.angular.z = 0.0
                self.cmd_pub.publish(self.cmd_vel_msg)
                self.state = 'FOLLOW_OBSTACLE'
                self.follow_obstacle()
            else:                                                                               # inače nastavi normalizing
                self.normalize()

        if self.state == 'FOLLOW_OBSTACLE':
            if self.isPastObstacle == True:                                                     # ako si prešao prepreku gasi sve motore i prelazi u obilaženje ćoškova
                self.cmd_vel_msg.linear.x = 0.0
                self.cmd_vel_msg.angular.z = 0.0
                self.cmd_pub.publish(self.cmd_vel_msg)
                self.state = 'CORNERING'
                self.corner()
            
            elif abs(self.current_y - self.m_line(self.current_x)) < 0.1 and self.mAfterCornering:  #ako si na m liniji ali ne prvi put prebaci se na move to goal
                self.cmd_vel_msg.linear.x = 0.0
                self.cmd_vel_msg.angular.z = 0.0
                self.cmd_pub.publish(self.cmd_vel_msg)
                self.state = 'MOVE_TO_GOAL'
                self.move_to_goal(goal_angle)
            
            else:
                self.follow_obstacle()

        if self.state == 'CORNERING':
            if self.corneringDone == True:                                                      # kada je cornering gotov ponovo normaliziraj
                self.cmd_vel_msg.linear.x = 0.0
                self.cmd_vel_msg.angular.z = 0.0
                self.cmd_pub.publish(self.cmd_vel_msg)
                self.isNormalized = False
                self.isPastObstacle = False
                self.corneringDone = False
                self.state = 'NORMALIZING'
                self.normalize()
            elif abs(self.current_y - self.m_line(self.current_x)) < 0.1:                       # ako se tijekom corneringa nađeš na m liniji idi prema cilju
                self.cmd_vel_msg.linear.x = 0.0
                self.cmd_vel_msg.angular.z = 0.0
                self.cmd_pub.publish(self.cmd_vel_msg)
                self.state = 'MOVE_TO_GOAL'
                self.move_to_goal(goal_angle)
                self.mAfterCornering = False
            else:
                self.corner()               # inače idi prema cilju i setiraj flag (prevencija aktivacije move to goal kada se prvi put detektira prepreka)
                self.mAfterCornering = True
            

    def m_line(self, x):                                                                        # matematička formulacija m linije
        y = (self.goal_y - self.y0_m) / (self.goal_x - self.x0_m) * (x - self.x0_m) + self.y0_m
        return y

    def normalize(self):    # algoritam za postavljanje u orijentaciju normalnu u odnosu na površinu prepreke - muči ga ako je prepreka prekratka (nema orijentacije za koji se vrijednosti senzora ujednače pa bude u loopu)
        if(abs(self.fl_sensor_value - self.fr_sensor_value) < self.normal_threshold):
            if (math.cos(math.pi/4) * self.fl_sensor_value > 0.3) and (math.cos(math.pi/4) * self.fl_sensor_value > 0.3):
                self.cmd_vel_msg.angular.z = 0.0
                self.cmd_vel_msg.linear.x = 0.1
                self.cmd_pub.publish(self.cmd_vel_msg)      # u ovom dijelu se robot približava prepreci na određenu udaljenost nakon što se postavi okomito na nju
            else:                                           # pomaže spriječiti prerano okidanje ulaska u cornering
                self.isNormalized = True                    # kada se postavi na ispravnu udaljenost postavlja se relevantni flag, pamti se orijentacija normalna...
                self.normalTheta = self.current_theta       # ... na prepreku i gase se motori
                self.cmd_vel_msg.angular.z = 0.0
                self.cmd_vel_msg.linear.x = 0.0
                self.cmd_pub.publish(self.cmd_vel_msg)
            return
        if(self.fl_sensor_value - self.fr_sensor_value) < -0.05:                                # dva ifa koji tjeraju robota u okomitu orijentaciju
            self.cmd_vel_msg.linear.x = 0.0
            self.cmd_vel_msg.angular.z = 0.1
            self.cmd_pub.publish(self.cmd_vel_msg)
            return
        if(self.fl_sensor_value - self.fr_sensor_value) > 0.05:
            self.cmd_vel_msg.linear.x = 0.0
            self.cmd_vel_msg.angular.z = -0.1
            self.cmd_pub.publish(self.cmd_vel_msg)
            return

    def follow_obstacle(self):                                                                  # algoritam za praćenje prepreke, skoro identičan kao praćenje cilja
        self.cmd_vel_msg.linear.x = 0.0 
        angle_diff = self.normalize_angle((self.normalTheta-math.pi/2) - self.current_theta)    # u odnosu na kut okomitosti postavi se 90° na prepreku
        if abs(angle_diff) > 0.1:
            self.cmd_vel_msg.linear.x = 0.0
            if angle_diff >0:
                self.cmd_vel_msg.angular.z = 0.1
            else:
                self.cmd_vel_msg.angular.z = -0.1
        else:
            if self.fl_sensor_value < 0.75:                                                     # quick and dirty threshold za detekciju prelaska prepreke
                self.cmd_vel_msg.linear.x = 0.15
            else:
                self.isPastObstacle = True                                                      # dizanje relevantnog flaga i gašenje motora
                self.corner_theta = self.current_theta
                self.cmd_vel_msg.linear.x = 0.0
                self.cmd_vel_msg.angular.z = 0.0
        
        self.cmd_pub.publish(self.cmd_vel_msg)
        return
    
    def corner(self):                                                                           # fuck it we ball
        angle_diff = self.normalize_angle((self.corner_theta + math.pi) - self.current_theta)
        if abs(angle_diff) > 0.1:
            self.cmd_vel_msg.linear.x = self.cornering_vel
            self.cmd_vel_msg.angular.z = self.cornering_omega
        else:
            self.cmd_vel_msg.linear.x = 0.0
            self.cmd_vel_msg.angular.z = 0.0
            self.corneringDone = True
        
        self.cmd_pub.publish(self.cmd_vel_msg)
        return
        
    def move_to_goal(self, goal_angle):                                                         # algoritam za praćenje cilja
        self.cmd_vel_msg.linear.x = 0.0 
        angle_diff = self.normalize_angle(goal_angle - self.current_theta)
        if abs(angle_diff) > 0.1:
            self.cmd_vel_msg.linear.x = 0.0
            if angle_diff >0:
                self.cmd_vel_msg.angular.z = 0.3
            else:
                self.cmd_vel_msg.angular.z = -0.3
            
        else:
            if self.fl_sensor_value >= self.obstacle_threshold and self.fr_sensor_value >= self.obstacle_threshold:   
                self.cmd_vel_msg.linear.x = 0.2    
                self.cmd_vel_msg.angular.z = 0.0
        self.cmd_pub.publish(self.cmd_vel_msg)


    def normalize_angle(self, angle):                                                           # mala funkcija za pretvaranje kuteva u prihvatljiv oblik
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle
            
def main(args=None):

    rclpy.init(args=args)
    bug_node = Bug2()
    rclpy.spin(bug_node)
    bug_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()