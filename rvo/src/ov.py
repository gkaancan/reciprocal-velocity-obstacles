#!/usr/bin/python3
# -*- coding: utf-8 -*-


##################################################
## Reciprocal Velocity Obstacle
##################################################

##################################################
## Author: Göktuğ Kaan CAN
## Email: kaancan-9@hotmail.com
## Status: Intern
##################################################



import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Point
from tf.transformations import euler_from_quaternion
from math import atan2,pi,cos,sin,sqrt
import numpy as np
import time




minkowski_min_x = 0.0
minkowski_min_y = 0.0
minkowski_max_x = 0.0
minkowski_max_y = 0.0

max_speed = 1.0

speed = Twist()
end_points = Point()

t = 5 #saniye cinsinden çarpışmanın algılanmaya başlanacağı süre 

deflection = [0.0,0.0,0.0,0.0] # her bir robot için ayrı sapma değerleri

resolution = 0.001 #çarpışma algılamak için gönderilen ışınların çözünürlüğü , sayi küçüldükçe ışınların oluşturulduğu noktaların sayısı artıyor

class Robot:
    x = 0.0
    y = 0.0
    theta = 0.0
    velocity = 1.0 #saniyede ne yazıyorsa yarısı*m/sn gidiyor 
    
class Point:
    x = 0.0
    y = 0.0

robot11 = Robot()
robot22 = Robot()
robot33 = Robot()
robot44 = Robot()


robots = []
robots.append(robot11)
robots.append(robot22)
robots.append(robot33)
robots.append(robot44)





def points_on_circumference(center=(0,0), r=50.00, n=20):
        return [
        (
            center[0] + (cos(2 * pi / n * x) * r),  # x
            center[1] + (sin(2 * pi / n * x) * r)   # y

        ) for x in range(0, n + 1)]

def points_in_circumference(center=(0,0),r=50.00):
    x = []
    y = []

    a,b = center
    radius = r

    while radius > 0:
        for x1,y1 in points_on_circumference(center=(a,b),r=radius):
            x.append(x1)
            y.append(y1)
        radius = radius - 1 # çevresinin üzerindeki noktaları alınan çemberler arasındaki yarıçap
    return (x,y)    

def get_position_1 (data):
   
    robots[0].x = data.pose.pose.position.x
    robots[0].y = data.pose.pose.position.y
    
    orientation_q = data.pose.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll, pitch, robots[0].theta) = euler_from_quaternion (orientation_list)
    
def get_position_2 (data):
    
    robots[1].x = data.pose.pose.position.x
    robots[1].y = data.pose.pose.position.y
    
    orientation_q = data.pose.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll, pitch, robots[1].theta) = euler_from_quaternion (orientation_list)

def get_position_3 (data):
    
    robots[2].x = data.pose.pose.position.x
    robots[2].y = data.pose.pose.position.y
    
    orientation_q = data.pose.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll, pitch, robots[2].theta) = euler_from_quaternion (orientation_list)

def get_position_4 (data):
    
    robots[3].x = data.pose.pose.position.x
    robots[3].y = data.pose.pose.position.y
    
    orientation_q = data.pose.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll, pitch, robots[3].theta) = euler_from_quaternion (orientation_list)

 

def stop(): # kodu durdurma 
    velocity_pub_1.publish(Twist())
    velocity_pub_2.publish(Twist())
    velocity_pub_3.publish(Twist())
    velocity_pub_4.publish(Twist())


    print("All robots have been stopped")
                       
def go_to_point_2(i,goal_x,goal_y): # RVO kullanıyor & multirobot
    
    global robots
    global deflection
    
    
    global speed
    speed = Twist()
    
    global max_speed
  
    turn = 0.0        
    
    inc_x = goal_x - robots[i-1].x
    inc_y = goal_y - robots[i-1].y
    
    distance = abs(sqrt(inc_x**2+inc_y**2))
    
    angle_to_goal = atan2(inc_y,inc_x)

    
    if not combined_velocity_obstacle(robots[i-1]):
        if robots[i-1].velocity < max_speed:
            robots[i-1].velocity = robots[i-1].velocity + 0.25
        if robots[i-1].velocity > max_speed:
            robots[i-1].velocity = max_speed

        deflection[i-1] =  0.0
        
        if distance > 0.1:


            speed.linear.x = robots[i-1].velocity  
            speed.angular.z = (robots[i-1].theta-angle_to_goal)*3
            

            return speed
        else:
            speed = Twist()
            return speed
    else: #çarpışma gerçekleşecekse program bu else bloğuna giriyor
        early_break = False

     
        while combined_reciprocal_velocity_obstacle(robots[i-1],velocity_vector_endpoint(robots[i-1].x,robots[i-1].y,robots[i-1].theta + deflection[i-1],robots[i-1].velocity)):
            deflection[i-1] = deflection[i-1] + 0.2
            if deflection[i-1] > 2.0:
                early_break = True

        pos_def = deflection[i-1]
        deflection[i-1] = 0.0

        while combined_reciprocal_velocity_obstacle(robots[i-1],velocity_vector_endpoint(robots[i-1].x,robots[i-1].y,robots[i-1].theta - deflection[i-1],robots[i-1].velocity)) and not early_break:
            deflection[i-1] = deflection[i-1] + 0.2
            if deflection[i-1] > 2.0:
                break 
      
        neg_def = deflection[i-1]
        deflection[i-1] = 0.0
                
        if neg_def > pos_def:
            turn = neg_def * -1
        else:
            turn = pos_def    

        if abs(turn) > 1.5 and robots[i-1].velocity > 0:
            robots[i-1].velocity = robots[i-1].velocity - 0.5

        if robots[i-1].velocity <= 0:
            robots[i-1].velocity = 0
            if not combined_reciprocal_velocity_obstacle(robots[i-1],velocity_vector_endpoint(robots[i-1].x,robots[i-1].y,robots[i-1].theta,robots[i-1].velocity+0.5)):
                robots[i-1].velocity = 0.5
 
        print("robot: {}  sapma: {:.3f}, hiz: {:.3f}".format(i,turn,robots[i-1].velocity) )
       
        speed.angular.z = turn*4

        speed.linear.x = robots[i-1].velocity
        
        
        return speed
                       
def minkowski_sum(a,b,robot): # B - A


    minkowski_sum_x = []
    minkowski_sum_y = []


    for points_1 in b[0]: #b
        for points_2 in a[0]:#a
            minkowski_sum_x.append(points_1-(points_2-robot.x)) # b-a

    for points_1 in b[1]: #b
        for points_2 in a[1]: #a
            minkowski_sum_y.append(points_1-(points_2-robot.y)) # b-a
    
    return np.array([minkowski_sum_x,minkowski_sum_y])        

def velocity_vector_endpoint(position_x,position_y,theta,velocity):
    return (position_x + cos(theta)*velocity/2 , position_y + sin(theta)*velocity/2) #velocity ye /2 ekledik çünkü verdiğimiz değerin yarısı hızında gidiyor sebebini anlamadım 

def velocity_obsacle(robot_1,robot_2): # VO AB (Vb)
    
    global t
    global minkowski_min_y,minkowski_max_x,minkowski_max_y,minkowski_min_x
    #global pA_x,pA_y
    robot_A = np.array(points_in_circumference(center=(robot_1.x,robot_1.y),r=0.1))
    robot_B = np.array(points_in_circumference(center=(robot_2.x,robot_2.y),r=0.1))

    minkowski = minkowski_sum(robot_A,robot_B,robot_1) # B - A

    minkowski_min_x = min(minkowski[0])
    minkowski_max_x = max(minkowski[0])

    minkowski_min_y = min(minkowski[1])
    minkowski_max_y = max(minkowski[1])
    
    vA_x , vA_y = velocity_vector_endpoint(robot_1.x,robot_1.y,robot_1.theta,robot_1.velocity) # vA , hız vektörünün bitiş noktası
    vector_A = vA_x - robot_1.x , vA_y - robot_1.y # x2-x1 , y2-y1

    
    vB_x , vB_y = velocity_vector_endpoint(robot_2.x,robot_2.y,robot_2.theta,robot_2.velocity) # vB , hız vektörünün bitiş noktası
    vector_B = vB_x - robot_2.x , vB_y - robot_2.y #x2-x1 , y2-y1
    
    pA_x , pA_y = robot_1.x + (vector_A[0]-vector_B[0])*t , robot_1.y + (vector_A[1]- vector_B[1])*t #λ(p A , v A − v B )
    
    i = 0
    #print("minkowski x [{:.2},{:.2}]  minkowski_y [{:.2},{:.2}]".format(minkowski_min_x,minkowski_max_x,minkowski_min_y,minkowski_max_y))
    #print("pA_x : {:.2}  pA_y : {:.2}".format(pA_x,pA_y))
    while i<1:
        pA_x , pA_y = robot_1.x + (vector_A[0]-vector_B[0])*t*i , robot_1.y + (vector_A[1]- vector_B[1])*t*i #λ(p A , v A − v B )
        if (minkowski_min_x <= pA_x and minkowski_max_x >= pA_x) and (minkowski_min_y < pA_y and minkowski_max_y > pA_y) :
            return True
        i = i + resolution   
    return False    

def combined_velocity_obstacle(robot): # birden fazla robot olduğunda çarpışmanın tespiti için velocity_obsacle fonksiyonunu yardımcı fonkisyon olarak kullanır.
    global robots
    
    for x in robots:
        if robot != x:
            if velocity_obsacle(robot,x):
                return True
    return False            

def combined_reciprocal_velocity_obstacle(robot,new_v1):# birden fazla robot olduğunda 
    global robots
    for x in robots:
        if robot != x:
            if resiprocal_velocity_obsactle(robot,x,new_v1):
                return True
      
    return False

def resiprocal_velocity_obsactle(robot_1,robot_2,new_v1): # new_v1 vektor end point 
    Va = Point()
    new2_Va = Point()

    # 2Va'
    new2_Va.x,new2_Va.y = (new_v1[0] - robot_1.x)*2 , (new_v1[1] - robot_1.y)*2
    # Va
    x,y = velocity_vector_endpoint(robot_1.x,robot_1.y,robot_1.theta,robot_1.velocity)
    Va.x = x - robot_1.x 
    Va.y = y - robot_1.y

    # 2Va'-Va 
    #(new2_Va.x - Va.x) , (new2_Va.y - Va.y)
    if velocity_obsacle_2(robot_1,robot_2,(new2_Va.x - Va.x,new2_Va.y - Va.y)):
        return True
    return False     
               
def velocity_obsacle_2(robot_1,robot_2,V): # VO AB (Vb)
    global t
    global minkowski_min_y,minkowski_max_x,minkowski_max_y,minkowski_min_x
    robot_A = np.array(points_in_circumference(center=(robot_1.x,robot_1.y),r=0.1))
    robot_B = np.array(points_in_circumference(center=(robot_2.x,robot_2.y),r=0.1))
    minkowski = minkowski_sum(robot_A,robot_B,robot_1) # B - A

    minkowski_min_x = min(minkowski[0])
    minkowski_max_x = max(minkowski[0])

    minkowski_min_y = min(minkowski[1])
    minkowski_max_y = max(minkowski[1])

    vector_A = V

    vB_x , vB_y = velocity_vector_endpoint(robot_2.x,robot_2.y,robot_2.theta,robot_2.velocity) # vB
    vector_B = vB_x - robot_2.x , vB_y - robot_2.y #x2-x1 , y2-y1
    i = 0
    while i<1:
        pA_x , pA_y = robot_1.x + (vector_A[0]-vector_B[0])*t*i , robot_1.y + (vector_A[1]- vector_B[1])*t*i #λ(p A , v A − v B )
        if (minkowski_min_x <= pA_x and minkowski_max_x >= pA_x) and (minkowski_min_y < pA_y and minkowski_max_y > pA_y) :
            return True
        i = i + resolution
    return False    

start = time.time()         

rospy.init_node("obstacle_avoidance")

rospy.Subscriber("/robot2/odom",Odometry,get_position_2)
rospy.Subscriber("/robot1/odom",Odometry,get_position_1)
rospy.Subscriber("/robot3/odom",Odometry,get_position_3)
rospy.Subscriber("/robot4/odom",Odometry,get_position_4)


velocity_pub_1 = rospy.Publisher("/robot1/cmd_vel",Twist,queue_size=10)
velocity_pub_2 = rospy.Publisher("/robot2/cmd_vel",Twist,queue_size=10)
velocity_pub_3 = rospy.Publisher("/robot3/cmd_vel",Twist,queue_size=10)
velocity_pub_4 = rospy.Publisher("/robot4/cmd_vel",Twist,queue_size=10)


r = rospy.Rate(250)

rospy.on_shutdown(stop)

while not rospy.is_shutdown():
    
    
    if time.time() - start > 3:
        velocity_pub_1.publish(go_to_point_2(1,5,5))
        velocity_pub_2.publish(go_to_point_2(2,0,0))
        velocity_pub_3.publish(go_to_point_2(3,5,0))
        velocity_pub_4.publish(go_to_point_2(4,0,5))


          
    r.sleep()
