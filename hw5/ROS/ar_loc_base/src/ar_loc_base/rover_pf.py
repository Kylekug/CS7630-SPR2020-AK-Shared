import roslib; roslib.load_manifest('ar_loc_base')
import rospy
from numpy import *
from numpy.linalg import pinv, inv
from math import pi, sin, cos
from geometry_msgs.msg import *
import tf
import bisect
import threading

from rover_kinematics import *

class RoverPF(RoverKinematics):
    def __init__(self, initial_pose, initial_uncertainty):
        RoverKinematics.__init__(self)
        self.initial_uncertainty = initial_uncertainty
        self.lock = threading.Lock()
        self.X = mat(vstack(initial_pose))
        # Initialisation of the particle cloud around the initial position
        self.N = 500
        self.particles = [self.X + self.drawNoise(initial_uncertainty) for i in range(0,self.N)]
        self.pa_pub = rospy.Publisher("~particles",PoseArray,queue_size=1)

    def getRotation(self, theta):
        R = mat(zeros((2,2)))
        R[0,0] = cos(theta); R[0,1] = -sin(theta)
        R[1,0] = sin(theta); R[1,1] = cos(theta)
        return R
    
    # Draw a vector uniformly around [0,0,0], scaled by norm
    def drawNoise(self, norm):
        if type(norm)==list:
            return mat(vstack(norm)*(2*random.rand(3,1)-vstack([1,1,1])))
        else:
            return mat(multiply(norm,((2*random.rand(3,1)-vstack([1,1,1])))))


    def predict(self, motor_state, drive_cfg, encoder_precision):
        self.lock.acquire()
        # The first time, we need to initialise the state
        if self.first_run:
            self.motor_state.copy(motor_state)
            self.first_run = False
            self.lock.release()
            return 
        # Prepare odometry matrices (check rover_odo.py for usage)
        iW = self.prepare_inversion_matrix(drive_cfg)
        S = self.prepare_displacement_matrix(self.motor_state,motor_state,drive_cfg)
        self.motor_state.copy(motor_state)

        # Apply the particle filter prediction step here
        # TODO

        # self.particles = ...
        dX = iW * S #stole this from compute_displacement
        
        for k,v in enumerate(self.particles): #stole this from integrate_odometry
            t_part = self.particles[k][2,0]
            self.particles[k][0,0] += dX[0,0]*cos(t_part) - dX[1,0]*sin(t_part)
            self.particles[k][1,0] += dX[0,0]*sin(t_part) + dX[1,0]*cos(t_part)
            self.particles[k][2,0] += dX[2,0]
        
        self.lock.release()

    def update_ar(self, Z, L, Uncertainty):
        self.lock.acquire()
        print "Update: L="+str(L.T)
        # Implement particle filter update using landmarks here
        # Note: the function bisect.bisect_left could be useful to implement
        # the resampling process efficiently
        # TODO

        # self.particles = ...
        
        
        z_part = [None] * (len(self.particles))
        z_world = mat(zeros((2,1)))
        
        normSum = 0
        temp_part = self.particles
        
        for k,v in enumerate(self.particles):
            
            #before messing with anything, let's save our particles

            
            #let's calculate observations we'd expect from each particle,
            #define vector in world frame w/ delta x and delta y to L,
            z_world[0,0] = L[0,0] - self.particles[k][0,0]
            z_world[1,0] = L[1,0] - self.particles[k][1,0]
            
            #rotate by -theta to get robo-frame (particle-frame), 
            t_part = -self.particles[k][2,0]
            z_part[k] = mat(zeros((2,1)))
            z_part[k][0,0] = z_world[0,0]*cos(t_part) - z_world[1,0]*sin(t_part)
            z_part[k][1,0] = z_world[0,0]*sin(t_part) + z_world[1,0]*cos(t_part)
            
            #let's now get the vector difference between expected observations
            #and actual observation, (we'll store it in z_part); then we get
            #magnitude
            z_part[k] = numpy.linalg.norm(z_part[k] - Z)
            
            #let's now calculate the weights we'll be using for prob.,
            #use Guassian prob. density:
            sigma = Uncertainty
            normalizeCoeff = 1 / (sigma * pow(2*pi,0.5))
            z_part[k] = normalizeCoeff * exp(-0.5 * pow(z_part[k],2) / pow(sigma,2))
            normSum += z_part[k]
            
        z_part = z_part / normSum
        
        #if (z_part.index(NaN)): print z_part
        
        #todo: sometimes z_part gets a NaN; suspect might be related to getting a 
        #"perfect" measurement on occasion? may just want to do some rough error checking
        randIndex = numpy.random.choice(range(len(self.particles)), self.N, p=z_part)
        
        for k,v in enumerate(randIndex): 
            self.particles[k] = temp_part[v] + self.drawNoise(Uncertainty)
        
        self.lock.release()

    def update_compass(self, angle, Uncertainty):
        self.lock.acquire()
        print "Update: C="+str(angle)
        # Implement particle filter update using landmarks here
        # Note: the function bisect.bisect_left could be useful to implement
        # the resampling process efficiently
        # TODO

        # self.particles = ...
        
        self.lock.release()

    def updateMean(self):
        X = mat(zeros((3,1)))
        for x in self.particles:
            X += x
        self.X = X / len(self.particles)
        
        return self.X

    def publish(self, pose_pub, target_frame, stamp):
        # Only compute the mean for plotting
        self.updateMean()
        pose = PoseStamped()
        pose.header.frame_id = target_frame
        pose.header.stamp = stamp
        pose.pose.position.x = self.X[0,0]
        pose.pose.position.y = self.X[1,0]
        pose.pose.position.z = 0.0
        Q = tf.transformations.quaternion_from_euler(0, 0, self.X[2,0])
        pose.pose.orientation.x = Q[0]
        pose.pose.orientation.y = Q[1]
        pose.pose.orientation.z = Q[2]
        pose.pose.orientation.w = Q[3]
        pose_pub.publish(pose)

        pa = PoseArray()
        pa.header = pose.header
        for p in self.particles:
            po = Pose()
            po.position.x = p[0,0]
            po.position.y = p[1,0]
            q = tf.transformations.quaternion_from_euler(0, 0, p[2,0])
            po.orientation = Quaternion(*q)
            pa.poses.append(po)
        self.pa_pub.publish(pa)

    def broadcast(self,br, target_frame, stamp):
        br.sendTransform((self.X[0,0], self.X[1,0], 0),
                     tf.transformations.quaternion_from_euler(0, 0, self.X[2,0]),
                     stamp, "/%s/ground"%self.name, target_frame)
        

