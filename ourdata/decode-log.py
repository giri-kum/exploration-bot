import sys
import lcm
import numpy as np
import matplotlib.pyplot as plt

# put lcm types you need here
# run make once to generate python lcm types
from lcmtypes import pose_xyt_t, particles_t

if len(sys.argv) < 2:
    sys.stderr.write("usage: decode-log <logfile>\n")
    sys.exit(1)

log = lcm.EventLog(sys.argv[1], "r")

#font = {'family' : 'normal',
#        'weight' : 'bold',
#        'size'   : 22}
#plt.rc('font', **font)
plt.rcParams.update({'font.size': 15})


true_x = []
true_y = []
true_theta = []
true_utime = []

slam_x = []
slam_y = []
slam_theta = []
slam_utime = []

get_particles = True
particle_x = []
particle_y = []
particle_theta = []
particles_x = []
particles_y = []
particles_theta = []
times = [1512686876964601,1512686878418152,1512686881061837,1512686882443644,1512686884423482,1512686886835653,1512686888304836,1512686889767772]
for event in log:
    if event.channel == "TRUE_POSE":
        msg = pose_xyt_t.decode(event.data)
        print("TruePose:")
        print("timestamp= %d" % msg.utime)
        print("pose: (%f, %f, %f)" % (msg.x, msg.y, msg.theta))
        true_utime.append(msg.utime)
        true_x.append(msg.x)
        true_y.append(msg.y)
        true_theta.append(msg.theta)
    
    if event.channel == "SLAM_POSE":
        msg = pose_xyt_t.decode(event.data)
        p = pose_xyt_t.decode(event.data)
        print("SLAMPose:")
        print("timestamp= %d" % msg.utime)
        print("pose: (%f, %f, %f)" % (msg.x, msg.y, msg.theta))
        slam_utime.append(msg.utime)
        slam_x.append(msg.x)
        slam_y.append(msg.y)
        slam_theta.append(msg.theta)

    if event.channel == "SLAM_PARTICLES" and get_particles:
        msg = particles_t.decode(event.data)
        print("SLAMParticles:")
        tol = 0.01
        print("pose: (%f, %f, %f)" % (p.x, p.y, p.theta))
        
        if( p.utime in times):
            for i in range(msg.num_particles):
                particle_x.append(msg.particles[i].pose.x)
                particle_y.append(msg.particles[i].pose.y)
                particle_theta.append(msg.particles[i].pose.theta)
            particles_x.append(particle_x)
            particles_y.append(particle_y)
            particles_theta.append(particle_theta)

plt.figure(1)
plt.plot(true_x, true_y, label = 'true_trajectory')
plt.plot(slam_x, slam_y, label = 'slam_trajectory')
plt.legend(loc = 'lower left')

error_x = []
error_y = []
error_theta = []
error_utime = []

print(len(slam_x))
print(len(true_x))

# Interpolation
for i in range(len(true_utime)):
    # find bounds
    found_bounds = 0
    for j in range(len(slam_utime)):
        if (slam_utime[j] > true_utime[i]):
            lower_bound = slam_utime[j-1]
            upper_bound = slam_utime[j]
            ind = j
            break

    interp = (float)(true_utime[i] - lower_bound) / (float)(upper_bound - lower_bound)
    #print upper_bound - lower_bound, " ", true_utime[i] - lower_bound, " ", interp

    #print true_x[i], " ", slam_x[j-1] + (slam_x[j] - slam_x[j-1])*interp, " ", true_x[i] - slam_x[j-1] + (slam_x[j] - slam_x[j-1])*interp

    error_x.append(true_x[i] - slam_x[j-1] + (slam_x[j] - slam_x[j-1])*interp)
    error_y.append(true_y[i] - slam_y[j-1] + (slam_y[j] - slam_y[j-1])*interp)
    error_theta.append(true_theta[i] - slam_theta[j-1] + (slam_theta[j] - slam_theta[j-1])*interp)
    error_utime.append(true_utime[i])


    '''
    loc = true_utime.index(slam_utime[i])
    error_x.append(true_x[loc]-slam_x[i])
    error_y.append(true_y[loc]-slam_y[i])
    error_theta.append(true_theta[loc]-slam_theta[i])
    error_utime.append(slam_utime[i])
    '''

# Compute stats
mean_x = np.mean(error_x)
mean_y = np.mean(error_y)
mean_theta = np.mean(error_theta)
std_x = np.std(error_x)
std_y = np.std(error_y)
std_theta = np.std(error_theta)
max_x = np.max(np.absolute(error_x))
max_y = np.max(np.absolute(error_y))
max_theta = np.max(np.absolute(error_theta))

for i in range (len(error_utime)):
    error_utime[i] = (float)(error_utime[i]) / pow(10,6) - 1.51181653e9


# Print statistics
print "error_x: mean = ", mean_x, " , std = ", std_x, " , max = ", max_x
print "error_y: mean = ", mean_y, " , std = ", std_y, " , max = ", -max_y
print "error_theta: mean = ", mean_theta, " , std = ", std_theta, " , max = ", -max_theta

# Create std lines
std_x_upper = np.ones(len(error_x)) * (mean_x + 3*std_x)
std_x_lower = np.ones(len(error_x)) * (mean_x - 3*std_x)
std_y_upper = np.ones(len(error_y)) * (mean_y + 3*std_y)
std_y_lower = np.ones(len(error_y)) * (mean_y - 3*std_y)
std_theta_upper = np.ones(len(error_theta)) * (mean_theta + 3*std_theta)
std_theta_lower = np.ones(len(error_theta)) * (mean_theta - 3*std_theta)

if(get_particles):
    for i in range(len(particles_x)):
        psx = particles_x[i]
        psy = particles_y[i]

        for j in range(len(psx)):
            plt.plot(psx[j],psx[j],'r.')
        print("Particles plotted")        



t1 = true_utime
t2 = slam_utime

k = float(1e6)
t1 = [float(i/k) - 1.51181653e9 for i in t1]
t2 = [float(i/k) - 1.51181653e9 for i in t2]


plt.figure(2)
plt.subplot(311)
plt.plot(t1, true_x, label = 'true_x')
plt.plot(t2, slam_x, label = 'slam_x')
plt.legend(loc = 'lower left')
plt.ylabel('x (m)')
plt.subplot(312)
plt.plot(t1, true_y, label = 'true_y')
plt.plot(t2, slam_y, label = 'slam_y')
plt.legend(loc = 'lower left')
plt.ylabel('y (m)')
plt.subplot(313)
plt.plot(t1, true_theta, label = 'true_theta')
plt.plot(t2, slam_theta, label = 'slam_theta')
plt.legend(loc = 'lower left')
plt.ylabel('theta (rads)')
plt.xlabel('time (s)')


plt.figure(3)
plt.subplot(311)
plt.plot(error_utime, error_x, label = 'error_x')
plt.plot(error_utime, std_x_upper, label = '3*std_x', color = 'red')
plt.plot(error_utime, std_x_lower, color = 'red')
plt.legend(loc = 'lower left')
plt.ylabel('x error (m)')

plt.subplot(312)
plt.plot(error_utime, error_y, label = 'error_y')
plt.plot(error_utime, std_y_upper, label = '3*std_y', color = 'red')
plt.plot(error_utime, std_y_lower, color = 'red')
plt.legend(loc = 'lower left')
plt.ylabel('y error (m)')

plt.subplot(313)
plt.plot(error_utime, error_theta, label = 'error_theta')
plt.plot(error_utime, std_theta_upper, label = '3*std_theta', color = 'red')
plt.plot(error_utime, std_theta_lower, color = 'red')
plt.legend(loc = 'lower left')
plt.ylabel('theta error (rads)')
plt.xlabel('time (s)')


plt.show()
