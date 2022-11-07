import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import interp1d

def showTrajectoryBicycle(time, states, G):
    # Show the trajectory
    plt.figure(figsize=(10,5))
    plt.subplot(3,2,1)
    plt.plot(time, states[0:-1,1])
    plt.grid()
    plt.xlabel('Time')
    plt.ylabel('y')

    plt.subplot(3,2,3)
    plt.plot(time, states[0:-1,2])
    plt.grid()
    plt.xlabel('Time')
    plt.ylabel("$\\theta$")

    plt.subplot(3,2,5)
    plt.plot(time, G)
    plt.grid()
    plt.xlabel('Time')
    plt.ylabel('$\gamma$')


    plt.subplot(1,2,2)
    plt.plot(states[:,0], states[:,1])
    plt.xlabel('x')
    plt.ylabel('y')
    plt.axis('equal')
    plt.grid()

    plt.tight_layout()
    plt.show()

def plotTrajectoryGoal(time, states, goal, V, G):
    # Make the polygon for the orientation of the robot
    th = states[0,2] 
    R = np.array([[np.cos(th), -np.sin(th)], [np.sin(th), np.cos(th)]])
    v = np.array([[-1,-1],[-1,1],[2,0]])
    vT = np.dot(v,R.T)    
    
    # Show the trajectory
    plt.figure(figsize=(10,5))
    plt.subplot(2,2,2)
    plt.plot(time, V)
    plt.grid()
    plt.xlabel('Time')
    plt.ylabel('Speed')
    plt.subplot(2,2,4)
    plt.plot(time, G)
    plt.grid()
    plt.xlabel('Time')
    plt.ylabel('$\gamma$')
    
    
    cmap = plt.cm.get_cmap("jet")
    
    plt.subplot(1,2,1)
    plt.plot(states[0,0],states[0,1], 'g', marker=vT, markersize=25, alpha=.5)
    plt.plot(goal[0],goal[1], 'ro')
    #plt.plot(states[:,0], states[:,1], '-', )
    T = states.shape[0]
    for i in range(T-1):
        plt.plot(states[i:i+2,0], states[i:i+2,1], color=cmap((255*i)//T))

    
    plt.axis('equal')
    plt.grid()
    
    plt.tight_layout()
    plt.show()

def linePoints(line,ref = [-1.,1.]):
    a,b,c = line
    if (a==0 and b==0):
        raise Exception("linePoints: a and b cannot both be zero")
    return [((-c-b*p)/a,p) if abs(a)>abs(b) else (p,(-c-a*p)/b) for p in ref]

def plotTrajectoryLine(time, states, line, V, G):
    # Make the polygon for the orientation of the robot
    th = states[0,2] 
    R = np.array([[np.cos(th), -np.sin(th)], [np.sin(th), np.cos(th)]])
    v = np.array([[-1,-1],[-1,1],[2,0]])
    vT = np.dot(v,R.T)    
    
    # Show the trajectory
    plt.figure(figsize=(10,5))
    plt.subplot(2,2,2)
    plt.plot(time, V)
    plt.grid()
    plt.xlabel('Time')
    plt.ylabel('Speed, $v$')
    plt.subplot(2,2,4)
    plt.plot(time, G)
    plt.grid()
    plt.xlabel('Time')
    plt.ylabel('Steering angle, $\gamma$')
    
    plt.subplot(1,2,1)
    plt.plot(states[0,0],states[0,1], 'g', marker=vT, markersize=25, alpha=.5)
    plt.gca().axline(*linePoints(line, [0,10]),color="orange")
    plt.plot(states[:,0], states[:,1], '-')
    plt.axis('equal')
    plt.grid()
    
    plt.tight_layout()
    plt.show()

def plotTrajectoryMovingGoal(time, states, Goal, V, G):
    # Make the polygon for the orientation of the robot
    th = states[0,2] 
    R = np.array([[np.cos(th), -np.sin(th)], [np.sin(th), np.cos(th)]])
    v = np.array([[-1,-1],[-1,1],[2,0]])
    vT = np.dot(v,R.T)    
    
    # Show the trajectory
    plt.figure(figsize=(10,5))
    plt.subplot(2,2,2)
    plt.plot(time, V)
    plt.grid()
    plt.xlabel('Time')
    plt.ylabel('Speed')
    plt.subplot(2,2,4)
    plt.plot(time, G)
    plt.grid()
    plt.xlabel('Time')
    plt.ylabel('$\gamma$')
    
    plt.subplot(1,2,1)
    plt.plot(states[0,0],states[0,1], 'g', marker=vT, markersize=25, alpha=.5)
    plt.plot(Goal[:,0],Goal[:,1], 'r--')
    plt.plot(states[:,0], states[:,1], '-')
    plt.axis('equal')
    plt.grid()
    
    plt.tight_layout()
    plt.show()

def trajectoryGenerator(waypoints, T, N, kind = 'linear'):
    # waypoints is a mx2 numpy array containing the x,y coordinates of the waypoints
    # T is the total time for the trajectory
    # N is the total number of sample points
    # kind is the interpolation method used by scipy.interpolate.interp1d ('linear', 'quadratic' or 'cubic')
    
    # Get the distances between the waypoints
    total_d = 0
    waypoints_d = np.zeros(waypoints.shape[0]-1)
    for i in range(waypoints.shape[0]-1):
        w0 = waypoints[i,:]
        w1 = waypoints[i+1,:]
        d = np.linalg.norm(w1-w0)
        waypoints_d[i] = d
        total_d += d

    # Set the time to be at each waypoint assuring constant speed
    waypoints_t = np.zeros(waypoints.shape[0])
    waypoints_t[0] = 0
    for i,d in enumerate(waypoints_d):
        waypoints_t[i+1] =  waypoints_t[i] + d/total_d

    # Get the data
    x = waypoints[:,0]
    y = waypoints[:,1]
    t = waypoints_t

    # Fit an interpolation function to the x and y coordinates in time
    fx = interp1d(t, x, kind='linear')
    fy = interp1d(t, y, kind='linear')

    tnew = np.linspace(t[0], t[-1], num=N, endpoint=True)
    return np.vstack((fx(tnew), fy(tnew))).T

