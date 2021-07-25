#3D rocket simulator
#update to use getter and setter methods to update class variables

import numpy as np
import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d.axes3d as p3
import matplotlib.animation as animation
import widgets
import PID

#Rocket Properties
d = 0.081 #distance between thrust and centre of mass
g = 9.81
m = 276.72e-3
'''Inertia = [[1660953.81e-9, -1927.75e-9, -33533.31e-9],
            [-1927.75e-9, 1644412.86e-9,	48417.23e-9],
            [33533.31e-9, 48417.23e-9, -153203.27e-9]]'''

Inertia=    [[2155504.71e-9, 1137.42e-9, 27318.12e-9],
            [1137.42e-9, 2130300.4e-9, 55930.98e-9],
            [27318.12e-9, 55930.98e-9, 157116.59e-9]]



#PID variables
SETPOINT = np.radians(0.0)
kp = 0.1425#0.16
ki = 0.0729#0.0
kd = 0.0532#0.001
N = 36.47 
tau =1/N
PID_period = 0.02
TVC_MAX = np.radians(5) #max actuation of TVC

#sim properties
ANIMATION_SPEED = 1 
MAX_SIM_DURATION = 5 #seconds
dt = PID_period
offset_angle_X = 5 #degrees
offset_angle_Y = 5 #degrees

#offset matricies
angx = np.radians(offset_angle_X)
angy = np.radians(offset_angle_Y)
Rx = np.array([[1.0,0.0,0.0], [0.0,np.cos(angx), -np.sin(angx)], [0, np.sin(angx), np.cos(angx)]])
Ry = np.array([[np.cos(angy), 0.0, np.sin(angy)], [0.0,1.0,0.0], [-np.sin(angy), 0.0, np.cos(angy)]])

#Thrust profile for rocket motor
thrustprofileC6 = [
    0.031, 0.946,
    0.092, 4.826,
    0.139, 9.936,
    0.192, 14.090,
    0.209, 11.446,
    0.231, 7.381,
    0.248, 6.151,
    0.292, 5.489,
    0.370, 4.921,
    0.475, 4.448,
    0.671, 4.258,
    0.702, 4.542,
    0.723, 4.164,
    0.850, 4.448,
    1.063, 4.353,
    1.211, 4.353,
    1.242, 4.069,
    1.303, 4.258,
    1.468, 4.353,
    1.656, 4.448,
    1.821, 4.448,
    1.834, 2.933,
    1.847, 1.325,
    1.860, 0.000]

thrust_t  = [thrustprofileC6[i] for i in range(len(thrustprofileC6)) if i%2 == 0]
thrust_f = [thrustprofileC6[i] for i in range(len(thrustprofileC6)) if i%2 == 1]

def thrustvecloc(phix, phiy):
    return np.array([np.sin(phiy) * np.cos(phix), -np.sin(phix), np.cos(phiy)*np.cos(phix)])
                
class rocket_object:
    def __init__(self, init_mass, Inertia_matrix, phix=0,phiy=0, x_init= [0.0,0.0,0.0],init_R = [[1.0,0.0,0.0],
                                                                                                 [0.0,1.0,0.0],
                                                                                                 [0.0,0.0,1.0]] ):
        self.x = np.array(x_init)
        self.v = np.array([0.0,0.0,0.0])
        self.a = np.array([0.0,0.0,0.0])
        self.mass = init_mass
        self.I = np.array(Inertia_matrix)
        self.R = np.array(init_R) # rotation matrix for vector conversion from local to global frame. 
        self.phix = phix #tvc angle about local x
        self.phiy = phiy #tvc angle about local y
        self.T = thrustvecloc(self.phix, self.phiy) #direction of TVC thrust in the local frame
        self.w = np.array([0.0,0.0,0.0])
        Hr = np.array([0.0,0.0,1.0])
        Hg = np.matmul(self.R, Hr)
        self.Hg = Hg
        self.thetax = np.arctan2(Hg[0],Hg[2])
        self.thetay = np.arctan2(-Hg[1], Hg[2])
        self.alpha = None

    def step(self, dt, thrust, phix, phiy):
        self.phix = phix
        self.phiy = phiy
        self.T = thrustvecloc(self.phix, self.phiy)
        W = np.array([0.0,0.0, -self.mass*g])
        Tr = np.array(thrust*self.T)
        Tg = np.matmul(self.R, Tr)
        Hr = np.array([0.0,0.0,1.0])
        # translation
        self.a = (W+Tg)/self.mass
        self.v += dt*self.a
        self.x +=dt*self.v

        #rotation
        T_torque = np.cross(-d*Hr,Tr)
        self.alpha = np.matmul(np.linalg.inv(self.I), T_torque)
        self.w += dt*self.alpha
        angle = np.linalg.norm(self.w)*dt
        axis = self.w if np.linalg.norm(self.w) == 0.0 else self.w/np.linalg.norm(self.w)
        delta = np.identity(3) * np.cos(angle) + np.sin(angle) * (np.array([[0, -axis[2], axis[1]], [axis[2], 0, -axis[0]],[axis[1],axis[0],0]]))+(1-np.cos(angle))*(np.matmul(np.array(axis).T,axis))
        self.R = np.matmul(self.R, delta)
        Hg = np.matmul(self.R, Hr)
        self.Hg = Hg
        self.thetax = np.arctan2(-Hg[1],Hg[2])
        self.thetay = np.arctan2(Hg[0], Hg[2])

def main():
    myrocket = rocket_object(m, Inertia, init_R = np.matmul(Rx, Ry))
    controller = PID.PID(kp, ki, kd, tau, PID_period, TVC_MAX, SETPOINT)

    t = []
    posi = []
    x = []
    y = []
    z = []
    h = []
    thetax = []
    thetay = []
    i = 0

    while myrocket.x[2] >=-1 and dt*i < MAX_SIM_DURATION:
        t.append(dt*i)
        thetax.append(myrocket.thetax)
        thetay.append(myrocket.thetay)
        posi.append(myrocket.x)
        x.append(myrocket.x[0])
        y.append(myrocket.x[1])
        z.append(myrocket.x[2])
        h.append(myrocket.Hg/np.linalg.norm(myrocket.Hg))
        #print(f"time{t[-1]}, heading : {myrocket.Hg}, headingmag: {np.linalg.norm(myrocket.Hg)}")
        thrustmag = np.interp(t[-1],thrust_t, thrust_f)
        phix, controller.prev_error_x, controller.I_x, controller.D_x = controller.PID_compute(myrocket.thetax,controller.prev_error_x,controller.I_x, controller.D_x )
        phiy, controller.prev_error_y, controller.I_y, controller.D_y = controller.PID_compute(myrocket.thetay,controller.prev_error_y,controller.I_y, controller.D_y )
        #phix, le_x, cume_x = PID(myrocket.thetax, le_x, cume_x)
        #phiy, leyx, cume_y = PID(myrocket.thetay, le_y, cume_y)
        myrocket.step(dt, thrustmag, phix, phiy)
        #print(f"t: {t[-1]}, pos : {list(myrocket.x)}, phix: {myrocket.phix}, thetax: {myrocket.thetax}, phiy {myrocket.phiy}, thetay: {myrocket.thetay}, Heading: {myrocket.Hg} ")
        i +=1

    


    plt.figure(1)
    plt.clf()
    plt.xlabel('t')
    plt.grid()
    plt.plot(t, thetax, label='thetax (m)')
    plt.legend()
    plt.show()

    plt.figure(2)
    plt.clf()
    plt.xlabel('y')
    plt.grid()
    plt.plot(y, z, label='z (m)')
    plt.legend()
    plt.show()

    fig = plt.figure()
    ax = plt.axes(projection='3d')
    ax.plot3D(x, y, z)
    ax.set_xlim3d([-15, 15])
    ax.set_ylim3d([-15, 15])
    plt.show()
    
    # 3D animation
    fig = plt.figure()
    ax = p3.Axes3D(fig)

    lines = ax.plot3D([0,0],[0,0],[0,1])
    lines2 = ax.plot3D([0],[0],[1],"ro", markersize=4)
    line2 = lines2[0]
    line = lines[0]


    ax.set_xlim3d([-10, 10])
    ax.set_xlabel('X')

    ax.set_ylim3d([-10, 10])
    ax.set_ylabel('Y')

    ax.set_zlim3d([-1, 30])
    ax.set_zlabel('Z')

    ax.set_title('Launch')


    def animate(frame):
        linexy = [[x[frame]+h[frame][0], x[frame]-2*h[frame][0]],[y[frame]+h[frame][1], y[frame]-2*h[frame][1]]]
        linez = [z[frame] + h[frame][2], z[frame]-2*h[frame][2]]
        line.set_data(np.array(linexy))
        line.set_3d_properties(np.array(linez))
        line2.set_data(np.array([x[frame],y[frame]]))
        line2.set_3d_properties(np.array([z[frame]]))
        return lines

    #line_ani = animation.FuncAnimation(fig, animate, 250 , interval = 20/ANIMATION_SPEED, blit=False)

    ani = widgets.Player(fig, animate, interval = 20/ANIMATION_SPEED, maxi=len(x))
    plt.show()

if __name__ == "__main__":
    main()
