import numpy as np

joint_limits = [
    [-np.pi/2, np.pi/4],  # make joint 1 smaller from real
    [-np.pi, np.pi]]  

# length of two links and base link
a0 = 0.043
a1 = 0.093
a2 = 0.07

def in_joint_range(q):
    for i, qi in enumerate(q):
        if qi < joint_limits[i][0] or qi > joint_limits[i][1]:
            return False
    return True
    
def select_best_q(candidates, q0, weight = [1,1]):
    # we prefer minimum travel
    min_v = None
    best_q = None
    for q in candidates:
        v = np.sum(((np.array(q) - np.array(q0)) * np.array(weight)) ** 2)
        if (min_v == None or min_v > v) and in_joint_range(q):
            min_v = v
            best_q = q
    return best_q

def ik(target_TCP_xz, q0):
    x, z = target_TCP_xz[0], target_TCP_xz[1]-a0
    print("(x,z)=",x ," ",z+a0)
    ik_candidate = []
     
    # calculate q_1 and q_2 using trigonometry
    # known parameters a1: link 1's length, a2: link 2's length, (x,z): the coordinates for the target point


    #q_2 = 2*np.arctan(np.sqrt((np.power(a1+a2, 2)-np.power(x,2)-np.power(z,2))/(np.power(x,2)+np.power(z,2)-np.power(a1-a2, 2))))
    #q_1 = np.arctan2(z, x) - np.arctan2(a2*np.sin(q_2), a1+a2*np.cos(q_2)) - np.pi/2
    #print "q2: ", q_2, "q1: ", q_1
    # candidate 1
    # q_1=???
    # q_2=???

    #if not np.isnan([q_1, q_2]).any():
    #    ik_candidate.append([q_1, q_2])
    
    q_2 = -1*2*np.arctan(np.sqrt((np.power(a1+a2, 2)-np.power(x,2)-np.power(z,2))/(np.power(x,2)+np.power(z,2)-np.power(a1-a2, 2))))
    q_1 = np.arctan2(z, x) - np.arctan2(a2*np.sin(q_2), a1+a2*np.cos(q_2)) - np.pi/2
    print "q2: ", q_2, "q1: ", q_1
    # candidate 2
    # q_1=???
    # q_2=???
    if not np.isnan([q_1, q_2]).any():
        ik_candidate.append([q_1, q_2])
    print "ik_candidate: ", ik_candidate
    return select_best_q(ik_candidate, q0)

def ikv(target_TCP_vel, q0):
    J = Jacobian(q0)
    qdot = np.linalg.solve(J, target_TCP_vel).tolist()  # "J \ target_TCP_vel" in matlab
    return qdot

def Jacobian(q):
    q1 = q[0]
    q2 = q[1]
    s1 = np.sin(q1)
    s12 = np.sin(q1+q2)
    c1 = np.cos(q1)
    c12 = np.cos(q1+q2)
    return np.array([[-a1 * s1 - a2 * s12 ,  -a2 * s12  ], 
                     [a1 * c1 + a2 * c12  ,  a2 * c12 ]])

# return end point of the second link
def fk(q):
    th1 = q[0] + np.pi / 2
    th12 = th1 + q[1]
    return [a0+a1 * np.cos(th1) + a2 * np.cos(th12) ,
            a0+a1 * np.sin(th1) + a2 * np.sin(th12)]

# return end point of the first link
def fk1(q):
    th1 = q[0] + np.pi / 2
    return [a0+a1 * np.cos(th1),
            a0+a1 * np.sin(th1)]


