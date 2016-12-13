import random
import math
import numpy as np
import sys
import serial
from datetime import datetime

bound = 2**16
basis = 3
alpha = 0.00001  # learning rate
gamma = 1  # discount 
epsilon = 0

N = 2*(basis + 1)**4

a = 15
norm = 0.8

class Vars:
    def __init__(self, state, action, last_state, last_action, total_reward, theta, c):
        self.state = state
        self.action = action
        self.last_state = last_state
        self.last_action = last_action
        self.total_reward = total_reward
        self.theta = theta
        self.c = c


def get_c(goal, choices, current=0, storage=[]):
    """
    :param goal: the desired length of vectors
    :param choices: the number possible choices for each entry in the vector
    :param current: the current length of vectors
    :param storage: the list of currently developed vectors
    :return: a list of distinct vectors of size "goal" with values chosen from the
    range [0, choices).
    """
    if current == goal:
        return storage
    elif current == 0:
        new_storage = [[x] for x in xrange(choices)]
    else:
        new_storage = [x+[y] for y in xrange(choices) for x in storage]
    return get_c(goal, choices, current+1, new_storage)


def getReward(s):
    return -(s[0]**2)


def q_value(x, c, theta):
    """
    :param s: list of state variables
    :param a: variable defining action (0 or 1 for L or R)
    :param c: list of N/2 vectors of dimension 5
    :param theta: vector of dimension N
    :return: q value of action
    """
    val = 0
    for i in xrange(N/2):
        val += theta[2*i] * math.cos(math.pi * np.dot(c[i], x)) + theta[2*i+1] * math.sin(math.pi * np.dot(c[i], x))
    return val


def normalize(s):
    s[0] /= 500.
    s[1] /= 10.
    s[2] /= norm
    s[3] /= 2.
    for i in xrange(4):
        if abs(s[i]) > 1:
            s[i] = s[i]/abs(float(s[i]))
    return s


def best_action(s, c, theta):
    """
    :param s: list of state variables
    :param c: list of N/2 vectors of dimension 5
    :param theta: vector of dimension N
    :return: the action (either 0 or 1) with the highest q score.
    """
    actions = range(a)
    
    val = []
    maxval = -1000
    best_action = 0
    for i in xrange(a):
        val = q_value(s, c, theta[i])
        if ( val > maxval):
            best_action = i
            maxval = val

    
    if (random.random() < epsilon):
        return random.choice(actions)
    return best_action





def update_coef(new_s, s, c, theta, theta_next, r):
    """
    :param new_s: list of new state variables
    :param new_a: best action available from new_s
    :param s: list of old state variables
    :param a: variable defining action taken to get from s to new_s
    :param c: list of N/2 vectors of dimension 5
    :param theta: vector of dimension N
    :param r: reward given
    :return: the updated theta vector
    """

    theta_new = np.zeros(N)

    temp = alpha * (r + gamma * q_value(new_s, c, theta_next) - q_value(s, c, theta))
    for i in xrange(N):
        if i % 2 == 0:
            deriv = math.cos(math.pi * np.dot(c[i/2], s))
        else:
            deriv = math.sin(math.pi * np.dot(c[i/2], s))
        theta_new[i] = min(max(0, theta[i] + temp * deriv), 100)


    return theta_new


def get_action(vars):

    if not vars.last_state and not vars.last_action:
        print "no prev"
        vars.last_state = vars.state
        vars.last_action = 7
        vars.action = 7
        return vars
    if abs(vars.state[2]) * norm > math.pi / 4:
        print 'angle exceeded'
        vars.total_reward = 0
        vars.last_state = None
        vars.last_action = None
        vars.action = 7
        return vars

    reward = getReward(vars.state)

    vars.action = best_action(vars.state, vars.c, vars.theta)

    
    theta_next = vars.theta[vars.action]
    theta_prev = vars.theta[vars.last_action]
    vars.theta[vars.last_action] = update_coef(vars.state, vars.last_state, vars.c, theta_prev, theta_next, reward)    



    vars.total_reward += reward
    vars.last_state = vars.state
    vars.last_action = vars.action
    return vars


def read_word(ser):
    past = ''
    word = ''
    while past != '\n':
        word += past
        seen = False
        while not seen:
            if ser.in_waiting:
                past = ser.read()
                seen = True
    return word


def main():
    # Initialization of the sets of coefficients, one for every action

    f = open('output.txt', 'r')
    theta = []
    for i in xrange(a):
        mystr = f.readline()
        t = [float(e) for e in mystr.split()]
        theta.append(t)


    c = get_c(4, basis+1)

    vars = Vars(None, 0, None, None, 0, theta, c)

    # opens line of communication with Serial on highest freq
    ser = serial.Serial('/dev/cu.usbmodem1421', 250000)
    print "setup done"


    while True:
        distance = float(read_word(ser))
        speed = float(read_word(ser))
        angle_filtered = float(read_word(ser))
        omega = float(read_word(ser))
        vars.state = normalize([distance, speed, angle_filtered * math.pi / 180., omega * math.pi / 180.])
        assert(vars.state is not None)
        vars = get_action(vars)
        output = str(vars.action) + " "
        ser.write(output)


main()
