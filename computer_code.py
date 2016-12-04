import random
import math
import numpy as np
import serial

bound = 2**16
basis = 3
alpha = 0.001  # learning rate
lambda_ = 0.9  # trace decay rate
gamma = 1.0  # discount rate

M = 2
N = (basis + 1)**3


def get_c(goal, choices, storage, current=0):
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
    return get_c(goal, choices, new_storage, current+1)


def get_reward(s):
    return - (s[0]**2)


def q_value(s, a, c, theta):
    """
    :param s: list of state variables
    :param a: variable defining action (0 or 1 for L or R)
    :param c: list of N/2 vectors of dimension 5
    :param theta: vector of dimension N
    :return: q value of action
    """
    x = np.append(s, [a])
    val = 0
    for i in xrange(N):
        val += theta[i] * math.cos(math.pi * np.dot(c[i], x))
    return val


def normalize(s):
    s[0] /= 0.21
    s[1] /= 3.
    return s


def best_action(s, c, theta):
    """
    :param s: list of state variables
    :param c: list of N/2 vectors of dimension 5
    :param theta: vector of dimension N
    :return: the action (either 0 or 1) with the highest q score.
    """
    left_val = q_value(s, 0, c, theta)
    right_val = q_value(s, 1, c, theta)
    if left_val > right_val:
        return 0
    elif left_val == right_val and random.random() < 0.5:
        return 0
    else:
        return 1


def best_q(s, c, theta):
    """
    :param s: list of state variables
    :param c: list of N/2 vectors of dimension 5
    :param theta: vector of dimension N
    :return: the highest q score possible given the possible actions.
    """
    return max(q_value(s, 0, c, theta), q_value(s, 1, c, theta))


def update_coef(new_s, new_a, s, a, c, theta, r):
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
    x = np.append(s, [a])
    theta_new = np.zeros(N)

    temp = alpha * (r + gamma * q_value(new_s, new_a, c, theta) - q_value(s, a, c, theta))
    for i in xrange(N):
        derivative = math.cos(math.pi * np.dot(c[i], x))
        theta_new[i] = min(max(0, theta[i] + temp * derivative), 100)
    return theta_new


def get_action(state, theta, c, last_state, last_action, total_reward):
    if not last_state:
        last_state = normalize(state)  # TODO check normalize
        last_action = 0
        total_reward = 0
        return 0, last_state, last_action, total_reward, theta
    if abs(state[0]) > 45:
        total_reward = 0
        last_state = None
        last_action = None
        return 0, last_state, last_action, total_reward, theta
    reward = get_reward(state)
    # TODO make sure normalization works
    state = normalize(state)
    action = best_action(state, c, theta)
    theta = update_coef(state, action, last_state, last_action, c, theta, reward)
    total_reward += reward
    last_state = state
    last_action = action
    return action, last_state, last_action, total_reward, theta


def main():
    theta = [random.random() for _ in xrange(N)]
    c = get_c(3, basis+1, [])
    last_state = None
    last_action = None
    total_reward = None
    # ".usbserial" if it doesn't work
    ser = serial.Serial('/dev/ttyACM0', 115200)  # opens line of communication with Serial on highest freq...
    print "setup done"

    while True:
        if ser.in_waiting():
            angle_filtered = float(ser.readline())
            omega = float(ser.readline())
            state = [angle_filtered, omega]
            action, last_state, last_action, total_reward, theta = get_action(state, theta, c, last_state, last_action, total_reward)
            ser.write(action)

main()
