# -----------------------------------
# car parking agent using Q-learning
# Author: Tao Chen
# Date: 2016.11.08
# -----------------------------------
import rospy
import numpy as np
import random
from environment import Environment
from environment import Agent
import os
import re
from collections import namedtuple
states = namedtuple('states','x y theta')
import cPickle

class LearningAgent(Agent):
    """An agent that learns to automatic parking"""

    def __init__(self, env, test = False):
        super(LearningAgent, self).__init__(env)  # sets self.env = env, state = None
        self.test = test
        if not self.test:
            self.epsilon = 0.4
        else:
            self.epsilon = 0
        print 'epsilon:',self.epsilon

        self.learning_rate = 0.90

        self.default_q = 0.0
        self.gamma = 0.8

        self.Q_values = {}
        self.state = None


    def reset(self):
        self.state = None
        self.action = None
        self.reward = None


    def update(self):
        if self.state == None:
            agent_pose = self.env.sense()
            self.state = states(x = agent_pose[0], y = agent_pose[1], theta = agent_pose[2])

        step = self.env.get_steps()
        if self.env.enforce_deadline:
            deadline = self.env.get_deadline()

        # Select action according to your policy
        action, max_q_value = self.get_action(self.state)
        # print 'max_q_value:',max_q_value

        # Execute action and get reward
        next_agent_pose,reward = self.env.act(self, action)
        self.next_state = states(x=next_agent_pose[0], y=next_agent_pose[1], theta=next_agent_pose[2])

        # Learn policy based on state, action, reward
        if not self.test:
            self.update_q_values(self.state, action, self.next_state, reward)

        if self.env.enforce_deadline:
            print "LearningAgent.update(): step = {}, deadline = {}, state = {}, action = {}, reward = {}".format(step, deadline,
                                                                                                                  self.next_state, action, reward)
        else:
            print "LearningAgent.update(): step = {}, state = {}, action = {}, reward = {}".format(step, self.next_state,
                                                                                                    action, reward)
        self.state = self.next_state

    def set_q_tables(self, path):
        with open(path, 'rb') as f:
            self.Q_values = cPickle.load(f)


    def save_state(self, state, action):
    	self.prev_state = self.state
        self.prev_action = action


    def update_q_values(self, state, action, next_state, reward):
    	old_q_value = self.Q_values.get((state,action), self.default_q)
        action, max_q_value = self.get_maximum_q_value(next_state)
    	new_q_value = old_q_value + self.learning_rate * (reward + self.gamma * max_q_value - old_q_value)
    	self.Q_values[(state,action)] = new_q_value
        # print 'Q_values.shape',len(self.Q_values)

    def get_maximum_q_value(self, state):
        q_value_selected = -10000000
        for action in Environment.valid_actions:
            q_value = self.get_q_value(state, action)
            if q_value > q_value_selected:
                q_value_selected = q_value
                action_selected = action
            elif q_value == q_value_selected:  # if there are two actions that lead to same q value, we need to randomly choose one between them
                action_selected = random.choice([action_selected, action])
        return action_selected, q_value_selected

    def get_action(self, state):
    	if random.random() < self.epsilon:
    		action_selected = random.choice(Environment.valid_actions)
    		q_value_selected = self.get_q_value(state, action_selected)
    	else:
            action_selected, q_value_selected = self.get_maximum_q_value(state)

    	return action_selected, q_value_selected

    def get_q_value(self, state, action):
    	return self.Q_values.get((state,action), self.default_q)


    def q_table_LfD(self, data_path):
        file_lists = os.listdir(data_path)
        for file in file_lists:
            file_path = os.path.join(data_path, file)
            with open(file_path, 'rb') as f:
                state_action_pairs = cPickle.load(f)
                for state_action in state_action_pairs:
                    if state_action in self.Q_values:
                        self.Q_values[state_action] += 5.0
                    else:
                        self.Q_values[state_action] = 5.0
                self.env.reward_db += state_action_pairs


def run(restore, q_manual_init = False, LfD = False):
    env = Environment()
    agt = env.create_agent(LearningAgent, test=True)
    env.set_agent(agt, enforce_deadline=False)

    n_trials = 10000000
    quit = False
    parent_path = os.path.dirname(os.path.realpath(__file__))
    data_path = os.path.join(parent_path, 'q_table')

    if not os.path.exists(data_path):
        os.makedirs(data_path)
    files_lst = os.listdir(data_path)
    max_index = 0
    filepath = ''
    for filename in files_lst:
        fileindex_list = re.findall(r'\d+', filename)
        if not fileindex_list:
            continue
        fileindex = int(fileindex_list[0])
        if fileindex >= max_index:
            max_index = fileindex
            filepath = os.path.join(data_path, filename)

    if restore:
        if os.path.exists(filepath):
            print 'restoring Q_values from {} ...'.format(filepath)
            agt.set_q_tables(filepath)
            print 'restoring done...'


    if LfD:
        print 'initializing Q_values from LfD(Learning from Demonstration)...'
        agt.q_table_LfD(os.path.join(parent_path, 'LfD', 'lfd0000003.cpickle'))



    for trial in xrange(max_index + 1, n_trials):
        print "Simulator.run(): Trial {}".format(trial)  # [debug]
        if not agt.test:
            if trial > 10000 and trial < 30000:
                agt.epsilon = 0.3
            elif trial > 30000 and trial < 50000:
                agt.epsilon = 0.2
            elif trial > 50000 and trial < 70000:
                agt.epsilon = 0.1
            elif trial > 70000:
                agt.epsilon = 0.05
        env.reset()
        print 'epsilon:', agt.epsilon
        while True:
            try:
                env.step()
            except KeyboardInterrupt:
                quit = True
            finally:
                if quit or env.done:
                    break

        env.set_agent_velocity(np.zeros(2))


        if not agt.test:
            if trial % 50 == 0:
                print "Trial {} done, saving Q table...".format(trial)
                q_table_file = os.path.join(data_path, 'trial' + str('{:07d}'.format(trial)) + '.cpickle')
                with open(q_table_file, 'wb') as f:
                    cPickle.dump(agt.Q_values, f, protocol=cPickle.HIGHEST_PROTOCOL)

        if quit:
            break

    print 'successful trials: ', env.succ_times
    print 'number of trials that hit the hard time limit: ', env.num_hit_time_limit
    print 'number of trials that ran out of time: ', env.num_out_of_time
    print 'number of trials that hit cars', env.hit_car_times
    print 'number of trials that hit walls', env.hit_wall_times


if __name__ == '__main__':
    rospy.init_node('automatic_parking')
    rate = rospy.Rate(1000)
    run(restore = True, LfD=False)



