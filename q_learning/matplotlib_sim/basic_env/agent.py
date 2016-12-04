# -----------------------------------
# car parking agent using Q-learning
# Author: Tao Chen
# Date: 2016.10.28
# -----------------------------------
import random
from car_parking_env import car_sim_env
from car_parking_env import Agent
import os
import re
import cPickle
import threading
from collections import namedtuple
states = namedtuple('states','x y theta')


class LearningAgent(Agent):
    """An agent that learns to automatic parking"""

    def __init__(self, env, test = False):
        super(LearningAgent, self).__init__(env)  # sets self.env = env, state = None
        self.test = test
        if self.test:
            self.epsilon = 0.0
        else:
            self.epsilon = 0.4

        self.learning_rate = 0.90

        self.default_q = 0.0
        self.gamma = 0.8

        self.Q_values = {}
        self.state = None
        self.prev_state = None
        self.prev_action = None
        self.prev_reward = None


    def reset(self):
        self.state = None
        self.prev_state = None
        self.prev_action = None
        self.prev_reward = None

    def update(self):
        agent_pose = self.env.sense()
        self.state = states(x = agent_pose[0], y = agent_pose[1], theta = agent_pose[2])

        step = self.env.get_steps()
        if self.env.enforce_deadline:
            deadline = self.env.get_deadline()

        action, max_q_value = self.get_action(self.state)

        next_agent_pose,reward = self.env.act(self, action)

        if not self.test:
            if self.prev_action != None:
                    self.update_q_values(self.prev_state, self.prev_action, self.prev_reward, max_q_value)

            if self.env.enforce_deadline:
                print "LearningAgent.update(): step = {}, deadline = {}, state = {}, action = {}, reward = {}".format(step, deadline,
                                                                                                                      self.state, action, reward)
            else:
                print "LearningAgent.update(): step = {}, state = {}, action = {}, reward = {}".format(step, self.state,
                                                                                                        action, reward)

        self.save_state(self.state, action, reward)

    def set_q_tables(self, path):
        with open(path, 'rb') as f:
            self.Q_values = cPickle.load(f)


    def save_state(self, state, action, reward):
        self.prev_state = self.state
        self.prev_action = action
        self.prev_reward = reward


    def update_q_values(self, prev_state, prev_action, prev_reward, max_q_value):
        old_q_value = self.Q_values.get((prev_state, prev_action), self.default_q)
        new_q_value = old_q_value + self.learning_rate * (prev_reward + self.gamma * max_q_value - old_q_value)
        self.Q_values[(prev_state, prev_action)] = new_q_value


    def get_maximum_q_value(self, state):
        q_value_selected = -10000000
        for action in car_sim_env.valid_actions:
            q_value = self.get_q_value(state, action)
            if q_value > q_value_selected:
                q_value_selected = q_value
                action_selected = action
            elif q_value == q_value_selected:  # if there are two actions that lead to same q value, we need to randomly choose one between them
                action_selected = random.choice([action_selected, action])
        return action_selected, q_value_selected


    def get_action(self, state):
    	if random.random() < self.epsilon:
    		action_selected = random.choice(car_sim_env.valid_actions)
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



def run(restore, LfD = False):
    env = car_sim_env()
    agt = env.create_agent(LearningAgent, test=True)
    env.set_agent(agt, enforce_deadline=True)

    train_thread = threading.Thread(name="train", target=train, args=(env, agt, restore, LfD))
    train_thread.daemon = True
    train_thread.start()
    env.plt_show()
    while True:
        continue



def train(env, agt, restore, LfD = False):
    n_trials = 9999999999
    quit = False
    parent_path = os.path.dirname(os.path.realpath(__file__))
    data_path = os.path.join(parent_path, 'q_table')
    lfd_path = os.path.join(parent_path, 'LfD')

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
        agt.q_table_LfD(lfd_path)



    for trial in xrange(max_index + 1, n_trials):
        print "Simulator.run(): Trial {}".format(trial)
        if not agt.test:
            if trial > 10000 and trial < 20000:
                agt.epsilon = 0.3
            elif trial > 20000 and trial < 40000:
                agt.epsilon = 0.2
            elif trial > 40000 and trial < 50000:
                agt.epsilon = 0.1
            elif trial > 50000 and trial < 60000:
                agt.epsilon = 0.05
            elif trial > 60000 and trial < 75000:
                agt.epsilon = 0.01
            elif trial > 75000:
                agt.epsilon = 0

        env.reset()
        while True:
            try:
                env.step()
            except KeyboardInterrupt:
                quit = True
            finally:
                if quit or env.done:
                    break


        test_interval = 200
        if trial % test_interval == 0:
            total_runs = env.succ_times + env.hit_wall_times + env.hit_car_times + env.num_hit_time_limit \
                         + env.num_out_of_time
            succ_rate = env.succ_times / float(total_runs)
            print 'successful trials: ', env.succ_times
            print 'number of trials that hit the hard time limit: ', env.num_hit_time_limit
            print 'number of trials that ran out of time: ', env.num_out_of_time
            print 'number of trials that hit cars', env.hit_car_times
            print 'number of trials that hit walls', env.hit_wall_times
            print 'successful rate', succ_rate
            print 'total runs:', total_runs
            if not agt.test:
                succ_rate_file = os.path.join(data_path, 'succ_rate' + '.cpickle')
                rates={}
                if os.path.exists(succ_rate_file):
                    with open(succ_rate_file) as f:
                        rates = cPickle.load(f)
                        os.remove(succ_rate_file)
                rates[trial] = succ_rate

                with open(succ_rate_file, 'wb') as f:
                    cPickle.dump(rates, f, protocol=cPickle.HIGHEST_PROTOCOL)
            env.clear_count()


        if not agt.test:
            if trial % 2000 == 0:
                print "Trial {} done, saving Q table...".format(trial)
                q_table_file = os.path.join(data_path, 'trial' + str('{:010d}'.format(trial)) + '.cpickle')
                with open(q_table_file, 'wb') as f:
                    cPickle.dump(agt.Q_values, f, protocol=cPickle.HIGHEST_PROTOCOL)

            if quit:
                break


if __name__ == '__main__':
    run(restore = True, LfD=True)



