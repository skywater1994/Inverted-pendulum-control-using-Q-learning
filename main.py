from inverted_pendulum.qlearning.qlearning import QLearning
from inverted_pendulum.fuzzy.fuzzy_controller import *
from numpy import pi, cos, sin, exp
import random
import time

import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch

from inverted_pendulum.simulation.simulation import Simulator
import numpy as np



class InvertedPendulumBalancer:


    def __init__(self):
        self.dt = 0.01
        random.seed(200)
        self.max_force = 125
        self.step_n =  int( 10 * 10**6 )
        self.last_n = 500 * 10**3
        self.simulator = Simulator()

        self.controller = QLearning( max_theta=2*pi, max_dtheta=30, max_x=60, max_dx=40, n_x=10, n_dx=10, n_theta=6, n_dtheta=20, n_action=5, is_fuzzy=True )
    
    def pot_grp(self, onee, twoo):
        self.aa = onee
        self.bb = twoo

        plt.plot(aa, bb)
        plt.savefig("final.png")


    def plot_survival_times(self):
        survival_times = []
        for i in range(len(lines)):
            if i >= 1:
                survival_times.append(10*(lines[i][0] - lines[i - 1][0]))
        plt.plot(survival_times)
        plt.plot([last_n for s in survival_times])
        plt.show()

    def plot_states():
        plt.plot(thetas)
        for l in lines:
            plt.axvline(l[0], color=l[1], alpha=0.5)
        plt.plot(xs)
        #plt.plot(reward)
        plt.show()
    
    def run(self):
        state = [-0.2, 0, pi - 0.15, 0]
        states = []
        t = 0
        plot_n = 7 # number of iterations before plotting
        plot_resolution = 1 # number of iterations before adding state to list

        states = []
        last_stable = 0
        survival_times = []
        survival_time = 0
        lines = []
        rewardd = 0
        rewardd_a = np.empty(1)
        inttt = np.empty(1)


        plt.rc_context({'axes.edgecolor':'orange', 'xtick.color':'red', 'ytick.color':'green', 'figure.facecolor':'white', 'axes.linewidth': 2})

        plt.ion()
        mng = plt.get_current_fig_manager()
        #mng.resize(*mng.window.maxsize(10))

        theta_ax = plt.subplot2grid((5,3), (2,0), colspan=3)
        plt.title('Change in theta')

        x_ax = plt.subplot2grid((5,3), (3,0), colspan=3)
        plt.title('Change in position')
        
        cart_ax = plt.subplot2grid((5,3), (0,0), colspan=3, rowspan=2)
        plt.title('Pendulum in motion')

        reward_ax = plt.subplot2grid((5,3), (4,0), colspan=3)
        plt.title('Cumulative Reward')

        #reward_ax = plt.subplot2grid((4,3),(4,0), colspan=3)
        #plt.title('Reward vs iterations')

        for i in range(self.step_n):
            state[2] += (random.random() - 0.5) * 0.001
            survival_time += 1
                
            t = t + self.dt

            prev_state = state
            if i % plot_resolution == 0:
                survival_times.append(survival_time)
                states.append(state)
            
            if i % 1000 == 0:
                if len(states) > self.last_n / plot_resolution:
                    xs = [s[0] for s in states]
                    thetas = [s[2] for s in states]
                    last_thetas = thetas[-int(self.last_n/plot_resolution):]
                    last_xs = xs[-int(self.last_n/plot_resolution):]

                    theta_std = np.std(last_thetas)
                    x_std = np.std(last_xs)
                    if theta_std < 0.1 and xs[-1] < 20 and (last_stable == 0 or i - last_stable > self.last_n):
                        lines.append( (i / plot_resolution, 'y') )
                        last_stable = i
                        survival_time = 0
                        state = self.simulator.random_state(state)
            
            theta = state[2]
            if theta <= pi / 2 or theta >= 3 * pi / 2:
                lines.append( (i / plot_resolution, 'g') )
                state = self.simulator.random_state(state)
                survival_time = 0
        
            q_state = [state[0], state[1], state[2] + pi, state[3]]

            action = self.controller.action(q_state)
            force = self.max_force * action[1]
            state = self.simulator.simulate_step(state, force, self.dt)

            next_q_state = [state[0], state[1], state[2] + pi, state[3]]

            reward = 5
            if abs(pi - state[2]) >= 0.1:
                reward = -30 * ( abs(pi - state[2]) ** 2 )
            
            if abs(state[0]) >= 15:
                reward -= abs(state[0]) ** 1.5

            self.controller.update(q_state, action[0], next_q_state, reward)




            if i > 0 and i % (plot_n - 1) == 0:

                rewardd = reward + rewardd
                rewardd_a = np.append(rewardd_a,rewardd)
                inttt = np.append(inttt, len(survival_times))
                print (len(survival_times))

                
                #pot_grp(inttt,rewardd_a)
                
                



                theta_ax.plot([s[2] for s in states], color='y')
                x_ax.plot([s[0] for s in states], color='c')
                reward_ax.plot(inttt, rewardd_a, color='r')
                #print (rewardd,len(survival_times))

                
                cart_ax.patches = []
                cart_ax.artists = []
                cart_ax.lines = []
                cart_width = 20
                cart_height = 2.5
                factor = 2
                r = 1
                cart_ax.axis([-factor * cart_width, factor * cart_width, 0, factor * cart_width])
                p_fancy = FancyBboxPatch((-cart_width / 2 + state[0] , 2.5 * r),
                             abs(cart_width), abs(cart_height),
                             boxstyle="round,pad=0.5",
                             fc=(0, 1.0, 0),
                             ec=(1., 0.5, 1.))
                cart_ax.add_patch(p_fancy)
                circle1 = plt.Circle((-cart_width / 4 + state[0] , 2 * r), r, color='k')
                circle2 = plt.Circle((cart_width / 4 + state[0] , 2 * r), r, color='k')
                cart_ax.add_artist(circle1)
                cart_ax.add_artist(circle2)

                L = 6 * cart_height
                L_discount = (L + L * sin(pi/2 - state[2]) ** 2)
                cart_ax.plot([state[0], state[0] - L_discount * cos(pi/2 - state[2])], 
                    [1.5 * cart_height, 1.5 * cart_height -  L_discount * sin(pi/2 - state[2])], 
                    color='r', 
                    solid_capstyle="round",
                    linewidth=2)
                
                
                #print (s[0] for s in states)
                plt.pause(0.000001)
                
                plt.tight_layout(pad=0.4, w_pad=0.5, h_pad=1.0)

            if i % 1000== 0:
                print(i)


balancer = InvertedPendulumBalancer()
balancer.run()




