import numpy as numpy
import matplotlib.pyplot as pyplot
from mpl_toolkits.mplot3d import Axes3D
import pickle
import sys
import os
from functools import partial

from policy import *
from utils import *
from generate_data import generate_data
from rrt_connect_env import RRTConnectEnv



def plot_policy_comparisons():
    policy = Policy(1)
    policy.load_model('good_models/model_envA1.ckpt')


    # dynamic domain
    plt.subplot(1, 2, 1)
    obs = np.array([np.linspace(-10, 10, 100)]).T
    fd = {policy.state_input: obs, policy.is_train: np.array([False])}
    probs = policy.sess.run(policy.prob, feed_dict=fd)

    plt.title("Flytrap A")

    plt.plot(obs, probs[:, 0], c='r', label='Learned Policy')
    plt.xlabel("feature")
    plt.ylabel("p(accept)")

    dd_rrt = np.heaviside(obs, 1)
    dd_rrt = dd_rrt[::-1]
    plt.plot(obs, dd_rrt, c='b', linestyle='-', label='Dynamic Domain')
    plt.legend()


    policy.load_model('data/model_envB3.ckpt')
    plt.subplot(1, 2, 2)
    obs = np.array([np.linspace(-2, 10, 100)]).T
    fd = {policy.state_input: obs, policy.is_train: np.array([False])}
    probs = policy.sess.run(policy.prob, feed_dict=fd)

    plt.title("Flytrap B")

    plt.plot(obs, probs[:, 0], c='r', label='Learned Policy')
    plt.xlabel("feature")
    plt.ylabel("p(accept)")

    dd_rrt = np.heaviside(obs, 1)
    plt.plot(obs, dd_rrt, c='b', linestyle='-', label='BallTree')
    plt.legend()


    plt.show()

def plot_feat(policy, **kwargs):
    obs = np.array([np.linspace(-10, 10, 100)]).T
    fd = {policy.state_input: obs, policy.is_train: np.array([False])}
    probs = policy.sess.run(policy.prob, feed_dict=fd)

    plt.plot(obs, probs[:, 0], **kwargs)
    plt.xlabel("feature")
    plt.ylabel("p(accept)")


def plot_model_a1():
    policy = Policy(1)
    # policy.load_model('good_models/model_env_a1_best/model_envA1.ckpt')
    model_dir = 'good_models/model_envA1_best/'
    files = [f for f in os.listdir(model_dir) if os.path.isfile(os.path.join(model_dir, f))]
    numbers = []
    for file in files:
        if '.index' in file:
            tokens = file.split('.')
            num = tokens[2][4:]
            numbers.append(int(num))

    numbers.sort()
    numbers = numbers[::3]

    alphas = np.linspace(0.5, 0.9, len(numbers))
    for i, num in enumerate(numbers):
        file = model_dir + 'model_envA1.ckpt.best' + str(num) + '.ckpt'
        policy.load_model(file)
        color = [float(len(numbers)+1-i)/float(len(numbers)+1), 0., float(i)/float(len(numbers)+1)]
        plot_feat(policy, linestyle='--', c=color)

    policy.load_model('good_models/model_envA1.ckpt')
    plot_feat(policy, linestyle='-', c='b', markersize=2)

    plt.title('Evolution of Policy over Iterations')

    plt.show()


def dist_model_a1():
    fig1 = plt.figure(1)
    ax1 = fig1.gca(projection='3d')

    fig2 = plt.figure(2)
    ax2 = fig2.gca()

    np.random.seed(0)
    l2_data_dict = generate_data('fly_trap_fixed_a', dubins=False)

    data = np.array(l2_data_dict['map'], np.float)
    l2_random_sampler = partial(map_sampler_goal_bias, eps=0.1)
    l2_goal = l2_goal_region
    l2_config = {'collision_check': map_collision_check,
              'random_sample': l2_random_sampler,
              'steer': holonomic_steer,
              'dist': l2_dist,
              'goal_region': l2_goal,
              'feat': get_feat_flytrap,
              'num_feat': 1}


    def plot_tree(rrt, ax):
        def plot_line(ax, node1, node2, **kwargs):
            ax.plot([node1[1], node2[1]], [node1[0], node2[0]], **kwargs)

        states = rrt.tree.node_states
        for node1_idx, node2_idx in rrt.tree.c_p_edges.iteritems():
            node1 = states[node1_idx]
            node2 = states[node2_idx]
            plot_line(ax, node1, node2, color='green')


    def plot_dist(policy, rrt, ax, data):
        x=np.linspace(0, data.shape[0]-1, 40)
        y=np.linspace(0, data.shape[1]-1, 40)

        xv, yv = np.meshgrid(x, y)
        xv = xv.flatten()
        yv = yv.flatten()

        obs = np.vstack((xv, yv)).T
        feats = np.zeros((len(obs), 1))
        for i, ob in enumerate(obs):
            feat = rrt.config['feat'](ob, rrt.tree, rrt.map_info)
            feats[i] = feat

        fd = {policy.state_input: feats, policy.is_train: np.array([False])}
        probs = policy.sess.run(policy.prob, feed_dict=fd)

        probs = probs[:, 0]
        prob_sum = np.sum(probs)
        probs /= prob_sum
        ax.scatter(yv, xv, probs, alpha=0.5)

    def plot_all(policy, rrt, ax, data):
        ax.clear()
        xx, yy = np.meshgrid(np.linspace(0, data.shape[0]-1, data.shape[0]), np.linspace(0, data.shape[1]-1, data.shape[1]))
        zz = np.zeros(xx.shape)
        ax.plot_surface(xx, yy, zz, rstride=1, cstride=1, facecolors=plt.cm.plasma(data), shade=False)
        ax.set_xlabel('x')
        ax.set_ylabel('y')

        plot_tree(rrt, ax1)
        plot_dist(policy, rrt, ax1, data)
        ax.view_init(40, -60)

    rrt = RRTConnectEnv(l2_config, l2_data_dict)
    policy = Policy(l2_config['num_feat'])
    policy.load_model('good_models/model_envA1.ckpt')

    obs = rrt.reset()
    done = False


    # idx = 0
    # while True:
    #     action = policy.get_action(obs)
    #     obs, reward, done, _ = rrt.step(action)
        
    #     if done:
    #         break
    #     idx += 1
    #     if idx % 5000 == 0:
    #         print("Idx: " + str(idx))

    # print("Solved in: " + str(idx))
    # exit()
    # ~ 35k total iterations

    iterations = [100, 900, 5000, 20000, 8000, 700]


    idx = 0
    for j in range(len(iterations)):

        for i in range(iterations[j]):
            action = policy.get_action(obs)
            obs, reward, done, _ = rrt.step(action)
            
            idx += 1

        print("iteration: " + str(j))

        plt.cla()
        plot_all(policy, rrt, ax1, data)
        rrt.tree.show(ax2, im=data)

        plt.show(block=False)
        plt.pause(0.1)

        fig1.savefig('images/dist' + str(idx) + '.png', bbox_inches='tight')        
        fig2.savefig('images/tree' + str(idx) + '.png', bbox_inches='tight')


    # for i in range(100):
    #     action = policy.get_action(obs)
    #     obs, reward, done, _ = rrt.step(action)
        
    #     idx += 1


    # plot_all(policy, rrt, ax1, data)
    # plt.show(block=False)
    # plt.pause(0.1)

    print("Done")
    plt.show()



if __name__ == '__main__':
    # plot_model_a1()
    # dist_model_a1()
    plot_policy_comparisons()





