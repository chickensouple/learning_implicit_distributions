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
from rrt_bi_env import RRTBiEnv





def plot_policy_comparisons():
    policy = Policy(1)
    policy.load_model('good_models/model_envA1.ckpt')


    # dynamic domain
    plt.subplot(1, 2, 1)
    obs = np.array([np.linspace(-10, 10, 100)]).T
    fd = {policy.state_input: obs, policy.is_train: np.array(False)}
    probs = policy.sess.run(policy.prob, feed_dict=fd)

    plt.title("Flytrap Train Environment")

    plt.plot(obs, probs[:, 0], c='b', label='Learned Policy', linewidth=4.)
    plt.xlabel("feature")
    plt.ylabel("p(accept)")

    dd_rrt = np.heaviside(obs, 1)
    dd_rrt = dd_rrt[::-1]
    plt.plot(obs, dd_rrt, c='r', linestyle='--', label='Dynamic Domain', linewidth=4.)
    plt.legend(loc='lower left')

    policy.load_model('data/model_envB3.ckpt')
    plt.subplot(1, 2, 2)
    obs = np.array([np.linspace(-2, 10, 100)]).T
    fd = {policy.state_input: obs, policy.is_train: np.array(False)}
    probs = policy.sess.run(policy.prob, feed_dict=fd)

    plt.title("Flytrap BallTree Environment")

    plt.plot(obs, probs[:, 0], c='b', label='Learned Policy', linewidth=4.)
    plt.xlabel("feature")
    plt.ylabel("p(accept)")

    dd_rrt = np.heaviside(obs, 1)
    plt.plot(obs, dd_rrt, c='r', linestyle='--', label='BallTree', linewidth=4.)
    plt.legend(loc='lower right')


    plt.show()

def plot_feat(policy, **kwargs):
    obs = np.array([np.linspace(-10, 10, 100)]).T
    fd = {policy.state_input: obs, policy.is_train: np.array(False)}
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
        if i == 0:
            plot_feat(policy, linestyle='-', c=color, linewidth=4.0)
        else:
            plot_feat(policy, linestyle='--', c=color, linewidth=4.0)

    policy.load_model('good_models/model_envA1.ckpt')
    plot_feat(policy, linestyle='-', c='b', markersize=2, linewidth=4.0)

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

        fd = {policy.state_input: feats, policy.is_train: np.array(False)}
        probs = policy.sess.run(policy.prob, feed_dict=fd)

        probs = probs[:, 0]
        prob_sum = np.sum(probs)
        probs /= prob_sum
        ax.scatter(yv, xv, probs, alpha=0.5)

    def plot_all(policy, rrt, ax, data):
        ax.clear()
        xx, yy = np.meshgrid(np.linspace(0, data.shape[0]-1, data.shape[0]), np.linspace(0, data.shape[1]-1, data.shape[1]))
        zz = np.zeros(xx.shape)
        ax.plot_surface(xx, yy, zz, rstride=1, cstride=1, facecolors=plt.cm.Greys(1-data), shade=False)
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



def plot_arm_policy():
    policy = Policy(4)
    policy.load_model('good_models/model_envArm3/model_envArm3.ckpt.140.ckpt')

    # # feat is [dist to joint1, dist to joint3, dist to joint4, dist to current tree]
    # feat1 = np.linspace(0, 0.5, 20)
    # feat2 = np.linspace(0, 0.5, 20)
    # feat3 = np.linspace(0, 0.5, 20)
    # feat4 = np.linspace(0, 0.5, 20)


    # feat_list = []
    # for f1 in feat1:
    #     for f2 in feat2:
    #         for f3 in feat3:
    #             for f4 in feat4:
    #                 feat_list.append(np.array([f1, f2, f3, f4]))
    
    # feat_list = np.array(feat_list)
    # probs = policy.get_prob(feat_list)
    # accept_probs = probs[:, 0]
    

    feat1 = np.linspace(0, 0.5, 2000)
    feat2 = np.linspace(0, 0.5, 2000)
    feat3 = np.linspace(0, 0.5, 2000)
    feat4 = np.linspace(0, 1., 2000)
    feat_list = []

    for f1 in feat1:
        feat_list.append(np.array([f1, 0.01, 0.01, 0.1]))
    # for f2 in feat2:
    #     feat_list.append(np.array([0.01, f2, 0.01, 0.1]))
    # for f3 in feat3:
    #     feat_list.append(np.array([0.01, 0.01, f3, 0.1]))
    # for f4 in feat4:
    #     feat_list.append(np.array([0.5, 0.01, 0.01, f4]))

    feat_list = np.array(feat_list)


    probs = policy.get_prob(feat_list)
    accept_probs = probs[:, 0]
    plt.plot(feat4, accept_probs)
    plt.show()





def sample_points(npts, config, policy, env, map_info):
    accepted_pts = np.zeros((0, 7))

    pts_to_sample = npts

    while len(accepted_pts) < npts:
        feats = []
        pts = []
        for i in range(pts_to_sample):
            sample = config['random_sample'](map_info)
            pts.append(sample)
            feats.append(config['feat'](sample, env.trees, map_info, 1))

        feats = np.array(feats)
        pts = np.array(pts)
        actions = policy.get_actions(feats)


        accepted_pts = np.append(accepted_pts, pts[actions, :], axis=0)

        pts_to_sample -= np.sum(actions)

    return accepted_pts


def sample_points2(npts, config, policy, env, map_info):
    feats = []
    pts = []
    for i in range(npts):
        sample = config['random_sample'](map_info)
        pts.append(sample)
        feats.append(config['feat'](sample, env.trees, map_info, env.tree_idx))

    feats = np.array(feats)
    pts = np.array(pts)

    accept_probs = policy.get_probs(feats)
    return pts, accept_probs


def plot_pointcloud():
    import rospy
    import scipy.io
    import sensor_msgs.point_cloud2
    from sensor_msgs.msg import PointCloud2
    from sensor_msgs.msg import PointField
    from std_msgs.msg import Header
    import arm

    mat = scipy.io.loadmat('pointclouddata/processed_11_color.mat')
    color_pointcloud = mat['save_struct'][0, 0]['points']


    mat2 = scipy.io.loadmat('pointclouddata/processed_11.mat')
    pointcloud = mat2['save_struct'][0, 0]

    qstart = np.array([90, 10, 0, -150, 0, 0, 0]) * math.pi / 180
    qgoal = np.array([85, -40, 0, -90, 90, 0, 0]) * math.pi / 180

    arm_data_dict = arm.arm_map_create(pointcloud, qstart, qgoal)
    arm_random_sampler = partial(arm.arm_random_sample, eps=0.1)
    arm_config = {'collision_check': arm.arm_collision_check,
                  'random_sample': arm_random_sampler,
                  'steer': arm.arm_steer,
                  'dist': arm.arm_dist_func,
                  'goal_region': arm.arm_goal_region,
                  'feat': arm.arm_feat_bi}

    policy = Policy(4)
    policy.load_model('good_models/model_envArm3/model_envArm3.ckpt.140.ckpt')


    env = RRTBiEnv(arm_config, arm_data_dict)

    def create_pointcloudrgb_msg(pointcloud):
        pc = np.array(pointcloud, dtype=np.float32)

        xfield = PointField()
        xfield.name = 'x'
        xfield.datatype = PointField.FLOAT32
        xfield.offset = 0
        xfield.count = 1


        yfield = PointField()
        yfield.name = 'y'
        yfield.datatype = PointField.FLOAT32
        yfield.offset = 4
        yfield.count = 1


        zfield = PointField()
        zfield.name = 'z'
        zfield.datatype = PointField.FLOAT32
        zfield.offset = 8
        zfield.count = 1


        rgbfield = PointField()
        rgbfield.name = 'rgb'
        rgbfield.datatype = PointField.FLOAT32 # UINT32?
        rgbfield.offset = 12
        rgbfield.count = 1


        fields = [xfield, yfield, zfield, rgbfield]
        header = Header()
        header.frame_id = 'map'

        msg = sensor_msgs.point_cloud2.create_cloud(header, fields, pointcloud)
        return msg

    def create_pointcloud_msg(pointcloud):
        header = Header()
        header.frame_id = 'map'
        msg = sensor_msgs.point_cloud2.create_cloud_xyz32(header, pointcloud)
        return msg

    # ros initialization
    rospy.init_node('pc')
    env_pub = rospy.Publisher('/processed_pc', PointCloud2, queue_size=1)
    policy_pub = rospy.Publisher('/policy_samples', PointCloud2, queue_size=1)
    rand_pub = rospy.Publisher('/random_samples', PointCloud2, queue_size=1)


    pc_msg = create_pointcloudrgb_msg(color_pointcloud)


    np.random.seed(1)
    # Planning
    print("Starting planning")
    obs = env.reset()
    i = 0
    while i < 100:
        action = policy.get_action(obs)
        obs, reward, done, _ = env.step(action)
        if done:
            print("done")
            break
        i += 1
    print("Ended Planning")

    num_nodes = len(env.trees[0].node_states) + len(env.trees[1].node_states)
    print("Num Nodes: " + str(num_nodes))


    print("Sampling Points")
    samples, accept_probs = sample_points2(400, arm_config, policy, env, arm_data_dict)
    print("Done Sampling Points")
    accept_probs = accept_probs / np.sum(accept_probs)
    accept_probs = np.array([accept_probs]).T

    def get_end_pts(samples):
        pts = []
        for sample in samples:
            _, pt = arm.kinematics_forward_l_default(sample)
            pts.append(pt[4])
        pts = np.array(pts, dtype=np.float32)
        return pts

    end_pts = get_end_pts(samples)
    policy_pts = np.append(end_pts, accept_probs, axis=1)
    sample_pts = np.append(end_pts, np.ones(accept_probs.shape) / len(accept_probs), axis=1)

    pc_sample_policy = create_pointcloudrgb_msg(policy_pts)
    pc_sample_default = create_pointcloudrgb_msg(sample_pts)


    rate = rospy.Rate(5)
    while not rospy.is_shutdown():
        env_pub.publish(pc_msg)
        policy_pub.publish(pc_sample_policy)
        rand_pub.publish(pc_sample_default)
        rate.sleep()


def plot_flytrap_results2():
    test_data = np.array([[-25175.06 , 9594.86 , 1379.99 , 408.87 , 23550.35 , 9290.34 , 66.25 , 23.27 , 24572.26 , 9496.08],
        [-5871.18 , 2348.53 , 1200.80 , 390.08 , 4345.09 , 1917.38 , 59.78 , 22.18 , 32629.19 , 13741.52 ],
        [-13687.95 , 11152.37 , 4532.97 , 3689.05 , 9066.31 , 7389.49 , 29.08 , 3.09 , 9067.31 , 7389.49],
        [-7317.60 , 5180.82 , 1903.06 , 1289.92 , 5263.22 , 3778.53 , 28.87 , 2.98 , 15331.89 , 11297.62],
        [-11622.73 , 17522.95 , 3227.25 , 4028.50 , 4787.72 , 6024.75 , 33.88 , 6.12 , 360976.14 , 780852.07],
        [-10734.36 , 18960.83 , 1677.75 , 2135.32 , 3093.38 , 3997.80 , 33.17 , 7.03 , 596522.64 , 1321001.90]])

    test_data[:, 0] *= -1

    ind = np.arange(3, dtype=np.float)
    width = 0.4


    labels = ['Original Distribution', 'Trained Distribution']

    def autolabel(rects, vals):
        """
        Attach a text label above each bar displaying its height
        """
        for idx, (rect, val) in enumerate(zip(rects, vals)):
            height = rect.get_height()
            if idx == 3:
                ax.text(rect.get_x() + rect.get_width()/2., 1.05*height,
                        '%0.1f' % val,
                        ha='center', va='bottom')
            else:
                ax.text(rect.get_x() + rect.get_width()/2., 1.05*height,
                        '%d' % val,
                        ha='center', va='bottom')

    titles = ['Search Cost (Negative Reward)', 'Number of Tree Nodes', 'Number of Collision Checks', 'Path Length (m)', 'Number of Samples']


    for i in range(5):
        fig, ax = plt.subplots()
        idx1 = i*2
        idx2 = idx1 + 1


        rects1 = ax.bar(ind, 
            test_data[0::2, idx1], 
            width, 
            color='r', 
            yerr=test_data[0::2, idx2], 
            ecolor='tab:gray', 
            capsize=10,
            error_kw={'elinewidth': 1})

        rects2 = ax.bar(ind+width, 
            test_data[1::2, idx1],
            width, 
            color='b', 
            yerr=test_data[1::2, idx2], 
            ecolor='tab:gray', 
            capsize=10,
            error_kw={'elinewidth': 1})



        if i == 0:
            ax.legend((rects1[0], rects2[0]), (labels[0], labels[1]), loc='upper right')

        ax.set_xticks(ind + width/2)
        ax.set_xticklabels(('RRT', 'BiRRT', 'EST'))
        ax.get_yaxis().set_ticks([])



        ylim = ax.get_ylim()
        ylim = list(ylim)
        ylim[0] = 0
        ax.set_ylim(ylim)



        autolabel(rects1, test_data[0::2, idx1])
        autolabel(rects2, test_data[1::2, idx1])
        plt.title(titles[i])

    plt.show()


def plot_arm_results2():
    test_data = np.array([[-9145.32 , 16763.48 , 488.57 , 804.32 , 8573.01 , 15801.46 , 11.46 , 4.00 , 8574.01 , 15801.46],
        [-3326.28 , 4495.21 , 86.04 , 69.61 , 3110.68 , 4228.76 , 11.04 , 3.17 , 13155.94 , 20077.12]])

    test_data[:, 0] *= -1

    ind = np.arange(1, dtype=np.float)
    width = 0.55

    labels = ['Cost', 'Num Nodes', 'Collision Checks', 'Path Length', 'Num Samples']

    def autolabel(rects, vals, offset=0):
        """
        Attach a text label above each bar displaying its height
        """
        for idx, (rect, val) in enumerate(zip(rects, vals)):
            # height = rect.get_height()
            width = rect.get_width()
            # ax.text(rect.get_x() + rect.get_width()/2., 1.05*height,
            #         '%0.1f' % val,
            #         ha='center', va='bottom')
            ax.text(width+offset, rect.get_y() + rect.get_height()/1.,
                    '%0.1f' % val,
                    ha='center', va='bottom')


    offsets = [1800, 80, 1800, 1, 3200]

    for i in range(5):
        ax = plt.subplot(5, 1, i+1)
        idx1 = i*2
        idx2 = idx1 + 1


        rects1 = ax.barh(ind, 
            test_data[0, idx1], 
            width, 
            color='r', 
            xerr=test_data[0, idx2], 
            ecolor='tab:gray', 
            capsize=10,
            error_kw={'elinewidth': 1})

        rects2 = ax.barh(ind+width, 
            test_data[1, idx1], 
            width, 
            color='b', 
            xerr=test_data[1, idx2], 
            ecolor='tab:gray', 
            capsize=10,
            error_kw={'elinewidth': 1})


        if i == 2:
            ax.legend((rects1[0], rects2[0]), ('BiRRT', 'Trained BiRRT'), loc='right')

        ax.set_yticks(ind + width/2)
        ax.set_yticklabels((labels[i],), minor=False)
        # plt.yticks(ind + width/2, labels[i])
        ax.invert_yaxis()
        ax.get_xaxis().set_ticks([])

        xlim = ax.get_xlim()
        xlim = list(xlim)
        xlim[0] = 0
        ax.set_xlim(xlim)

        autolabel(rects1, [test_data[0, idx1]], offsets[i])
        autolabel(rects2, [test_data[1, idx1]], offsets[i])

    plt.show()

if __name__ == '__main__':
    # plt.rcParams.update({'font.size': 16})
    plt.rcParams.update({'font.size': 22})
    plt.rcParams.update({'font.weight': 'bold'})
    # font = {'family' : 'normal',
    #         'weight' : 'bold',
    #         'size'   : 18}

    # plt.rc('font', **font)

    # plot_model_a1()
    # dist_model_a1()
    plot_policy_comparisons()
    # plot_arm_policy()
    # plot_pointcloud()
    # plot_flytrap_results2()
    # plot_arm_results2()



