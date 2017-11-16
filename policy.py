from tree import Tree
import matplotlib.pyplot as plt
import tensorflow as tf
import numpy as np
from utils import *

class DefaultPolicy(object):
    def __init__(self):
        pass

    def get_action(self, obs):
        return 1

class BallTreePolicy(object):
    def __init__(self):
        pass

    def get_action(self, obs):
        if obs[0] >= 0:
            return 1
        else: 
            return 0

class DynamicDomainPolicy(object):
    def __init__(self):
        pass

    def get_action(self, obs):
        if obs[0] <= 0:
            return 1
        else: 
            return 0

class Policy(object):
    def __init__(self, num_feats, sess=None):
        self.num_feats = num_feats
        self._build_model(num_feats)
        if sess == None:
            self.sess = tf.InteractiveSession()
            tf.global_variables_initializer().run() 
        else:
            self.sess = sess

    def _build_model(self, num_feats):
        with tf.variable_scope('policy'):
            self.state_input = tf.placeholder(tf.float32, [None, num_feats], name='state_input')


            # used for batch norm
            self.is_train = tf.placeholder(tf.bool, name='is_train')
            

            layer1 = tf.contrib.layers.fully_connected(self.state_input, 
                32, 
                activation_fn=tf.nn.elu,
                scope='layer1')
            
            layer1 = tf.contrib.layers.batch_norm(layer1, 
                center=True, 
                scale=True,
                is_training=self.is_train)

            layer2 = tf.contrib.layers.fully_connected(layer1, 
                16, 
                activation_fn=tf.nn.elu,
                scope='layer2')


            layer2 = tf.contrib.layers.batch_norm(layer2, 
                center=True,
                scale=True,
                is_training=self.is_train)

            self.logit = tf.contrib.layers.fully_connected(layer2,
                2,
                activation_fn=None,
                biases_initializer=None,
                scope='logit')

            # tresholding probabilities so that probability of accept or reject
            # is at least min_prob
            min_prob = 0.05
            softmax_min_prob = np.log(min_prob/(1.0-min_prob))

            accept_logit = tf.gather(self.logit, [0], axis=1)
            reject_logit = tf.gather(self.logit, [1], axis=1)

            accept_logit = tf.maximum(accept_logit, reject_logit + softmax_min_prob)
            reject_logit = tf.maximum(reject_logit, accept_logit + softmax_min_prob)

            self.logit = tf.concat([accept_logit, reject_logit], axis=1)

            with tf.variable_scope('prob'):
                # self.prob is N by 2 matrix
                # where each row corresponds to feature
                # self.prob[:, 0] is probability of accepting each feature
                # self.prob[:, 1] is probability of rejecting
                self.prob = tf.nn.softmax(self.logit, dim=1)

            # numerically stable way to calculate log probs
            self.log_probs = tf.nn.log_softmax(self.logit)

            # training
            self.selected_action = tf.placeholder(tf.int32, [None], name='selected_action')
            self.advantage = tf.placeholder(tf.float32, [None], name='advantage')

            row_indices = tf.range(0, tf.shape(self.selected_action)[0])
            concat_indices = tf.stack([row_indices, self.selected_action], axis=1)

            # negative sign to do gradient ascent instead of descent
            # reinforcement learning loss
            entropy_coeff = 0.
            prob_coeff = 0.
            selected_probs = tf.gather_nd(self.prob, concat_indices)
            selected_log_probs = tf.gather_nd(self.log_probs, concat_indices)
            self.rloss = -tf.reduce_mean(selected_log_probs * self.advantage)
            self.entropy_loss = -tf.reduce_mean(selected_probs * selected_log_probs)
            self.prob_loss = tf.reduce_mean(tf.gather(self.prob, [1], axis=1))
            self.total_loss = self.rloss + self.entropy_loss * entropy_coeff + self.prob_loss * prob_coeff

            self.network_vars = tf.get_collection(tf.GraphKeys.TRAINABLE_VARIABLES, scope='policy')
            self.grads = tf.gradients(self.total_loss, self.network_vars)

            update_ops = tf.get_collection(tf.GraphKeys.UPDATE_OPS)
            with tf.control_dependencies(update_ops):
                self.optimize = tf.train.AdamOptimizer(1e-3).apply_gradients(zip(self.grads, self.network_vars))

    def get_action(self, obs):
        fd = {self.state_input: np.array([obs]), self.is_train: np.array(False)}
        prob = self.sess.run(self.prob, feed_dict=fd).squeeze()
        accept_prob = prob[0]

        x = np.random.random_sample()
        action = int(x < accept_prob)

        return action

    def update(self, obs, actions, advantage):
        fd = {self.state_input: obs, 
            self.selected_action: actions,
            self.advantage: advantage,
            self.is_train: np.array(True)}

        # loss, _ = self.sess.run([self.total_loss, self.optimize], feed_dict=fd)
        total_loss, rloss, entropy_loss, prob_loss, _ = self.sess.run([self.total_loss, 
            self.rloss, 
            self.entropy_loss, 
            self.prob_loss, 
            self.optimize], 
            feed_dict=fd)
        return total_loss, rloss, entropy_loss, prob_loss

    def save_model(self, filename):
        saver = tf.train.Saver()
        save_path = saver.save(self.sess, filename)
        print("Model saved in file: %s" % filename)

    def load_model(self, filename):
        saver = tf.train.Saver()
        saver.restore(self.sess, filename)
        print("Model loaded from file: %s" % filename)


def count_params():
    "print number of trainable variables"
    size = lambda v: reduce(lambda x, y: x*y, v.get_shape().as_list())
    n = sum(size(v) for v in tf.trainable_variables())
    return n

if __name__ == '__main__':
    
    Policy = Policy(3)
    print "Num params:", count_params()


