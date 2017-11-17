import tensorflow as tf
import numpy as np


class Baseline(object):
    def __init__(self, num_feats, name='baseline', sess=None):
        self.num_feats = num_feats
        self.name = name

        if sess == None:
            self.sess = tf.InteractiveSession()
            tf.global_variables_initializer().run() 
        else:
            self.sess = sess

        self._build_model(name, num_feats)


    def _build_model(self, name, num_feats):
        with tf.variable_scope(name):
            self.state_input = tf.placeholder(tf.float32, [None, num_feats], name='state_input')
            
            layer1 = tf.contrib.layers.fully_connected(self.state_input, 
                32, 
                activation_fn=tf.nn.elu,
                scope='layer1')
            layer2 = tf.contrib.layers.fully_connected(layer1, 
                16, 
                activation_fn=tf.nn.elu,
                scope='layer2')
            
            self.value = tf.contrib.layers.fully_connected(layer2, 
                1,
                activation_fn=None)
            self.value = tf.squeeze(self.value)

            self.reward_disc = tf.placeholder(tf.float32, [None], name='reward_disc')
            self.loss = tf.reduce_mean(tf.square(self.reward_disc - self.value))

            self.optimize = tf.train.AdamOptimizer(1e-3).minimize(self.loss)

    def get_baseline(self, states):
        fd = {self.state_input: states}
        value = self.sess.run(self.value, feed_dict=fd)
        return value

    def train(self, states, reward_disc):
        fd = {self.state_input: states, self.reward_disc: reward_disc}
        loss, _ = self.sess.run([self.loss, self.optimize], feed_dict=fd)
        return loss

