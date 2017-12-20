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

            # layer1 = tf.contrib.layers.fully_connected(self.state_input, 
            #     32, 
            #     activation_fn=tf.nn.elu,
            #     scope='layer1')
            # layer2 = tf.contrib.layers.fully_connected(layer1, 
            #     16, 
            #     activation_fn=tf.nn.elu,
            #     scope='layer2')

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
            
            self.value = tf.contrib.layers.fully_connected(layer2, 
                1,
                activation_fn=None)
            self.value = tf.squeeze(self.value)

            self.reward_disc = tf.placeholder(tf.float32, [None], name='reward_disc')
            self.loss = tf.reduce_mean(tf.square(self.reward_disc - self.value))

            update_ops = tf.get_collection(tf.GraphKeys.UPDATE_OPS, scope=name)
            with tf.control_dependencies(update_ops):
                self.optimize = tf.train.AdamOptimizer(1e-3).minimize(self.loss)

    def get_baseline(self, states):
        fd = {self.state_input: states, self.is_train: np.array(False)}
        value = self.sess.run(self.value, feed_dict=fd)
        return value

    def train(self, states, reward_disc):
        fd = {self.state_input: states, self.reward_disc: reward_disc, self.is_train: np.array(True)}
        loss, _ = self.sess.run([self.loss, self.optimize], feed_dict=fd)
        return loss

