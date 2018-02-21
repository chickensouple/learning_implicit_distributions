import numpy as np
import matplotlib.pyplot as plt

class Tree(object):
    def __init__(self):
        # each node will store a state and additional info
        # state is the point in state space
        # node_info is extra info about the state
        # for examples, costs, cached values, paths, etc.
        self.node_states = []
        self.node_info = []

        # maps a child_idx to a parent_idx
        self.c_p_edges = dict()

        # maps a parent_idx to a list of child_idx
        self.p_c_edges = dict()

    def clear(self):
        self.node_states = []
        self.node_info = []
        self.c_p_edges = dict()
        self.p_c_edges = dict()

    def insert_node(self, node_state, node_info=None, parent_idx=None):
        self.node_states.append(node_state)
        self.node_info.append(node_info)
        if parent_idx != None:
            self.insert_edge(len(self.node_states)-1, parent_idx)

    def insert_edge(self, child_idx, parent_idx):
        self.c_p_edges[child_idx] = parent_idx
        if parent_idx in self.p_c_edges:
            self.p_c_edges[parent_idx].append(child_idx)
        else:
            self.p_c_edges[parent_idx] = [child_idx]

    def update_parent(self, child_idx, new_parent_idx):
        old_parent_idx = self.c_p_edges[child_idx]
        
        # removing child from old parent list
        self.p_c_edges[old_parent].remove(child_idx)
        if len(self.p_c_edges[old_parent]) == 0:
            self.p_c_edges.pop(old_parent)

        self.insert_edge(child_idx, new_parent_idx)

    def apply_to_children(self, idx, func):
        """
        Applies a function to a node and all of its children
        
        Args:
            idx (TYPE): Description
            func (TYPE): Description
        """
        func(self.node_states[idx])

        if idx not in self.p_c_edges:
            return

        node_list = self.p_c_edges[idx]
        for node in node_list:
            self.apply_to_children(node, func)

    def path_to_root(self, idx):
        curr_idx = idx

        path = [curr_idx]
        while curr_idx != 0:
            curr_idx = self.c_p_edges[curr_idx]
            path.append(curr_idx)
        return path

    def closest_idx(self, node, dist, return_dist=False):
        dists = dist(np.array(self.node_states), node)

        idx = np.argmin(dists)
        
        if return_dist:
            return idx, dists[idx]
        else:
            return idx

    def show(self, ax=None, im=None, goal=None, path_idx=None, color='green'):
        def plot_line(node1, node2, **kwargs):
            plt.plot([node1[1], node2[1]], [node1[0], node2[0]], **kwargs)

        def plot_path(curr_node, path, **kwargs):
            temp_node = curr_node
            for node in path:
                plot_line(temp_node, node, **kwargs)
                temp_node = node

        if ax == None:
            ax = plt.gca()

        if np.any(im != None):
            plt.imshow(1-im, interpolation='nearest', 
                extent=[0, im.shape[0], 0, im.shape[1]],
                origin='lower',
                cmap='Greys')
        if np.all(goal != None):
            plt.scatter(goal[1], goal[0], color='red')        
        for node1_idx, node2_idx in self.c_p_edges.iteritems():
            node1 = self.node_states[node1_idx]
            node2 = self.node_states[node2_idx]
            plot_line(node1, node2, color=color)

        if path_idx == None:
            return
        path = self.path_to_root(path_idx)
        path.reverse()
        for i in range(len(path)):
            if i == 0:
                continue
            node1 = self.node_states[path[i-1]]
            node2 = self.node_states[path[i]]
            plot_line(node1, node2, color='red')


