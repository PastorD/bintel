"""                                                                                                               
Implementation of CORE-RL with fixed regularization weight 
using DDPG as the baseline RL algorithm  

The algorithm is tested on an experimental car-following task
"""

import tensorflow as tf
import numpy as np
import tflearn
import argparse
import pprint as pp
from scipy.io import savemat
import datetime
import random

from prior import BasePrior
from car_dat import allCars

from replay_buffer import ReplayBuffer

from sklearn.svm import SVR

# ===========================
#   Actor and Critic DNNs
# ===========================

class ActorNetwork(object):
    """
    Input to the network is the state, output is the action
    under a deterministic policy.

    The output layer activation is a tanh to keep the action
    between -action_bound and action_bound
    """

    def __init__(self, sess, state_dim, action_dim, action_bound, learning_rate, tau, batch_size):
        self.sess = sess
        self.s_dim = state_dim
        self.a_dim = action_dim
        self.action_bound = action_bound
        self.learning_rate = learning_rate
        self.tau = tau
        self.batch_size = batch_size

        # Actor Network
        self.inputs, self.out, self.scaled_out = self.create_actor_network()

        self.network_params = tf.trainable_variables()

        # Target Network
        self.target_inputs, self.target_out, self.target_scaled_out = self.create_actor_network()

        self.target_network_params = tf.trainable_variables()[
            len(self.network_params):]

        # Op for periodically updating target network with online network
        # weights
        self.update_target_network_params = \
            [self.target_network_params[i].assign(tf.multiply(self.network_params[i], self.tau) +
                                                  tf.multiply(self.target_network_params[i], 1. - self.tau))
                for i in range(len(self.target_network_params))]

        # This gradient will be provided by the critic network
        self.action_gradient = tf.placeholder(tf.float32, [None, self.a_dim])

        # Combine the gradients here
        self.unnormalized_actor_gradients = tf.gradients(self.scaled_out, self.network_params, -self.action_gradient)
        #Normalize gradient
        self.actor_gradients = list(map(lambda x: tf.div(x, self.batch_size), self.unnormalized_actor_gradients))
                
        # Optimization Op
        self.optimize = tf.train.AdamOptimizer(self.learning_rate).apply_gradients(zip(self.actor_gradients, self.network_params))

        self.num_trainable_vars = len(self.network_params) + len(self.target_network_params)

    def create_actor_network(self):
        w_init = tflearn.initializations.uniform(minval=-0.1, maxval=0.1)
        inputs = tflearn.input_data(shape=[None, self.s_dim])
        net = tflearn.fully_connected(inputs, 50, weights_init=w_init)
        net = tflearn.layers.normalization.batch_normalization(net)
        net = tflearn.activations.relu(net)
        net = tflearn.fully_connected(net, 40, weights_init=w_init)
        net = tflearn.layers.normalization.batch_normalization(net)
        net = tflearn.activations.relu(net)
        out = tflearn.fully_connected(
            net, self.a_dim, activation='tanh', weights_init=w_init)
        # Scale output to -action_bound to action_bound
        scaled_out = tf.multiply(out, self.action_bound)
        return inputs, out, scaled_out

    def train(self, inputs, a_gradient):
        self.sess.run(self.optimize, feed_dict={
            self.inputs: inputs,
            self.action_gradient: a_gradient
        })

    def predict(self, inputs):
        return self.sess.run(self.scaled_out, feed_dict={
            self.inputs: inputs
        })

    def predict_target(self, inputs):
        return self.sess.run(self.target_scaled_out, feed_dict={
            self.target_inputs: inputs
        })

    def update_target_network(self):
        self.sess.run(self.update_target_network_params)

    def get_num_trainable_vars(self):
        return self.num_trainable_vars


class CriticNetwork(object):
    """
    Input to the network is the state and action, output is Q(s,a).
    The action must be obtained from the output of the Actor network.

    """

    def __init__(self, sess, state_dim, action_dim, learning_rate, tau, gamma, num_actor_vars):
        self.sess = sess
        self.s_dim = state_dim
        self.a_dim = action_dim
        self.learning_rate = learning_rate
        self.tau = tau
        self.gamma = gamma

        # Create the critic network
        self.inputs, self.action, self.out = self.create_critic_network()

        self.network_params = tf.trainable_variables()[num_actor_vars:]

        # Target Network
        self.target_inputs, self.target_action, self.target_out = self.create_critic_network()

        self.target_network_params = tf.trainable_variables()[(len(self.network_params) + num_actor_vars):]

        # Op for periodically updating target network with online network
        # weights with regularization
        self.update_target_network_params = \
            [self.target_network_params[i].assign(tf.multiply(self.network_params[i], self.tau) \
            + tf.multiply(self.target_network_params[i], 1. - self.tau))
                for i in range(len(self.target_network_params))]

        # Network target (y_i)
        self.predicted_q_value = tf.placeholder(tf.float32, [None, 1])

        # Define loss and optimization Op
        self.loss = tflearn.mean_square(self.predicted_q_value, self.out)
        self.optimize = tf.train.AdamOptimizer(
            self.learning_rate).minimize(self.loss)

        # Get the gradient of the net w.r.t. the action.
        # For each action in the minibatch (i.e., for each x in xs),
        # this will sum up the gradients of each critic output in the minibatch
        # w.r.t. that action. Each output is independent of all
        # actions except for one.
        self.action_grads = tf.gradients(self.out, self.action)

    def create_critic_network(self):
        inputs = tflearn.input_data(shape=[None, self.s_dim])
        action = tflearn.input_data(shape=[None, self.a_dim])
        net = tflearn.fully_connected(inputs, 50)
        net = tflearn.layers.normalization.batch_normalization(net)
        net = tflearn.activations.relu(net)

        # Add the action tensor in the 2nd hidden layer
        # Use two temp layers to get the corresponding weights and biases
        t1 = tflearn.fully_connected(net, 40)
        t2 = tflearn.fully_connected(action, 40)


        net = tflearn.activation(
            tf.matmul(net, t1.W) + tf.matmul(action, t2.W) + t2.b, activation='relu')

        # linear layer connected to 1 output representing Q(s,a)
        # Weights are init to Uniform[-3e-3, 3e-3]
        w_init = tflearn.initializations.uniform(minval=-0.003, maxval=0.003)
        out = tflearn.fully_connected(net, 1, weights_init=w_init)
        return inputs, action, out

    def train(self, inputs, action, predicted_q_value):
        return self.sess.run([self.out, self.optimize], feed_dict={
            self.inputs: inputs,
            self.action: action,
            self.predicted_q_value: predicted_q_value
        })

    def predict(self, inputs, action):
        return self.sess.run(self.out, feed_dict={
            self.inputs: inputs,
            self.action: action
        })
    
    def predict_target(self, inputs, action):
        return self.sess.run(self.target_out, feed_dict={
            self.target_inputs: inputs,
            self.target_action: action
        })

    def action_gradients(self, inputs, actions):
        return self.sess.run(self.action_grads, feed_dict={
            self.inputs: inputs,
            self.action: actions
        })

    def update_target_network(self):
        self.sess.run(self.update_target_network_params)


class OrnsteinUhlenbeckActionNoise:
    def __init__(self, mu, sigma=0.3, theta=.15, dt=1e-2, x0=None):
        self.theta = theta
        self.mu = mu
        self.sigma = sigma
        self.dt = dt
        self.x0 = x0
        self.reset()

    def __call__(self):
        x = self.x_prev + self.theta * (self.mu - self.x_prev) * self.dt + \
                self.sigma * np.sqrt(self.dt) * np.random.normal(size=self.mu.shape)
        self.x_prev = x
        return x

    def reset(self):
        self.x_prev = self.x0 if self.x0 is not None else np.zeros_like(self.mu)

    def __repr__(self):
        return 'OrnsteinUhlenbeckActionNoise(mu={}, sigma={})'.format(self.mu, self.sigma)

# ===========================
#   Tensorflow Summary Ops
# ===========================

def build_summaries():
    episode_reward = tf.Variable(0.)
    tf.summary.scalar("Reward", episode_reward)
    episode_ave_max_q = tf.Variable(0.)
    tf.summary.scalar("Qmax Value", episode_ave_max_q)

    summary_vars = [episode_reward, episode_ave_max_q]
    summary_ops = tf.summary.merge_all()

    return summary_ops, summary_vars

def moving_average(a, n=3) :
    ret = np.cumsum(a, dtype=float)
    ret[n:] = ret[n:] - ret[:-n]
    return ret[n - 1:] / n


class Learner(object):
    """ 
    DDPG Learner 
    The action must be obtained from the output of the Actor network.    
    """

    def __init__(self, sess, env, state_dim, action_dim, action_bound, actor_lr, critic_lr, tau, gamma, batch_size):
        self.sess = sess
        self.env = env
        self.s_dim = state_dim
        self.a_dim = action_dim
        self.a_bound = action_bound
        self.actor_lr = actor_lr
        self.critic_lr = critic_lr
        self.tau = tau
        self.gamma = gamma
        self.batch_size = batch_size

        self.actor = ActorNetwork(self.sess, self.s_dim, self.a_dim, self.a_bound,
                             self.actor_lr, self.tau, self.batch_size)
        self.critic = CriticNetwork(self.sess, self.s_dim, self.a_dim,
                               self.critic_lr, self.tau, self.gamma,
                               self.actor.get_num_trainable_vars())
        self.actor_noise = OrnsteinUhlenbeckActionNoise(mu=np.zeros(self.a_dim))

        self.sess.run(tf.global_variables_initializer())
        #tflearn.is_training(True)
        
    # Training based on collected data
    def train(self, replay_buffer, minibatch_size):
        # Get dynamics and initialize prior controller  
        prior = BasePrior()

        #self.sess.as_default()
        
        # Needed to enable BatchNorm
        #tflearn.is_training(True)

        #Sample a batch from the replay buffer                                                                                         
        s_batch, a_batch, r_batch, t_batch, s2_batch = replay_buffer.sample_batch(minibatch_size)
        
        # Calculate targets                                                                                                            
        target_q = self.critic.predict_target(s2_batch, self.actor.predict_target(s2_batch))
        y_i = []
        for k in range(minibatch_size):
            if t_batch[k]:
                y_i.append(r_batch[k])
            else:
                y_i.append(r_batch[k] + self.critic.gamma * target_q[k])

        # Update the critic given the targets                                                                                          
        predicted_q_value, _ = self.critic.train(s_batch, a_batch, np.reshape(y_i, (minibatch_size, 1)))
        ep_ave_max_q += np.amax(predicted_q_value)
        
        # Update the actor policy using the sampled gradient                                                                           
        a_outs = self.actor.predict(s_batch)
        grads = self.critic.action_gradients(s_batch, a_outs)
        self.actor.train(s_batch, grads[0])
        
        # Update target networks                                                                                                       
        self.actor.update_target_network()
        self.critic.update_target_network()
        
    
    # Training loop within simulator
    def train_rollout(self, args, reward_result):
        # Set up summary Ops      
        summary_ops, summary_vars = build_summaries()
        
        # Get dynamics and initialize prior controller
        prior = BasePrior()
        
        # Initialize target network weights  
        self.actor.update_target_network()
        self.critic.update_target_network()

        # Initialize replay memory  
        replay_buffer = ReplayBuffer(int(args['buffer_size']), int(args['random_seed']))
        
        # Needed to enable BatchNorm. 
        tflearn.is_training(True)

        paths = list()
        
        lambda_store = np.zeros((int(args['max_episode_len']),1))
        
        for i in range(int(args['max_episodes'])):

            s = self.env.reset_inc()
            
            ep_reward = 0.
            ep_ave_max_q = 0
            
            obs, action, act_prior, rewards, obs_ref, prior_ref, collisions = [], [], [], [], [], [], []

            #Get reward using baseline controller
            s0 = np.copy(s)
            ep_reward_opt = 0.
            for kk in range(int(args['max_episode_len'])):
                a = self.env.getPrior()
                prior_ref.append(np.array([a]))
                s0, r, stop_c, act = self.env.step(a)
                ep_reward_opt += r
                obs_ref.append(s0)
                if (stop_c):
                    break

            # Get reward using regRL algorithm       
            s = self.env.reset()
            
            for j in range(int(args['max_episode_len'])):
                
                # Set control prior regularization weight 
                lambda_mix = 15.
                lambda_store[j] = lambda_mix

                # Get control prior          
                a_prior = self.env.getPrior()
                
                # Rl control with exploration noise 
                ab = self.actor.predict(np.reshape(s, (1, self.actor.s_dim))) + self.actor_noise()
                
                # Mix the actions (RL controller + control prior)  
                act = ab[0]/(1+lambda_mix) + (lambda_mix/(1+lambda_mix))*a_prior
                
                # Take action and observe next state/reward 
                s2, r, terminal, act = self.env.step(act)
                collisions.append(self.env.collision_flag)
                act = np.array(act, ndmin=1)

                # Add info from time step to the replay buffer   
                replay_buffer.add(np.reshape(s, (self.actor.s_dim,)), np.reshape(ab, (self.actor.a_dim,)), r,
                                  terminal, np.reshape(s2, (self.actor.s_dim,)))

                # Keep adding experience to the memory until   
                # there are at least minibatch size samples  
                if replay_buffer.size() > int(args['minibatch_size']):

                    #Sample a batch from the replay buffer  
                    s_batch, a_batch, r_batch, t_batch, s2_batch = replay_buffer.sample_batch(int(args['minibatch_size']))

                    # Calculate targets    
                    target_q = self.critic.predict_target(
                        s2_batch, self.actor.predict_target(s2_batch))
                    y_i = []
                    for k in range(int(args['minibatch_size'])):
                        if t_batch[k]:
                            y_i.append(r_batch[k])
                        else:
                            y_i.append(r_batch[k] + self.critic.gamma * target_q[k])

                    # Update the critic given the targets 
                    predicted_q_value, _ = self.critic.train(s_batch, a_batch, np.reshape(y_i, (int(args['minibatch_size']), 1)))
                    ep_ave_max_q += np.amax(predicted_q_value)

                    # Update the actor policy using the sampled gradient  
                    a_outs = self.actor.predict(s_batch)
                    grads = self.critic.action_gradients(s_batch, a_outs)
                    self.actor.train(s_batch, grads[0])
                    # Update target networks   
                    self.actor.update_target_network()
                    self.critic.update_target_network()

                s = s2
                ep_reward += r

                obs.append(s)
                rewards.append(r)
                action.append(act)
                act_prior.append(np.array([a_prior]))

                # Collect results at end of episode 
                if terminal:
                    print('| Reward: {:d} | Episode: {:d} | Qmax: {:.4f}'.format(int(ep_reward - ep_reward_opt), i, (ep_ave_max_q / float(j))))
                    reward_result[0,i] = ep_reward
                    reward_result[1,i] = ep_reward_opt
                    reward_result[2,i] = np.mean(lambda_store)
                    reward_result[3,i] = max(collisions)
                    path = {"Observation":np.concatenate(obs).reshape((-1,6)),
                            "Observation_ref":np.concatenate(obs_ref).reshape((-1,6)),
                            "Action":np.concatenate(action),
                            "Action_Prior":np.concatenate(act_prior),
                            "Action_Prior_Ref":np.concatenate(prior_ref), 
                            "Reward":np.asarray(rewards)}
                    paths.append(path)

                    break
                
        return [summary_ops, summary_vars, paths, reward_result]



def main(args, reward_result):

    with tf.Session() as sess:

        # Initialize environment and seed
        np.random.seed(int(args['random_seed']))
        tf.set_random_seed(int(args['random_seed']))
        env = allCars()
        state_dim = 6
        action_dim = 1
        action_bound = 5

        learner = Learner(sess, env, state_dim, action_dim, action_bound, float(args['actor_lr']), float(args['critic_lr']), float(args['tau']), float(args['gamma']), float(args['minibatch_size']))

        #[summary_ops, summary_vars, paths, reward_result] = learner.train(replay_buffer, minibatch_size)
        [summary_ops, summary_vars, paths, reward_result] = learner.train_rollout(args, reward_result)
        
        return [summary_ops, summary_vars, paths]

if __name__ == '__main__':
    # Set a random seed
    rand_seed = random.randrange(1000000)
    parser = argparse.ArgumentParser(description='provide arguments for DDPG agent')

    # agent parameters
    parser.add_argument('--actor-lr', help='actor network learning rate', default=0.0001)
    parser.add_argument('--critic-lr', help='critic network learning rate', default=0.001)
    parser.add_argument('--gamma', help='discount factor for critic updates', default=0.99)
    parser.add_argument('--tau', help='soft target update parameter', default=0.001) 
    parser.add_argument('--buffer-size', help='max size of the replay buffer', default=1000000)
    parser.add_argument('--minibatch-size', help='size of minibatch for minibatch-SGD', default=64)

    # run parameters
    parser.add_argument('--env', help='choose the gym env- tested on {Pendulum-v0}', default='Pendulum-v0')
    parser.add_argument('--random-seed', help='random seed for repeatability', default=rand_seed) 
    parser.add_argument('--max-episodes', help='max num of episodes to do while training', default=2600) 
    parser.add_argument('--max-episode-len', help='max length of 1 episode', default=100) 
    parser.add_argument('--render-env', help='render the gym env', action='store_false')
    parser.add_argument('--use-gym-monitor', help='record gym results', action='store_false')
    parser.add_argument('--monitor-dir', help='directory for storing gym results', default='./results/gym_ddpg')
    parser.add_argument('--summary-dir', help='directory for storing tensorboard info', default='./results/tf_ddpg')

    parser.set_defaults(render_env=False)
    parser.set_defaults(use_gym_monitor=False)
    
    args = vars(parser.parse_args())
    
    pp.pprint(args)

    reward_result = np.zeros((4,int(args['max_episodes'])))
    [summary_ops, summary_vars, paths] = main(args, reward_result)

    # Save learning results to a MATLAB data file
    savemat('data_prior15_a_' + datetime.datetime.now().strftime("%y-%m-%d-%H-%M") + '.mat',dict(data=paths, reward=reward_result))
