import sys
import time
import signal
import argparse
import airsim

import numpy as np
import torch
import socket
from models import *
from comm import CommNetMLP
from utils import *
from action_utils import parse_action_args
from trainer import Trainer
import pickle
import os
from json_editor import Json_Editor
import pprint

timeFolderName = str(time.strftime("%a, %d %b %Y %H:%M:%S +0000", time.gmtime()))

torch.utils.backcompat.broadcast_warning.enabled = True
torch.utils.backcompat.keepdim_warning.enabled = True

torch.set_default_tensor_type('torch.DoubleTensor')

parser = argparse.ArgumentParser(description='PyTorch RL trainer')
# training
# note: number of steps per epoch = epoch_size X batch_size x nprocesses
parser.add_argument('--num_epochs', default=1, type=int,
                    help='number of training epochs')
parser.add_argument('--epoch_size', type=int, default=1,
                    help='number of update iterations in an epoch')
parser.add_argument('--max_steps', default=10000000, type=int,
                    help='force to end the game after this many steps')
parser.add_argument('--batch_size', type=int, default=200,
                    help='number of steps before each update (per thread)')
parser.add_argument('--nprocesses', type=int, default=1,
                    help='How many processes to run')
parser.add_argument('--train_thresh', type=float, default=0.3,
                    help='train threshold to stop the training')
parser.add_argument('--last_n_episode', type=int, default=10,
                    help='Last n episodes to check if training should be stopped')
parser.add_argument('--min_episode', type=int, default=500,
                    help='After min episodes to check if training should be stopped')
# model
parser.add_argument('--hid_size', default=128, type=int,
                    help='hidden layer size')
parser.add_argument('--recurrent', action='store_true', default=True,
                    help='make the model recurrent in time')
# optimization
parser.add_argument('--gamma', type=float, default=1.0,
                    help='discount factor')
# parser.add_argument('--tau', type=float, default=1.0,
#                     help='gae (remove?)')
parser.add_argument('--seed', type=int, default=-1,
                    help='random seed. Pass -1 for random seed')  # TODO: works in thread?
parser.add_argument('--normalize_rewards', action='store_true', default=False,
                    help='normalize rewards in each batch')
parser.add_argument('--lrate', type=float, default=0.001,
                    help='learning rate')
parser.add_argument('--entr', type=float, default=0,
                    help='entropy regularization coeff')
parser.add_argument('--value_coeff', type=float, default=0.01,
                    help='coeff for value loss term')

parser.add_argument('--save', default="True", type=str,
                    help='save the model after training')

parser.add_argument('--save_every', default=100, type=int,
                    help='save the model after every n_th epoch')
parser.add_argument('--load', default="False", type=str,
                    help='load the model')
parser.add_argument('--display', action="store_true", default=False,
                    help='Display environment state')
parser.add_argument('--random', action='store_true', default=False,
                    help="enable random model")

# CommNet specific args
parser.add_argument('--commnet', action='store_true', default=False,
                    help="enable commnet model")
parser.add_argument('--ic3net', action='store_true', default=True,
                    help="enable commnet model")
parser.add_argument('--nagents', type=int, default=2,
                    help="Number of agents (used in multiagent)")
parser.add_argument('--nbots', type=int, default=2,
                    help="Number of bots (used in multiagent)")
parser.add_argument('--comm_mode', type=str, default='avg',
                    help="Type of mode for communication tensor calculation [avg|sum]")
parser.add_argument('--comm_passes', type=int, default=1,
                    help="Number of comm passes per step over the model")
parser.add_argument('--comm_mask_zero', action='store_true', default=False,
                    help="Whether communication should be there")
parser.add_argument('--mean_ratio', default=1.0, type=float,
                    help='how much coooperative to do? 1.0 means fully cooperative')
parser.add_argument('--rnn_type', default='MLP', type=str,
                    help='type of rnn to use. [LSTM|MLP]')
parser.add_argument('--detach_gap', default=10, type=int,
                    help='detach hidden state and cell state for rnns at this interval.'
                    + ' Default 10000 (very high)')
parser.add_argument('--comm_init', default='uniform', type=str,
                    help='how to initialise comm weights [uniform|zeros]')
parser.add_argument('--hard_attn', default=False, action='store_true',
                    help='Whether to use hard attention: action - talk|silent')
parser.add_argument('--comm_action_one', default=False, action='store_true',
                    help='Whether to always talk, sanity check for hard attention.')
parser.add_argument('--advantages_per_action', default=False, action='store_true',
                    help='Whether to multipy log porb for each chosen action with advantages')
parser.add_argument('--share_weights', default=False, action='store_true',
                    help='Share weights for hops')
# parser.add_argument('--test', default=False, type=bool,
#                     help='Train or Test')
parser.add_argument('--mode', default="Test", type=str,
                    help='Train or Test')   
parser.add_argument('--test-model', default="/predator.pt", type=str,
                    help='Model to test')    
parser.add_argument('--scenario', type=str, default='predator',
                    help='predator or planning ')
parser.add_argument('--airsim_vis', action='store_true', default=False,
                    help='Visualize in Airsim when testing')
parser.add_argument('--visualization', action='store_true', default=False,
                    help="enable commnet model")



# init_args_for_env(parser)
args = parser.parse_args()

# Data to be written
dictionary = {
    "name": args.scenario,
}

if args.ic3net:
    args.commnet = 1
    args.hard_attn = 1
    args.mean_ratio = 0

# Enemy comm
args.nfriendly = args.nagents
if hasattr(args, 'enemy_comm') and args.enemy_comm:
    if hasattr(args, 'nenemies'):
        args.nagents += args.nenemies
    else:
        raise RuntimeError("Env. needs to pass argument 'nenemy'.")

# visualization = True
is_centralized = False
N_frame = 5

if args.scenario == 'predator':
    from predator_prey import QuadrotorFormation
    env = QuadrotorFormation(n_agents=args.nagents, n_bots=args.nbots, visualization=False)
    if(args.airsim_vis == True):
        #Set Up JSON file for AirSim
        js_modifier = Json_Editor(2*args.nagents)
        js_modifier.modify()

elif args.scenario == 'planning':
    from planning import QuadrotorFormation
    env = QuadrotorFormation(n_agents=args.nagents, N_frame=N_frame,
                             visualization=False, is_centralized=is_centralized)
    if(args.airsim_vis == True):
        #Set Up JSON file for AirSim
        js_modifier = Json_Editor(args.nagents)
        js_modifier.modify()

else:
    print("Scenario is wrong. Please select: predator or planning")
num_inputs = 12
args.num_actions = env.n_action


# Multi-action
if not isinstance(args.num_actions, (list, tuple)):  # single action case
    args.num_actions = [args.num_actions]

args.dim_actions = 1
args.num_inputs = num_inputs

# Hard attention
if args.hard_attn and args.commnet:
    # add comm_action as last dim in actions
    args.num_actions = [*args.num_actions, 2]
    args.dim_actions = args.dim_actions + 1

# Recurrence
if args.commnet and (args.recurrent or args.rnn_type == 'LSTM'):
    args.recurrent = True
    args.rnn_type = 'LSTM'

parse_action_args(args)

if args.seed == -1:
    args.seed = np.random.randint(0, 10000)
torch.manual_seed(args.seed)

#print(args.naction_heads)

if args.commnet:
    print("Policy Net: CommNetMLP")
    policy_net = CommNetMLP(args, num_inputs)
elif args.random:
    policy_net = Random(args, num_inputs)
elif args.recurrent:
    policy_net = RNN(args, num_inputs)
    print("Policy Net: RNN")
else:
    policy_net = MLP(args, num_inputs)

if not args.display:
    display_models([policy_net])

# share parameters among threads, but not gradients
for p in policy_net.parameters():
    p.data.share_memory_()

if args.scenario == 'predator':
    trainer = Trainer(args, policy_net, env, None)
    disp_trainer = Trainer(args, policy_net, env, None)

disp_trainer.display = True

def disp():
    x = disp_trainer.get_episode()

log = dict()
log['epoch'] = LogField(list(), False, None, None)
log['reward'] = LogField(list(), True, 'epoch', 'num_episodes')
log['enemy_reward'] = LogField(list(), True, 'epoch', 'num_episodes')
log['success'] = LogField(list(), True, 'epoch', 'num_episodes')
log['steps_taken'] = LogField(list(), True, 'epoch', 'num_episodes')
log['add_rate'] = LogField(list(), True, 'epoch', 'num_episodes')
log['comm_action'] = LogField(list(), True, 'epoch', 'num_steps')
log['enemy_comm'] = LogField(list(), True, 'epoch', 'num_steps')
log['value_loss'] = LogField(list(), True, 'epoch', 'num_steps')
log['action_loss'] = LogField(list(), True, 'epoch', 'num_steps')
log['entropy'] = LogField(list(), True, 'epoch', 'num_steps')

# if args.plot:
#     vis = visdom.Visdom(env=args.plot_env)

def find_airsim_initial_poses(n_agents):
    x_initial=0
    y_initial=0
    positions=[]
    pose_x=x_initial
    pose_y=y_initial
    for i in range(n_agents):
        pose_x=x_initial-4*int(i/10)
        pose_y=y_initial+4*int(i-int(i/10)*10)
        positions.append([pose_x,pose_y,0])
    return positions

def stateToPosition(state):
    state_arr=state.split('{')[13].split(':')
    x=float(state_arr[1].split(',')[0])
    y=float(state_arr[2].split(',')[0])
    z=float(state_arr[3].split('}')[0])
    return [x,y,z]

def getAllPositions(client,n_agents):
    positions=[]
    for i in range(n_agents):
        state = client.getMultirotorState(vehicle_name=f"Drone{i+1}")
        s = pprint.pformat(state)
        positions.append(stateToPosition(s))
    return positions

def check_complated(targets,poses):
    for i in range(len(targets)):
        err=np.sqrt(pow(poses[i][0]-targets[i][0],2)+pow(poses[i][1]-targets[i][1],2)+pow(poses[i][2]-targets[i][2],2))
        if err>.5:
            return False
    return True


def run(num_epochs):

    for ep in range(args.epoch_size):
        takeoff = False

        _, agent_pos, bot_pos = trainer.test_batch(ep)

def save(ep, tm):

    timeFolderName = str("{}-{}-{}-{}-{}".format(tm.tm_mday, tm.tm_mon, tm.tm_year, tm.tm_hour, tm.tm_min))
    infoFolderName = str("{}-{}-{}".format(args.nagents, args.nbots, ep))
    current_dir =  os.path.abspath(os.path.dirname(__file__))

    if not os.path.exists(current_dir  + "/weight/" + args.scenario + "/" + timeFolderName):
        os.makedirs(current_dir  + "/weight/" + args.scenario + "/" + str(timeFolderName))
    file_path = current_dir  + "/weight/" + args.scenario + "/" + timeFolderName + "/" + infoFolderName + ".pt"
    
    print("file_path: ", file_path)
    d = dict()
    d['policy_net'] = policy_net.state_dict()
    d['log'] = log
    d['trainer'] = trainer.state_dict()
    torch.save(d, file_path)
    print("model saved")


def load(test_model):
    current_dir =  os.path.abspath(os.path.dirname(__file__))
    # file_path = current_dir + "/weight" + "/" + args.scenario + ".pt"
    file_path = current_dir + "/weight" + test_model
    
    d = torch.load(file_path)
    policy_net.load_state_dict(d['policy_net'])
    log.update(d['log'])
    trainer.load_state_dict(d['trainer'])
    print("Weights loaded")


def signal_handler(signal, frame):
    print('You pressed Ctrl+C! Exiting gracefully.')
    if args.display:
        env.end_display()
    sys.exit(0)


signal.signal(signal.SIGINT, signal_handler)

if args.load == "True" or args.mode == "Test":
    load(args.test_model)


run(args.num_epochs)
if args.display:
    env.end_display()


if sys.flags.interactive == 0 and args.nprocesses > 1:
    trainer.quit()
    import os
    os._exit(0)
