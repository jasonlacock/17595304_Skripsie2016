
# coding: utf-8

# In[ ]:

#!/usr/bin/env python


#import numpy as np
import logging
import sys, os
import random

logging.basicConfig(level=logging.INFO)

logger = logging.getLogger('openzwave')

import openzwave
from openzwave.node import ZWaveNode
from openzwave.value import ZWaveValue
from openzwave.scene import ZWaveScene
from openzwave.controller import ZWaveController
from openzwave.network import ZWaveNetwork
from openzwave.option import ZWaveOption
import time
from pydispatch import dispatcher

device="/dev/zwave"
log="None"
sniff=300.0

class packet(object):
    def __init__(self, ID):
        self.ID = ID
        self.time_start = 0
        self.time_end = 0
        self.B_start = 0
        self.A_start = 0
        self.A_time = 0
        self.C_start = 0
        self.C_time = 0
        self.B_time = 0
        self.sent = 0

class wall_clock (object):
    def __init__(self):
        self.starter = 0
        self.total_time = 0
        self.RTT = 0
        self.quickest = 1000
        self.slowest = 0
        self.quickest_A = 1000
        self.slowest_A = 0
        self.quickest_B = 1000
        self.slowest_B = 0
        self.quickest_C = 1000
        self.slowest_C = 0
        self.packet_count = 0
        self.RTT_avg = 0
        self.A_total = 0
        self.A_avg = 0
        self.B_total = 0
        self.B_avg = 0
        self.C_total = 0
        self.C_avg = 0
        self.SOF_start = 0
        self.Write_start = 0
        self.ACK_start = 0
        self.Retries_start = 0
        self.Dropped_start = 0
        self.ACK_waiting_start = 0
        self.CAN_start = 0
        self.active_sent = 0
        self.active_received = 0
        self.active_time = 0
        self.active = []
        self.packets = []
        self.TXs = 0
        self.RXs = 0

    def send_packet(self):
        self.active.append(1)
        new_packet = packet(1)
        self.packets.append(new_packet)
        index = self.active.index(1)
        msg = self.packets[index]
        msg.A_start = time.time()

        #Request neighbour node update
        neighbour_update = network.controller.request_node_neighbor_update(33)
        print()
        if(neighbour_update == True):
            print("Requesting neighbour update")

    def check_packet(self):
        count = len(self.active)
        if(count>1):
            print(count)
            print(self.active)
        i=0
        while(i < count):
            msg = self.packets[i]
            duration = 0
            if(msg.sent == 0):
                duration = time.time() - msg.packet_start
            else:
                duration = time.time() - msg.ack_start
            if(duration >= 0.5):
                self.active.remove(self.active[i])
                del self.packets[i]
                jason.total_time = jason.total_time
                print("fail")
                print(self.active)
                self.total_time = self.total_time + 0.5
                count=count-1
            else:
                i=i+1

    def calc_RTT(self, state):
        data=1
        index = self.active.index(data)
        msg = self.packets[index]
        if(state == "InProgress"):
            msg.sent = 1
            self.TXs = self.TXs + 1
            print("Packet Sent, waiting for ack!")
            msg.C_start = time.time()
            duration = time.time() - msg.B_start
            msg.B_time = duration
            if(msg.B_time < self.quickest_B):
                self.quickest_B = msg.B_time
            if(msg.B_time > self.slowest_B):
                self.slowest_B = msg.B_time
            self.B_total = self.B_total + msg.B_time
        elif(state == "Completed"):
            print("ACK Received!")
            self.RXs = self.RXs + 1
            duration = time.time() - msg.C_start
            msg.C_time = duration
            if(msg.C_time < self.quickest_C):
                self.quickest_C = msg.C_time
            if(msg.C_time > self.slowest_C):
                self.slowest_C = msg.C_time
            RTT = msg.A_time + msg.B_time + msg.C_time
            #self.RTT = self.RTT + RTT
            self.active.remove(data)
            del self.packets[index]
            self.packet_count = self.packet_count + 1
            if(RTT < self.quickest):
                self.quickest = RTT
            if(RTT > self.slowest):
                self.slowest = RTT
            print()
            print("Phase A Transmission Time: {} ms".format(round((1000*msg.A_time),3)))
            print("Phase B Transmission Time: {} ms".format(round((1000*msg.B_time),3)))
            print("Phase C Transmission Time: {} ms".format(round((1000*msg.C_time),3)))
            print("Round Trip Time: {} ms".format(round((1000*RTT),3)))
            print()
            #if(len(self.active) == 0):
            self.total_time = self.total_time + RTT
            #self.RTT = 0
            self.RTT_avg = self.total_time / self.packet_count
            self.C_total = self.C_total + msg.C_time
            self.A_avg = self.A_total / self.packet_count
            self.B_avg = self.B_total / self.packet_count
            self.C_avg = self.C_total / self.packet_count
            msg.sent = 0
        elif(state=="Starting"):
            print("Beginning packet transmission")
            msg.sent = 1
            msg.B_start = time.time()
            duration = time.time() - msg.A_start
            msg.A_time = duration
            if(msg.A_time < self.quickest_A):
                self.quickest_A = msg.A_time
            if(msg.A_time > self.slowest_A):
                self.slowest_A = msg.A_time
            self.A_total = self.A_total + msg.A_time


for arg in sys.argv:
    if arg.startswith("--device"):
        temp,device = arg.split("=")
    elif arg.startswith("--log"):
        temp,log = arg.split("=")
    elif arg.startswith("--sniff"):
        temp,sniff = arg.split("=")
        sniff = float(sniff)
    elif arg.startswith("--help"):
        print("help : ")
        print("  --device=/dev/yourdevice ")
        print("  --log=Info|Debug")

        #Define some manager options
options = ZWaveOption(device,   config_path="/srv/hass/src/python-openzwave/openzwave/config",   user_path=".", cmd_line="")
options.set_log_file("OZW_Log.log")
options.set_append_log_file(False)
options.set_console_output(False)
options.set_save_log_level(log)
options.set_logging(True)
options.lock()

def louie_network_started(network):
    print("Hello from network : I'm started : homeid {:08x} - {} nodes were found.".format(network.home_id, network.nodes_count))

def louie_network_failed(network):
    print("Hello from network : can't load :(.")

def louie_network_ready(network):
    print("Hello from network : I'm ready : {} nodes were found.".format(network.nodes_count))
    print("Hello from network : my controller is : {}".format(network.controller))
    dispatcher.connect(louie_node_update, ZWaveNetwork.SIGNAL_NODE)
    dispatcher.connect(louie_value_update, ZWaveNetwork.SIGNAL_CONTROLLER_COMMAND)

def louie_node_update(network, node):
    print("Hello from node : {}.".format(node))
def louie_value_update(network, controller, node, node_id,  state_int, state, state_full, error_int, error, error_full):
#    jason.check_packet()
    if((jason.starter == 1) and (1 in jason.active)):
        jason.calc_RTT(state)

#Create a network object
network = ZWaveNetwork(options, autostart=False)

jason = wall_clock()

#We connect to the louie dispatcher
dispatcher.connect(louie_network_started, ZWaveNetwork.SIGNAL_NETWORK_STARTED)
dispatcher.connect(louie_network_failed, ZWaveNetwork.SIGNAL_NETWORK_FAILED)
dispatcher.connect(louie_network_ready, ZWaveNetwork.SIGNAL_NETWORK_READY)

network.start()

print("------------------------------------------------------------")
print("Use openzwave library : {}".format(network.controller.ozw_library_version))
print("Use python library : {}".format(network.controller.python_library_version))
print("Use ZWave library : {}".format(network.controller.library_description))
print("Network home id : {}".format(network.home_id_str))
print("Nodes in network : {}".format(network.nodes_count))
print("------------------------------------------------------------")
print("Waiting for network to become ready : ")
print("------------------------------------------------------------")


#We wait for the network.
for i in range(0,90):
    if network.state>=network.STATE_READY:
        print("***** Network is ready")
        break
    else:
        sys.stdout.write(".")
        sys.stdout.flush()
        time.sleep(1.0)

time.sleep(2.0)
stats = network.controller.stats
#Toggle the lightbulb on and off

jason.SOF_start = stats['SOFCnt']
jason.Write_start = stats['writeCnt']
jason.ACK_start = stats['ACKCnt']
jason.Retries_start = stats['retries']
jason.Dropped_start = stats['dropped']
jason.ACK_waiting_start = stats['ACKWaiting']
jason.CAN_start = stats['CANCnt']


for node in network.nodes:
    nodeID = network.nodes[node].node_id
    role = network.nodes[node].role
    print("Node: {}".format(nodeID))
    print("Role: {}".format(role))

jason.starter = 1
i=0
for i in range (0,100):
    print("level {}".format(i+1))
    jason.send_packet()
    time.sleep(0.5)

time.sleep(1)

print("------------------------------------------------------------")
print("Controller capabilities : {}".format(network.controller.capabilities))
print("Controller node capabilities : {}".format(network.controller.node.capabilities))
print("Nodes in network : {}".format(network.nodes_count))
stats = network.controller.stats
print("Driver statistics : {}".format(stats))
print("------------------------------------------------------------")
bits_sent = ((2 * jason.TXs * 4)+(4 * jason.RXs)) * 8
throughput = bits_sent / jason.total_time
print("Data Transmitted: {} bits".format(bits_sent))
SOF_count = stats['SOFCnt'] - jason.SOF_start
Write_count = stats['writeCnt'] - jason.Write_start
ACK_count = stats['ACKCnt'] - jason.ACK_start
Dropped_count = stats['dropped'] - jason.Dropped_start
Retries_count = stats['retries'] - jason.Retries_start
ACK_waiting_count = stats['ACKWaiting'] - jason.ACK_waiting_start
CAN_count = stats['CANCnt'] - jason.CAN_start
print()
print("SOF: {}".format(SOF_count))
print("Writes: {}".format(Write_count))
print("ACKs: {}".format(ACK_count))
print("Dropped: {}".format(Dropped_count))
print("Retries: {}".format(Retries_count))
print("ACK Waiting: {}".format(ACK_waiting_count))
print("CANs: {}".format(CAN_count))
print()
print("Total Trip Time: {} s".format(round((jason.total_time), 3)))
print("Throughput: {} bits/s".format(round(throughput,3)))
print("Average Round Trip Time: {} ms".format(round((1000*jason.RTT_avg), 3)))
print("Average Round Trip Time: {} ms".format(round((1000*jason.RTT_avg), 3)))
print("Fastest Round Trip Time: {} ms".format(round((1000*jason.quickest), 3)))
print("Slowest Round Trip Time: {} ms".format(round((1000*jason.slowest), 3)))
print()
print("Average Phase A Transmission Time: {} ms".format(round((1000*jason.A_avg), 3)))
print("Fastest Phase A Transmission Time: {} ms".format(round((1000*jason.quickest_A), 3)))
print("Slowest Phase A Transmission Time: {} ms".format(round((1000*jason.slowest_A), 3)))
print()
print("Average Phase B Transmission Time: {} ms".format(round((1000*jason.B_avg), 3)))
print("Fastest Phase B Transmission Time: {} ms".format(round((1000*jason.quickest_B), 3)))
print("Slowest Phase B Transmission Time: {} ms".format(round((1000*jason.slowest_B), 3)))
print()
print("Average Phase C Transmission Time: {} ms".format(round((1000*jason.C_avg), 3)))
print("Fastest Phase C Transmission Time: {} ms".format(round((1000*jason.quickest_C), 3)))
print("Slowest Phase C Transmission Time: {} ms".format(round((1000*jason.slowest_C), 3)))
print()

print("Sent: {}".format(jason.TXs))

print("Received: {}".format(jason.RXs))
print()

time.sleep(2.0)

network.stop()

