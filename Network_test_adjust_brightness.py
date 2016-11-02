
# coding: utf-8

# In[ ]:


#!/usr/bin/env python
# -*- coding: utf-8 -*-

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
        self.ack_start = 0
        self.packet_start = 0
        self.packet_time = 0
        self.ack_time = 0
        self.sent = 0

class wall_clock (object):
    def __init__(self):
        self.starter = 0
        self.total_time = 0
        self.RTT = 0
        self.quickest = 1000
        self.slowest = 0
        self.quickest_packet = 1000
        self.slowest_packet = 0
        self.quickest_ack = 1000
        self.slowest_ack = 0
        self.packet_count = 0
        self.RTT_avg = 0
        self.packet_total = 0
        self.packet_avg = 0
        self.ack_total = 0
        self.ack_avg = 0
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
        self.fails = 0

    def send_packet(self, node, val, data):
#        neighbour = network.controller.request_node_neighbor_update(32)
 #       if(neighbour == True):
#          print("Requesting node neighbour update")
        self.active.append(data)
        new_packet = packet(data)
        self.packets.append(new_packet)
        index = self.active.index(data)
        msg = self.packets[index]
        msg.packet_start = time.time()
        network.nodes[node].set_dimmer(val,data)

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
                jason.total_time = jason.total_time + 0.5
                print("fail")
                self.fails = self.fails + 1
#               if(self.fails >= 10):
#                    starter=0    
                print(self.active)
                count=count-1
            else:
                i=i+1

    def calc_RTT(self, data):
        index = self.active.index(data)
        msg = self.packets[index]

        if(msg.sent == 0):
            msg.sent = 1
            self.TXs = self.TXs + 1
            print("Checkpoint 1! Value: {}".format(data))
            msg.ack_start = time.time()
            duration = time.time() - msg.packet_start
            msg.packet_time = duration
            if(msg.packet_time < self.quickest_packet):
                self.quickest_packet = msg.packet_time
            if(msg.packet_time > self.slowest_packet):
                self.slowest_packet = msg.packet_time
            self.packet_total = self.packet_total + msg.packet_time
        else:
            print("Checkpoint 2! Value: {}".format(data))
            level = network.nodes[node].get_dimmer_level(val)
            print("Brightness: {}".format(level))
            self.RXs = self.RXs + 1
            duration = time.time() - msg.ack_start
            msg.ack_time = duration
            if(msg.ack_time < self.quickest_ack):
                self.quickest_ack = msg.ack_time
            if(msg.ack_time > self.slowest_ack):
                self.slowest_ack = msg.ack_time
            RTT = msg.packet_time + msg.ack_time
            self.RTT = self.RTT + RTT
            self.active.remove(data)
            del self.packets[index]
            self.packet_count = self.packet_count + 1
            if(RTT < self.quickest):
                self.quickest = self.RTT
            if(RTT > self.slowest):
                self.slowest = self.RTT
            print()
            print("Checkpoint 1 Transmission Time: {} ms".format(round((1000*msg.packet_time),3)))
            print("Checkpoint 2 Transmission Time: {} ms".format(round((1000*msg.ack_time),3)))
            print("Round Trip Time: {} ms".format(round((1000*RTT),3)))
            print()
            print()
            if(len(self.active) == 0):
                self.total_time = self.total_time + self.RTT
                self.RTT = 0
                self.RTT_avg = self.total_time / self.packet_count
            self.ack_total = self.ack_total + msg.ack_time
            self.packet_avg = self.packet_total / self.packet_count
            self.ack_avg = self.ack_total / self.packet_count


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
    dispatcher.connect(louie_value_update, ZWaveNetwork.SIGNAL_VALUE)



def louie_node_update(network, node):
    print("Hello from node : {}.".format(node))

def louie_value_update(network, node, value):
    jason.check_packet()
    if((jason.starter == 1) and (value.data in jason.active) and (jason.fails < 5)):
        jason.calc_RTT(value.data)


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
for node in network.nodes:
    jason.SOF_start = stats['SOFCnt']
    jason.Write_start = stats['writeCnt']
    jason.ACK_start = stats['ACKCnt']
    jason.Retries_start = stats['retries']
    jason.Dropped_start = stats['dropped']
    jason.ACK_waiting_start = stats['ACKWaiting']
    jason.CAN_start = stats['CANCnt']

    nodeID = network.nodes[node].node_id
    if(nodeID != 1):
        role = network.nodes[node].role
        print("Node: {}".format(nodeID))
        time.sleep(1)
        jason.starter = 1
        random.seed()
        for val in network.nodes[node].get_dimmers():
            time.sleep(2)
            i=0
            for i in range (0,100):
                flag=False
                brightness = random.randint(1,99)
                while(brightness==False):
                    if(brightness not in jason.active):
                        flag=True
                    else:
                        brightness = random.randint(1,99)
                print("level {}".format(i+1))
                jason.send_packet(node, val, brightness)
                jason.check_packet()
                time.sleep(0.5)

time.sleep(2.0)

print("------------------------------------------------------------")
print("Controller capabilities : {}".format(network.controller.capabilities))
print("Controller node capabilities : {}".format(network.controller.node.capabilities))
print("Nodes in network : {}".format(network.nodes_count))
stats = network.controller.stats
print("Driver statistics : {}".format(stats))
print("------------------------------------------------------------")
bits_sent = ((jason.TXs * 4) + (2 * jason.TXs * 2)+(2 * jason.RXs)) * 8
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
print("Fastest Round Trip Time: {} ms".format(round((1000*jason.quickest), 3)))
print("Slowest Round Trip Time: {} ms".format(round((1000*jason.slowest), 3)))
print()
print("Average Checkpoint 1 Transmission Time: {} ms".format(round((1000*jason.packet_avg), 3)))
print("Fastest Checkpoint 1 Transmission Time: {} ms".format(round((1000*jason.quickest_packet), 3)))
print("Slowest Checkpoint 1 Transmission Time: {} ms".format(round((1000*jason.slowest_packet), 3)))
print()
print("Average Checkpoint 2 Transmission Time: {} ms".format(round((1000*jason.ack_avg), 3)))
print("Fastest Checkpoint 2 Transmission Time: {} ms".format(round((1000*jason.quickest_ack), 3)))
print("Slowest Checkpoint 2 Transmission Time: {} ms".format(round((1000*jason.slowest_ack), 3)))
print()
print("Sent: {}".format(jason.TXs))
print("Received: {}".format(jason.RXs))
print()
time.sleep(2.0)

network.stop()

