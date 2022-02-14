# Based on ReachView code from Egor Fedorov (egor.fedorov@emlid.com)
# Updated for Python 3.6.8 on a Raspberry  Pi


""" import time
import pexpect
import subprocess
import sys
import logging """
import numpy as np
import random
import matplotlib.pyplot as plt
import csv


"""

logger = logging.getLogger("btctl")


class Bluetoothctl:
     """
"""A wrapper for bluetoothctl utility. """
"""

    def __init__(self):
        subprocess.check_output("rfkill unblock bluetooth", shell=True)
        self.process = pexpect.spawnu("bluetoothctl", echo=False)

    def send(self, command, pause=0):
        self.process.send(f"{command}\n")
        time.sleep(pause)
        if self.process.expect(["bluetooth", pexpect.EOF]):
            raise Exception(f"failed after {command}")

    def get_output(self, *args, **kwargs):
         """
"""Run a command in bluetoothctl prompt, return output as a list of lines. """
"""
        self.send(*args, **kwargs)
        return self.process.before.split("\r\n")

    def start_scan(self):
         """
"""Start bluetooth scanning process. """
"""
        try:
            self.send("scan on")
        except Exception as e:
            logger.error(e)

    def make_discoverable(self):
         """
"""Make device discoverable. """
"""
        try:
            self.send("discoverable on")
        except Exception as e:
            logger.error(e)

    def parse_device_info(self, info_string):
         """
"""Parse a string corresponding to a device. """
"""
        device = {}
        block_list = ["[\x1b[0;", "removed"]
        if not any(keyword in info_string for keyword in block_list):
            try:
                device_position = info_string.index("Device")
            except ValueError:
                pass
            else:
                if device_position > -1:
                    attribute_list = info_string[device_position:].split(" ", 2)
                    device = {
                        "mac_address": attribute_list[1],
                        "name": attribute_list[2],
                    }
        return device

    def get_available_devices(self):
         """
"""Return a list of tuples of paired and discoverable devices. """
"""
        available_devices = []
        try:
            out = self.get_output("devices")
        except Exception as e:
            logger.error(e)
        else:
            for line in out:
                device = self.parse_device_info(line)
                if device:
                    available_devices.append(device)
        return available_devices

    def get_paired_devices(self):
         """
"""Return a list of tuples of paired devices. """
"""
        paired_devices = []
        try:
            out = self.get_output("paired-devices")
        except Exception as e:
            logger.error(e)
        else:
            for line in out:
                device = self.parse_device_info(line)
                if device:
                    paired_devices.append(device)
        return paired_devices

    def get_discoverable_devices(self):
         """
"""Filter paired devices out of available. """
"""
        available = self.get_available_devices()
        paired = self.get_paired_devices()
        return [d for d in available if d not in paired]

    def get_device_info(self, mac_address):
         """
"""Get device info by mac address. """
"""
        try:
            out = self.get_output(f"info {mac_address}")
        except Exception as e:
            logger.error(e)
            return False
        else:
            return out

    def pair(self, mac_address):
         """
"""Try to pair with a device by mac address. """
"""
        try:
            self.send(f"pair {mac_address}", 4)
        except Exception as e:
            logger.error(e)
            return False
        else:
            res = self.process.expect(
                ["Failed to pair", "Pairing successful", pexpect.EOF]
            )
            return res == 1

    def trust(self, mac_address):
        try:
            self.send(f"trust {mac_address}", 4)
        except Exception as e:
            logger.error(e)
            return False
        else:
            res = self.process.expect(
                ["Failed to trust", "Pairing successful", pexpect.EOF]
            )
            return res == 1

    def remove(self, mac_address):
         """
"""Remove paired device by mac address, return success of the operation. """
"""
        try:
            self.send(f"remove {mac_address}", 3)
        except Exception as e:
            logger.error(e)
            return False
        else:
            res = self.process.expect(
                ["not available", "Device has been removed", pexpect.EOF]
            )
            return res == 1

    def connect(self, mac_address):
         """
"""Try to connect to a device by mac address. """
"""
        try:
            self.send(f"connect {mac_address}", 2)
        except Exception as e:
            logger.error(e)
            return False
        else:
            res = self.process.expect(
                ["Failed to connect", "Connection successful", pexpect.EOF]
            )
            return res == 1

    def disconnect(self, mac_address):
         """
"""Try to disconnect to a device by mac address. """
"""
        try:
            self.send(f"disconnect {mac_address}", 2)
        except Exception as e:
            logger.error(e)
            return False
        else:
            res = self.process.expect(
                ["Failed to disconnect", "Successful disconnected", pexpect.EOF]
            )
            return res == 1

    def advertise(self):
        try:
            self.send(f"menu advertise")
            self.send(f"clear")
            self.send(f"uuids 0x1800 0x1801 0x1802")
            self.send(f"name MyDevice")
            self.send(f"manufacturer 0x0059 0x34 0x58 0x1f 0x41")
            self.send(f"back")
            self.send(f"advertise on")
            time.sleep(20)

        except Exception as e:
            logger.error(e)
            return False
        else:
            res = self.process.expect(
                ["Failed to disconnect", "Successful disconnected", pexpect.EOF]
            )
            return res == 1

 """

# Function to reverse bits of a given integer
def reverseBits(n):

    pos = 8 - 1      # maintains shift

    # store reversed bits of `n`. Initially, all bits are set to 0
    reverse = 0

    # do till all bits are processed
    while pos >= 0 and n:

        # if the current bit is 1, then set the corresponding bit in the result
        if n & 1:
            reverse = reverse | (1 << pos)

        n >>= 1         # drop current bit (divide by 2)
        pos = pos - 1   # decrement shift by 1

    return reverse

def mam(a,b):
    return (17*a+b)%(2**16)


#channel selection algorithm 2
def CSA2(master_channel_map,access_address,length_of_hops):

    master_channel_map_integer = int(master_channel_map,16)
    master_channel_map = hex(master_channel_map_integer)

    access_address1 = access_address[0:6]
    access_address2 = "0x"
    access_address2 = access_address2 + access_address[6:]

    access_address1_integer = int(access_address1, 16)
    access_address2_integer = int(access_address2, 16)

    master_hop = []


    channel_identifier = hex(access_address1_integer ^ access_address2_integer)

    for counter in range(1, length_of_hops+1):

        output = hex(int(channel_identifier,16) ^ counter)


        #permution
        output1 = output[0:4]

        output2 = "0x"
        output2 = output2 + output[4:]

        output1 = hex(reverseBits(int(output1,16)))

        if(output2!="0x"):
            output2 = hex(reverseBits(int(output2,16)))


        output = output1+output2[2:]

        #mam
        output = hex(mam(int(output,16),int(channel_identifier,16)))


        #permution
        output1 = output[0:4]

        output2 = "0x"
        output2 = output2 + output[4:]

        output1 = hex(reverseBits(int(output1,16)))

        if(output2!="0x"):
            output2 = hex(reverseBits(int(output2,16)))


        output = output1+output2[2:]


        #mam
        output = hex(mam(int(output,16),int(channel_identifier,16)))

        #permution
        output1 = output[0:4]

        output2 = "0x"
        output2 = output2 + output[4:]

        output1 = hex(reverseBits(int(output1,16)))

        if(output2!="0x"):
            output2 = hex(reverseBits(int(output2,16)))


        output = output1+output2[2:]


        #mam
        output = hex(mam(int(output,16),int(channel_identifier,16)))

        prn_e = hex(int(output,16) ^ int(channel_identifier,16))


        unmappedChannel = int(prn_e,16) % 37

        reverse_ch_map = bin(int(master_channel_map,16))[2:].zfill(40)[::-1]
        if reverse_ch_map[unmappedChannel] == "1" :
            master_hop.append(unmappedChannel)
        else:
            n = 0

            for i in reverse_ch_map[0:37]:
                if i == "1":
                    n+=1

            if(n==0):
                #print(reverse_ch_map)
                print('used channel finished')

            remapping_index = int((n*int(prn_e,16)) / 2**16)

            used_channels = []
            for i in range(0,37):
                if reverse_ch_map[i] == "1":
                    used_channels.append(i)
            master_hop.append(used_channels[remapping_index])
            del used_channels[:]

    return master_hop

def system_collision_probability():
    piconet_collision = [0] * number_of_piconets
    for i in range(0, number_of_piconets):
        for j in range(0,length_of_hops):
            for k in range(0,number_of_piconets):
                if(i==k):
                    continue
                elif( hop_results[i][j] == hop_results[k][j]):
                    piconet_collision[i]+=1
                    break

    return sum(piconet_collision)/(number_of_piconets*length_of_hops)

def channel_collision_probability(channel):

    channel_collision = [0] * number_of_piconets

    for i in range(0, number_of_piconets):
        for j in range(0,length_of_hops):
            for k in range(0,number_of_piconets):
                if(i==k):
                    continue
                elif(hop_results[i][j] == hop_results[k][j] and channel == hop_results[k][j]):
                    channel_collision[i]+=1
                    break
    return sum(channel_collision)/(number_of_piconets*length_of_hops)

def channel_collision_probability_piconet(piconet,channel):
    channel_collision = 0
    for j in range(0,length_of_hops):
        for k in range(0,number_of_piconets):
            if(piconet==k):
                continue
            elif(hop_results[piconet][j] == hop_results[k][j] and channel == hop_results[k][j]):
                channel_collision+=1
                break
    return channel_collision/(number_of_piconets*length_of_hops)

def test():
    for i in range(0,number_of_piconets):
        binary = bin(int(master_maps[i],16))[2:].zfill(40)
        reverse_channel_map = binary[::-1]
        for channel,j in enumerate(reverse_channel_map[0:37]):
           if j == "1":
               found = 0
               for k in range(0,length_of_hops):
                   if hop_results[i][k] == channel:
                       found = 1
               print(found)

def bandwidth_indicator():

    number_of_unused_channel = [0] * number_of_piconets
    makhraj = [0] * number_of_piconets
    bandwidth = [0] * number_of_piconets
    for x in range(0, number_of_piconets):
        #convert to binary
        binary = bin(int(master_maps[x],16))[2:].zfill(40)
        reverse_channel_map = binary[::-1]
        number_of_unused_channel = 0
        found = [0] * 37
        for index,j in enumerate(reverse_channel_map[0:37]):
            for z in range(0,length_of_hops):
                if hop_results[x][z] == index:
                    found[index] = 1
            if j == "1" and found[index] == 1:
                number_of_unused_channel += 1

        for channel,j in enumerate(reverse_channel_map[0:37]):
           if j == "1" and found[channel] == 1:

               channel_count_on_hops = [0] * 37
               channel_count_on_time_slots = [0] * length_of_hops
               for piconet in range(0,number_of_piconets):
                   for k in range(0,length_of_hops):
                       if hop_results[piconet][k] == channel:
                           channel_count_on_hops[channel]+=1
                           channel_count_on_time_slots[k] = 1
               makhraj[x] += (channel_count_on_hops[channel]/sum(channel_count_on_time_slots))

        bandwidth[x] = number_of_unused_channel/makhraj[x]
    avg_bandwidth = sum(bandwidth)/len(bandwidth)
    return avg_bandwidth


""" print("Init bluetooth...")
bl = Bluetoothctl()
print("Ready!")
bl.start_scan()
print("Scanning for 10 seconds...") """

#for i in range(0, 10):
#    print(i)
#    time.sleep(1)

""" str1 = bl.get_device_info("F2:87:B0:C7:B9:A9")
my_string = "\tManufacturerData Value:"

matched_indexes = []
i = 0
length = len(str1)

while i < length:
    if my_string == str1[i]:
        matched_indexes.append(i)
        break
    i += 1

print(str1) """
#channel_map = str1[i+1]
#print(channel_map)
#print("Channel Map:", channel_map[5:18])



def generate_channel_maps(m,number_of_piconets):
    #generate first channel map
    values = np.random.binomial(1, 1-m,40)
    for i in range(0, 40):
        temp_map.append(values[i])

    channel_maps.insert(0, temp_map)

    base_ch_map = ''.join([str(elem) for elem in channel_maps[0]])
    master_maps.append(hex(int(base_ch_map, 2)))

    #generate other channel maps based on first channel map
    #toggle one bit randomly for each master
    for j in range (1,number_of_piconets):
        position = random.randint(3, 39)
        if base_ch_map[position] == '1':
            new_character = '0'
        elif base_ch_map[position] == '0':
            new_character = '1'
        this_ch_map = base_ch_map[:position] + new_character + base_ch_map[position+1:]
        master_maps.append(hex(int(this_ch_map, 2)))

    return master_maps


def generate_access_address(number_of_piconets):

    for i in range(0,number_of_piconets):
        this_master_access = 0
        del temp_access[:]
        values = np.random.binomial(1,0.5,32)
        for j in range(0, 32):
            temp_access.append(values[j])

        access_address_maps.insert(i, temp_access)

        this_master_access = ''.join([str(elem) for elem in access_address_maps[0]])
        master_access.append(hex(int(this_master_access, 2)))

    return master_access


# Function to calculate sum of each row
def row_sum(arr,i) :

    sum = 0

    # finding the row sum
    for i in arr[i] :
        # Add the element
        sum += i

    return sum

# Function to calculate sum of each column
def column_sum(arr,j) :

    sum = 0

    # finding the column sum
    for row in range(len(arr)):
        sum += arr[row][j]

    return sum

output_ble_default_collision = []
output_remove_bad_channels_collision = []
output_experimental_collision = []

avg_num_channel_default = []
avg_num_channel_sim = []
avg_num_channel_ex = []

avg_predict_bw_default = []
avg_predict_bw_sim = []
avg_predict_bw_ex = []

avg_bw_default = []
avg_bw_sim = []
avg_bw_ex = []

m = 0.7
from_piconet = 5
to_piconet = 25
iteration = 20
output_number_of_piconets = list(range(from_piconet,to_piconet+1))

for p in range(from_piconet,to_piconet+1):
    temp_output_ble_default_collision = []
    temp_output_remove_bad_channels_collision = []
    temp_output_experimental_collision = []

    temp_avg_num_channel_default = []
    temp_avg_num_channel_sim = []
    temp_avg_num_channel_ex = []

    temp_avg_bw_default = []
    temp_avg_bw_sim = []
    temp_avg_bw_ex = []

    temp_avg_predict_bw_default = []
    temp_avg_predict_bw_sim = []
    temp_avg_predict_bw_ex = []

    for v in range(0,iteration):
        # Start probability algorithm
        number_of_piconets = ploc
        length_of_hops = 100
        hop_results = []
        channel_maps = []
        master_maps = []
        temp_map = []
        master_access = []
        temp_access = []
        access_address_maps = []
        #primitive interference
        threshold = 1
        system_collision = 1
        first_visit = 0
        count_num_channel_default = 0
        count_num_channel_sim = 0
        count_num_channel_ex = 0

        ex_master_maps = generate_channel_maps(m,number_of_piconets)
        ex_access_address = generate_access_address(number_of_piconets)


        #calculate average number of used channels in piconet before simulation
        for i in range(0,number_of_piconets):
            #convert to binary
            binary = bin(int(master_maps[i],16))[2:].zfill(40)
            for j in range(3,40):
                if binary[j] == '1':
                    count_num_channel_default+=1

        temp_avg_num_channel_default.append(count_num_channel_default/number_of_piconets)

        for i in range(0, number_of_piconets):
            hop_results.append(CSA2(master_maps[i],master_access[i],length_of_hops))

        #calculate bandwidth prediction for ble default for this piconet before simulation
        predict_bandwidth_matrix = [ [0]*number_of_piconets for i in range(0,37)]

        for i in range(0,37):
            for j in range(0,number_of_piconets):
                #convert to binary
                binary = bin(int(master_maps[j],16))[2:].zfill(40)
                reverse_channel_map = binary[::-1]
                if reverse_channel_map[i] == "1":
                    predict_bandwidth_matrix[i][j] = 1

        soorat = [0] * number_of_piconets
        makhraj = [0] * 37
        for i in range(0,37):
            for j in range(0,number_of_piconets):

                #convert to binary
                binary = bin(int(master_maps[j],16))[2:].zfill(40)
                reverse_channel_map = binary[::-1]

                if reverse_channel_map[i] == "1":
                   soorat[j]+=1
                   makhraj[i] = row_sum(predict_bandwidth_matrix,i)

        sum_makhraj = [0] * number_of_piconets
        for i in range(0,37):
            for j in range(0,number_of_piconets):

                #convert to binary
                binary = bin(int(master_maps[j],16))[2:].zfill(40)
                reverse_channel_map = binary[::-1]

                if reverse_channel_map[i] == "1":
                   sum_makhraj[j] += makhraj[i]

        bandwidth_predict = [m / n for m, n in zip(soorat, sum_makhraj)]
        temp_avg_predict_bw_default.append(sum(bandwidth_predict)/len(bandwidth_predict))

        #calculate bandwidth indicator for ble default for this piconet before simulation
        temp_avg_bw_default.append(bandwidth_indicator())

        while (system_collision >= threshold):
            del hop_results[:]

            for i in range(0, number_of_piconets):
                hop_results.append(CSA2(master_maps[i],master_access[i],length_of_hops))

            system_collision = system_collision_probability()
            all_channel_collisions = [0] * 37
            for i in range(0, 37):
                all_channel_collisions[i] = channel_collision_probability(i)

            #print(system_collision)

            if first_visit==0:
                #set threshold -5%
                threshold = system_collision - 0.05
                temp_output_ble_default_collision.append(system_collision)
                first_visit = 1


            bad_channel_index = all_channel_collisions.index(max(all_channel_collisions))
            channel_collision_probability_piconet_list = [] * number_of_piconets
            for i in range(0,number_of_piconets):
                channel_collision_probability_piconet_list.append(channel_collision_probability_piconet(i,bad_channel_index))


            #calculate number of unused channels for each piconet
            number_of_unused_channel = [] * number_of_piconets
            for i in range(0, number_of_piconets):
                 unused = 0
                 binary_master_map = bin(int(master_maps[i],16))[2:].zfill(40)
                 reverse_binary_master_map = binary_master_map[::-1]
                 for j in reverse_binary_master_map[0:37]:
                    if j == "0":
                        unused+=1
                 number_of_unused_channel.append(unused)

                 if(unused==0):
                    print('used channel finished')

            #Consider Number of Unused Channels in Piconet Selection
            #print("before",channel_collision_probability_piconet_list)
            for i in range(0,len(channel_collision_probability_piconet_list)):
                channel_collision_probability_piconet_list[i] = channel_collision_probability_piconet_list[i]/(2**(number_of_unused_channel[i]))
            #print("after",channel_collision_probability_piconet_list)

            bad_piconet_index = channel_collision_probability_piconet_list.index(max(channel_collision_probability_piconet_list))

            binary_master_map = bin(int(master_maps[bad_piconet_index],16))[2:].zfill(40)
            reverse_binary_master_map = binary_master_map[::-1]

            #print(master_maps)

            #print('bad_channel_index',bad_channel_index)

            #print('bad_piconet_index',bad_piconet_index)

            if reverse_binary_master_map[bad_channel_index] == '1':
                #remove bad channel from map
                reverse_binary_master_map = reverse_binary_master_map[:bad_channel_index] + '0' + reverse_binary_master_map[bad_channel_index+1:]

            double_reverse_binary_master_map = reverse_binary_master_map[::-1]

            master_maps[bad_piconet_index] = hex(int(double_reverse_binary_master_map, 2))

        #calculate average number of used channels in piconet after simulation
        for i in range(0,number_of_piconets):
            #convert to binary
            binary = bin(int(master_maps[i],16))[2:].zfill(40)
            for j in range(3,40):
                if(binary[j] == '1'):
                    count_num_channel_sim+=1

        temp_avg_num_channel_sim.append(count_num_channel_sim/number_of_piconets)

        system_collision = system_collision_probability()
        temp_output_remove_bad_channels_collision.append(system_collision)
        #print('Finished Simulation Algorithm')


        #calculate bandwidth prediction after simulation
        predict_bandwidth_matrix = [ [0]*number_of_piconets for i in range(0,37)]

        for i in range(0,37):
            for j in range(0,number_of_piconets):
                #convert to binary
                binary = bin(int(master_maps[j],16))[2:].zfill(40)
                reverse_channel_map = binary[::-1]
                if reverse_channel_map[i] == "1":
                    predict_bandwidth_matrix[i][j] = 1

        soorat = [0] * number_of_piconets
        makhraj = [0] * 37
        for i in range(0,37):
            for j in range(0,number_of_piconets):

                #convert to binary
                binary = bin(int(master_maps[j],16))[2:].zfill(40)
                reverse_channel_map = binary[::-1]

                if reverse_channel_map[i] == "1":
                   soorat[j]+=1
                   makhraj[i] = row_sum(predict_bandwidth_matrix,i)

        sum_makhraj = [0] * number_of_piconets
        for i in range(0,37):
            for j in range(0,number_of_piconets):

                #convert to binary
                binary = bin(int(master_maps[j],16))[2:].zfill(40)
                reverse_channel_map = binary[::-1]

                if reverse_channel_map[i] == "1":
                   sum_makhraj[j] += makhraj[i]

        bandwidth_predict = [m / n for m, n in zip(soorat, sum_makhraj)]
        temp_avg_predict_bw_sim.append(sum(bandwidth_predict)/len(bandwidth_predict))


        #calculate bandwidth indicator for this piconet after simulation
        temp_avg_bw_sim.append(bandwidth_indicator())



        #print("Before Experimental Channel Maps: ",master_maps)
        # Start Experimental Algorithm

        hop_results = []
        del hop_results[:]
        for i in range(0, number_of_piconets):
            hop_results.append(CSA2(ex_master_maps[i],ex_access_address[i],length_of_hops))


        channel_collision_probability_piconet_list = [] * number_of_piconets
        for i in range(0, number_of_piconets):
            channel_collision_probability_piconet_list.append([])
            for j in range(0,37):
                channel_collision_probability_piconet_list[i].append(channel_collision_probability_piconet(i,j))


        # channel sepration each channel maximum for 1 piconet
        chosen = []
        #skip these piconets in next step
        skipList = []

        for i in range(0,number_of_piconets):

            #bayad tooye channel haye khub begardim
            binary_master_map = bin(int(master_maps[i],16))[2:].zfill(40)
            reverse_binary_master_map = binary_master_map[::-1]

            minpos1 = 0
            minpos1found = 0
            minpos2 = 0
            minpos2found = 0

            for j in range(0,37):
                if channel_collision_probability_piconet_list[i][j] <= channel_collision_probability_piconet_list[i][minpos1] and reverse_binary_master_map[j] == "1"  and j not in chosen:
                    minpos1found = 1
                    minpos1 = j

            if minpos1found:
                for j in range(0,37):
                    if channel_collision_probability_piconet_list[i][j] <= channel_collision_probability_piconet_list[i][minpos2] and reverse_binary_master_map[j] == "1"  and j not in chosen and j!=minpos1:
                        minpos2found = 1
                        minpos2 = j


            if minpos1found and minpos2found:
                chosen.append(minpos1)
                chosen.append(minpos2)

                #channel mape piconet i ro taghir bedim joori ke faghat minpos1 o minpos2 ro dashte bashe
                #age 2 ta peyda nashod channel mapo taghir nemidim

                for k in range(0,len(reverse_binary_master_map[0:37])):
                    if reverse_binary_master_map[k] == "1" and k != minpos1 and k!= minpos2:
                        listreverse_binary_master_map = list(reverse_binary_master_map)
                        listreverse_binary_master_map[k] = "0"
                        reverse_binary_master_map = ''.join(listreverse_binary_master_map)

                double_reverse_binary_master_map = reverse_binary_master_map[::-1]

                master_maps[i] = hex(int(double_reverse_binary_master_map, 2))

            else:
                #map doesnt change
                skipList.append(i)


        while len(skipList) != number_of_piconets:
            for k in range(0,number_of_piconets):

                if k not in skipList:
                    #bayad tooye channel haye khub begardim
                    binary_master_map = bin(int(ex_master_maps[k],16))[2:].zfill(40)
                    reverse_binary_master_map = binary_master_map[::-1]

                    minpos = 0
                    minposfound = 0

                    for j in range(0,37):
                        if channel_collision_probability_piconet_list[k][j] <= channel_collision_probability_piconet_list[k][minpos] and reverse_binary_master_map[j] == "1"  and j not in chosen:
                            minposfound = 1
                            minpos = j
                            break

                    if minposfound:
                        chosen.append(minpos)

                        #be channel mape in master minpos ro ham ezafe mikonim
                        binary_master_map = bin(int(master_maps[k],16))[2:].zfill(40)
                        reverse_binary_master_map = binary_master_map[::-1]

                        if reverse_binary_master_map[minpos] == "0":
                            listreverse_binary_master_map = list(reverse_binary_master_map)
                            listreverse_binary_master_map[minpos] = "1"
                            reverse_binary_master_map = ''.join(listreverse_binary_master_map)

                        double_reverse_binary_master_map = reverse_binary_master_map[::-1]

                        master_maps[k] = hex(int(double_reverse_binary_master_map, 2))

                    else:
                        skipList.append(k)


        ## calculate system collision probability with new master maps
        del hop_results[:]

        for i in range(0, number_of_piconets):
            hop_results.append(CSA2(master_maps[i],master_access[i],length_of_hops))

        system_collision = system_collision_probability()
        #print(system_collision)
        temp_output_experimental_collision.append(system_collision)


        #calculate bandwidth prediction after experimental
        predict_bandwidth_matrix = [ [0]*number_of_piconets for i in range(0,37)]

        for i in range(0,37):
            for j in range(0,number_of_piconets):
                #convert to binary
                binary = bin(int(master_maps[j],16))[2:].zfill(40)
                reverse_channel_map = binary[::-1]
                if reverse_channel_map[i] == "1":
                    predict_bandwidth_matrix[i][j] = 1

        soorat = [0] * number_of_piconets
        makhraj = [0] * 37
        for i in range(0,37):
            for j in range(0,number_of_piconets):

                #convert to binary
                binary = bin(int(master_maps[j],16))[2:].zfill(40)
                reverse_channel_map = binary[::-1]

                if reverse_channel_map[i] == "1":
                   soorat[j]+=1
                   makhraj[i] = row_sum(predict_bandwidth_matrix,i)

        sum_makhraj = [0] * number_of_piconets
        for i in range(0,37):
            for j in range(0,number_of_piconets):

                #convert to binary
                binary = bin(int(master_maps[j],16))[2:].zfill(40)
                reverse_channel_map = binary[::-1]

                if reverse_channel_map[i] == "1":
                   sum_makhraj[j] += makhraj[i]

        bandwidth_predict = [m / n for m, n in zip(soorat, sum_makhraj)]
        temp_avg_predict_bw_ex.append(sum(bandwidth_predict)/len(bandwidth_predict))

        #calculate bandwidth indicator for this piconet after experimental
        temp_avg_bw_ex.append(bandwidth_indicator())

        #print("After Experimental Channel Maps: ",master_maps)


    output_ble_default_collision.append(sum(temp_output_ble_default_collision)/len(temp_output_ble_default_collision))
    output_remove_bad_channels_collision.append(sum(temp_output_remove_bad_channels_collision)/len(temp_output_remove_bad_channels_collision))
    output_experimental_collision.append(sum(temp_output_experimental_collision)/len(temp_output_experimental_collision))
    avg_num_channel_default.append(sum(temp_avg_num_channel_default)/len(temp_avg_num_channel_default))
    avg_num_channel_sim.append(sum(temp_avg_num_channel_sim)/len(temp_avg_num_channel_sim))

    avg_predict_bw_default.append(sum(temp_avg_predict_bw_default)/len(temp_avg_predict_bw_default))
    avg_predict_bw_sim.append(sum(temp_avg_predict_bw_sim)/len(temp_avg_predict_bw_sim))
    avg_predict_bw_ex.append(sum(temp_avg_predict_bw_ex)/len(temp_avg_predict_bw_ex))

    avg_bw_default.append(sum(temp_avg_bw_default)/len(temp_avg_bw_default))
    avg_bw_sim.append(sum(temp_avg_bw_sim)/len(temp_avg_bw_sim))
    avg_bw_ex.append(sum(temp_avg_bw_ex)/len(temp_avg_bw_ex))

#print('Finished Experimental Algorithm')

#print(output_number_of_piconets)
#print(output_ble_default_collision)
#print(output_remove_bad_channels_collision)
#print(output_experimental_collision)

#save data in csv file

csv_number_of_piconets = ['number_of_piconets']
csv_output_ble_default_collision = ['output_ble_default_collision']
csv_output_remove_bad_channels_collision = ['output_remove_bad_channels_collision']
csv_output_experimental_collision = ['output_experimental_collision']

csv_output_default_predict_bw = ['csv_output_default_predict_bw']
csv_output_sim_predict_bw = ['csv_output_sim_predict_bw']
csv_output_experimental_predict_bw = ['csv_output_experimental_predict_bw']

csv_output_default_bw = ['csv_output_default_bw']
csv_output_sim_bw = ['csv_output_sim_bw']
csv_output_experimental_bw = ['csv_output_experimental_bw']

csv_number_of_piconets.extend(output_number_of_piconets)
csv_output_ble_default_collision.extend(output_ble_default_collision)
csv_output_remove_bad_channels_collision.extend(output_remove_bad_channels_collision)
csv_output_experimental_collision.extend(output_experimental_collision)

csv_output_default_predict_bw.extend(avg_predict_bw_default)
csv_output_sim_predict_bw.extend(avg_predict_bw_sim)
csv_output_experimental_predict_bw.extend(avg_predict_bw_ex)

csv_output_default_bw.extend(avg_bw_default)
csv_output_sim_bw.extend(avg_bw_sim)
csv_output_experimental_bw.extend(avg_bw_ex)

nms = [['m',m],csv_number_of_piconets, csv_output_ble_default_collision, csv_output_remove_bad_channels_collision, csv_output_experimental_collision,csv_output_default_bw,csv_output_sim_bw,csv_output_experimental_bw,csv_output_default_bw,csv_output_sim_predict_bw,csv_output_experimental_predict_bw]

f = open('results_m7.csv', 'w')

with f:

    writer = csv.writer(f)

    for row in nms:
        writer.writerow(row)

# close the file
f.close()

#bl.advertise()