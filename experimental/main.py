# Based on ReachView code from Egor Fedorov (egor.fedorov@emlid.com)
# Updated for Python 3.6.8 on a Raspberry  Pi


import time
import pexpect
import subprocess
import sys
import logging
import numpy as np
import random
import matplotlib.pyplot as plt
import csv


def replacer(s, newstring, index, nofail=False):
    # raise an error if index is outside of the string
    if not nofail and index not in range(len(s)):
        raise ValueError("index outside given string")

    # if not erroring, but the index is still not in the correct range..
    if index < 0:  # add it to the beginning
        return newstring + s
    if index > len(s):  # add it to the end
        return s + newstring

    # insert the new string between "slices" of the original
    return s[:index] + newstring + s[index + 1:]


logger = logging.getLogger("btctl")


class Bluetoothctl:

#A wrapper for bluetoothctl utility.


    def __init__(self):
        subprocess.check_output("rfkill unblock bluetooth", shell=True)
        self.process = pexpect.spawnu("bluetoothctl", echo=False)

    def send(self, command, pause=0):
        self.process.send(f"{command}\n")
        time.sleep(pause)
        if self.process.expect(["bluetooth", pexpect.EOF]):
            raise Exception(f"failed after {command}")

    def get_output(self, *args, **kwargs):

#Run a command in bluetoothctl prompt, return output as a list of lines.

        self.send(*args, **kwargs)
        return self.process.before.split("\r\n")

    def start_scan(self):

#Start bluetooth scanning process.

        try:
            self.send("scan on")
        except Exception as e:
            logger.error(e)

    def make_discoverable(self):

#Make device discoverable.

        try:
            self.send("discoverable on")
        except Exception as e:
            logger.error(e)

    def parse_device_info(self, info_string):

#Parse a string corresponding to a device.

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

#Return a list of tuples of paired and discoverable devices.

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

#Return a list of tuples of paired devices.

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

#Filter paired devices out of available.

        available = self.get_available_devices()
        paired = self.get_paired_devices()
        return [d for d in available if d not in paired]

    def get_device_info(self, mac_address):

#Get device info by mac address.

        try:
            out = self.get_output(f"info {mac_address}")
        except Exception as e:
            logger.error(e)
            return False
        else:
            return out

    def pair(self, mac_address):

#Try to pair with a device by mac address.

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

#Remove paired device by mac address, return success of the operation.

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

#Try to connect to a device by mac address.

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

#Try to disconnect to a device by mac address.

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

    def advertise(self,data):
        try:
            self.send(f"advertise off")
            self.send(f"advertise broadcast")
            self.send(f"menu advertise")
            self.send(f"clear")
            self.send(f"uuids 0x1800 0x1801 0x1802")
            self.send(f"name Reza")
            self.send(f"manufacturer 0x0059 "+data)
            self.send(f"back")
            self.send(f"advertise off")
            self.send(f"advertise on")
            time.sleep(20000)

        except Exception as e:
            logger.error(e)
            return False
        else:
            res = self.process.expect(
                ["Failed to disconnect", "Successful disconnected", pexpect.EOF]
            )
            return res == 1



number_of_piconets = 3

print("Init bluetooth...")
bl = Bluetoothctl()

print("Ready!")
bl.start_scan()
print("Scanning for 10 seconds...")

device_addresses = ["F2:87:B0:C7:B9:A9","D5:08:51:A6:BE:9F","D7:78:15:21:18:CD"]
temp_str = []
channel_maps = []
original_master_maps = []
master_maps = []

my_string = "\tManufacturerData Value:"
for i in range(0, number_of_piconets):
    temp_str.append(bl.get_device_info(device_addresses[i]))
    matched_indexes = []
    j = 0
    length = len(temp_str[i])

    while j < length:
        if my_string == temp_str[i][j]:
            matched_indexes.append(j)
            break
        j += 1

    print(temp_str[i])
    channel_maps.append(temp_str[i][j+1])
    this_channel_map = "0x" + channel_maps[i][2:18].replace(" ", "")

    #convert little endian to our concept
    list1 = list(this_channel_map)
    list1[2] = this_channel_map[10]
    list1[10] = this_channel_map[2]
    list1[3] = this_channel_map[11]
    list1[11] = this_channel_map[3]
    list1[4] = this_channel_map[8]
    list1[8] = this_channel_map[4]
    list1[5] = this_channel_map[9]
    list1[9] = this_channel_map[5]
    this_channel_map = ''.join(list1)

    original_master_maps.append(this_channel_map)
    master_maps.append(this_channel_map)

print("Master Maps:",master_maps)

# channel sepration each channel
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
        if reverse_binary_master_map[j] == "1"  and j not in chosen:
            minpos1found = 1
            minpos1 = j

    if minpos1found:
        for j in range(0,37):
            if reverse_binary_master_map[j] == "1"  and j not in chosen and j!=minpos1:
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
            binary_master_map = bin(int(original_master_maps[k],16))[2:].zfill(40)
            reverse_binary_master_map = binary_master_map[::-1]

            minpos = 0
            minposfound = 0

            for j in range(0,37):
                if reverse_binary_master_map[j] == "1"  and j not in chosen:
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


#add zero to master_maps if length is small
for piconet in range(0,number_of_piconets):
    print(len(master_maps[piconet]))
    if len(master_maps[piconet]) == 11:
        master_maps[piconet] = master_maps[piconet].replace ("x", "x0")

    elif len(master_maps[piconet]) == 10:
        master_maps[piconet] = master_maps[piconet].replace ("x", "x00")

    elif len(master_maps[piconet]) == 9:
        master_maps[piconet] = master_maps[piconet].replace ('x', 'x000')

    elif len(master_maps[piconet]) == 8:
        master_maps[piconet] = master_maps[piconet].replace ('x', 'x0000')

    elif len(master_maps[piconet]) == 7:
        master_maps[piconet] = master_maps[piconet].replace ('x', 'x00000')

    elif len(master_maps[piconet]) == 6:
        master_maps[piconet] = master_maps[piconet].replace ('x', 'x000000')

    elif len(master_maps[piconet]) == 5:
        master_maps[piconet] = master_maps[piconet].replace ('x', 'x0000000')

    elif len(master_maps[piconet]) == 4:
        master_maps[piconet] = master_maps[piconet].replace ('x', 'x00000000')

    elif len(master_maps[piconet]) == 3:
        master_maps[piconet] = master_maps[piconet].replace ('x', 'x000000000')

    elif len(master_maps[piconet]) == 2:
        master_maps[piconet] = master_maps[piconet].replace ('x', 'x0000000000')

    elif len(master_maps[piconet]) == 1:
        master_maps[piconet] = master_maps[piconet].replace ('x', 'x00000000000')

convert_list = [0] * number_of_piconets
print("before convert master_maps:",master_maps)
#convert our concept to little endian
for i in range(0,number_of_piconets):

    convert_list[i] = list(master_maps[i])
    convert_list[i][2] = master_maps[i][10]
    convert_list[i][10] = master_maps[i][2]
    convert_list[i][3] = master_maps[i][11]
    convert_list[i][11] = master_maps[i][3]
    convert_list[i][4] = master_maps[i][8]
    convert_list[i][8] = master_maps[i][4]
    convert_list[i][5] = master_maps[i][9]
    convert_list[i][9] = master_maps[i][5]
    master_maps[i] = ''.join(convert_list[i])

print("after convert master_maps:",master_maps)
data = ""
for piconet in range(0,number_of_piconets):
    temp_data = ""
    print(master_maps[piconet])
    temp_data = replacer(master_maps[piconet]," 0x"+master_maps[piconet][4] , 4)
    temp_data = replacer(temp_data," 0x"+temp_data[9] , 9)
    temp_data = replacer(temp_data," 0x"+temp_data[14] , 14)
    temp_data = replacer(temp_data," 0x"+temp_data[19] , 19)
    data += " " + temp_data

print(data)
bl.advertise(data)
