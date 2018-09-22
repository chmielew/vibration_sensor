import pexpect
import time
import struct
import binascii
import numpy as np

def hexStrToFloat(hexstr):
    val = struct.unpack('>f', binascii.unhexlify(hexstr))
    return val

def hexStrToInt16(hexstr):
    val = int(hexstr[0:2],16) + (int(hexstr[3:5],16)<<8)
    return val

def hexStrToInt8(hexstr):
    val = int(hexstr[0:2],16)
    return val

def float_to_hex(f):
    return hex(struct.unpack('<I', struct.pack('<f', f))[0])

class Sensor:
    def __init__(self):
        # local variables containing values of needed parameters 
        # to send valid commands to sensor
        self.hnd_trigger_measurement = "0x86"
        self.hnd_set_threshold_for_monitoring = "0x2a"
        self.trigger_measurement_write_value = "0x01"
        self.threshold_monitoring_write_value = "0x01"
        self.read_calculated_value_hnd_dict = {
            "rms" : "0x58",
            "average" : "0x5a",
            "max_val" : "0x5c",
            "min_val" : "0x5e",
            "amplitude" : "0x60",
            "crest_factor" : "0x62"
        }
        self.read_signal_hnd = "0xb4"
        self.read_fft_hnd = "0xe2"
        
        
        self.child = pexpect.spawn("gatttool -I")

    def connect(self, address):
        self.child.sendline("connect {0}".format(address))
        try:
            self.child.expect("Connection successful", timeout=5)
        except:
            return False
        return True
        
    def disconnect(self):
        self.child.sendline("disconnect")

    def trigger_measurement(self, frequency, duration):
        command = "char-write-cmd " + self.hnd_trigger_measurement + " " + self.trigger_measurement_write_value + '{:04x}'.format(int(frequency)) + float_to_hex(float(duration))[2:]
        print(command)
        self.child.sendline(command)

    def read_calculated_value(self, chosen_value):
        command = "char-read-hnd " + self.read_calculated_value_hnd_dict[chosen_value]
        self.child.sendline(command)
        self.child.expect("Characteristic value/descriptor: ", timeout=10)
        self.child.expect("\r\n", timeout=10)
        response = str(self.child.before)
        response = response.replace(" ","")
        response = str(response[-2:] + response[-4:-2] + response[-6:-4] + response[:-6])
        if((self.read_calculated_value_hnd_dict[chosen_value] != self.read_calculated_value_hnd_dict["max_val"]) and (self.read_calculated_value_hnd_dict[chosen_value] != self.read_calculated_value_hnd_dict["min_val"]) and (self.read_calculated_value_hnd_dict[chosen_value] != self.read_calculated_value_hnd_dict["amplitude"])):
            #becouse it is a tuple with one element
            result = hexStrToFloat(response)
            return result[0]
        result = int(response[-4:],16)
        print(result)
        print(chosen_value + " " +response)
        return result

    def read_signal(self):
        command = "char-read-hnd " + self.read_signal_hnd
        control = '00'
        result_array = np.array([])
        while (control != '03'):
            self.child.sendline(command)
            self.child.expect("Characteristic value/descriptor: ", timeout=10)
            self.child.expect("\r\n", timeout=10)
            result = str(self.child.before)
 
            for x in range(3, len(result),6):
                current_element = str((result[x+3:x+5] + result[x:x+2]))
                if current_element != 'ffff':
                    result_array = np.append(result_array, hexStrToInt16(current_element))
            control = result[0:2]
        return result_array

    def read_fft(self):
        command = "char-read-hnd " + self.read_fft_hnd
        control = '00'
        result_array = np.array([])
        while (control != '03'):
            self.child.sendline(command)
            self.child.expect("Characteristic value/descriptor: ", timeout=10)
            self.child.expect("\r\n", timeout=10)
            result = str(self.child.before)
            print(result)
            for x in range(3, len(result),3):
                current_element = str((result[x:x+2]))
                if current_element != 'ff':
                    result_array = np.append(result_array, hexStrToInt8(current_element))
            control = result[0:2]
        print(np.size(result_array))
        result_array[0] = 0
        return result_array

    def set_threshold_for_threshold_exceeded_monitoring(self, threshold):
            command = "char-write-cmd " + self.hnd_set_threshold_for_monitoring + " " + self.threshold_monitoring_write_value + '{:04x}'.format(int(threshold))
            print(command)
            self.child.sendline(command)

    def monitor_threshold_exceeded(self):
        try:
            self.child.expect("Notification handle = 0x002a value: ", timeout=0.1)
            self.child.expect("\r\n", timeout=0.1)
        except:
            return False
        return True

    def is_measurement_finished(self):
        try:
            self.child.expect("Notification handle = 0x0086 value: ", timeout=0.1)
            self.child.expect("\r\n", timeout=0.1)
        except:
            return False
        return True
