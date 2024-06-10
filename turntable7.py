###########################
# VERSION 7 # 19 Mar 2019 #
###########################

import time
from time import sleep
from struct import *
from array import array
import math
from math import log
import os
import sys
from enum import Enum
import datetime
import threading
import platform

if not __debug__:
    import serial

WINDOWS_10 = False

this_platform = platform.system()
if this_platform in ('Linux', 'Darwin'):
    import tty, termios  # for getch()
elif this_platform == 'Windows':
    import msvcrt
    w = sys.getwindowsversion()
    if w[0] >= 10: # [0] is major versiona, e.g. sys.getwindowsversion(major=6, minor=1, build=7601, platform=2, service_pack='Service Pack 1')
        # and now enable ANSI processing on Windows 10 and above
        WINDOWS_10 = True
        import ctypes

        kernel32 = ctypes.windll.kernel32
        kernel32.SetConsoleMode(kernel32.GetStdHandle(-11), 7)
        ######################
else:
    print("Unknown platform. Exiting")
    sys.exit()

global_t = None  # Global timer for menus display
global_t2 = None  # Global timer for polling second serial port

global_start_time = 0
global_start_pos = 0



########### motor identification and parameters  ####################
#
MTR = 1                         # TMCL address of motor
# 80 s/rot
#VEL = 1101                     # This is in microsteps per sec, adjust as necessary to achieve desired rotation rate
#DIV = 2                        # velocity divisor

# 360 sec/rot
VEL = 1957                      # This is in microsteps per sec, adjust as necessary to achieve desired rotation rate
DIV = 4                         # velocity divisor

MICROSTEPS = 128  # microsteps per full step
#MICROSTEPS = 64                # microsteps per full step

# Some default speed settings in sec/rot
MOT_HIGH_SPEED = 50
MOT_MED_SPEED = 360
MOT_LOW_SPEED = 3600

NUDGE_DEG = 1
SMALL_NUDGE_DEG = 0.1
#
####################################################################
#
#SERIAL_DEV = '/dev/tty.usbserial'          # macos RS-232 USB adapter
#SERIAL_DEV = '/dev/tty.usbserial-A703P9MW'  # macos RS-485 USB adapter 
#SERIAL_DEV = '/dev/tty.usbmodemTMCSTE1'  # macos TM-1141 USB interface
#SERIAL_DEV = '/dev/tty.usbserial-FT1N0K2F' # macosx RS485 from RS Electronics interface
#
#SERIAL_DEV2 = '/dev/tty.wchusbserial1420'  # macos Arduino Nano

SER2_BAUDRATE=115200

SERIAL_DEV='COM6'
SERIAL_DEV2='COM5'

####################################################################


class movement(Enum):
    ROR = 1  # Rotate right continuous
    ROL = 2  # Rotate left continuous
    ADVR = 3  # Advance right a fixed amount (defined below). Also, negative left = right.
    ADVL = 4  # Advance left a fixed amount (defined below). Also, negative right = left.
    MV0 = 5  # Move to the origin, i.e. to (micro)step position = 0
    RS0 = 6  # Reset the origin to the current (micro)step position
    STP = 7  # Stop
    SETV = 8  # Set velocity
    GETP = 9  # Get current (micro)step position
    MVP = 10  # Move to an absolute position
    RSMST = 11  # Reset microstep resolution
    REV = 12  # Reverse direction
    SETVDIV = 13  # Velocity divisor


class turntable:                                # Class for B&K Type 3922 turntable equipped with Trinamic Stepper Motor with controller
    def __init__(self, motor_address, motor_velocity, motor_vel_div, motor_mst_res):
        self.motor_address = motor_address      # TMCL address
        self.motor_velocity = motor_velocity    # integer velocity
        self.motor_mst_res = motor_mst_res      # Microstep resolution, in microsteps/fullstep
        self.motor_vel_div = motor_vel_div      # velocity divisor
        self.saved_vel = self.motor_velocity
        self.saved_veldiv = self.motor_vel_div
        self.motor_dir = 1                      # Default is counter-clockwise from above
        self.remote_sweeping = False
        self.sweep_preparation = False
        self.ser = None
        if not __debug__:
            self.ser = serial.Serial(port=SERIAL_DEV)
            self.ser.write(
                self.add_checksum(
                    pack('>BBBBi', self.motor_address, 5, 4, 0, self.motor_velocity)))

            self.ser.read(9)
            self.ser.write(
                self.add_checksum(
                    pack('>BBBBi', self.motor_address, 5, 140, 0,
                                        int(math.log(self.motor_mst_res, 2)))))
            self.ser.read(9)
            self.ser.write(
                self.add_checksum(
                    pack('>BBBBi', self.motor_address, 5, 154, 0, self.motor_vel_div)))

            self.ser.read(9)
            self.ser.write(
                self.add_checksum(
                    pack('>BBBBi', self.motor_address, 9, 90, 0,
                                        self.motor_dir)))  # reverse motor direction
            self.ser.read(9)

        self.set_values()

    def set_values(self):
        ##### non-changing motor and turntable characteristics ####
        SPR = 200  # full steps per revolution
        RATIO = 420                                                                             # ratio from spur to full turntable, estimated currently
        #RATIO = 1
        ###########################################################

        self.motor_ratio = RATIO

        self.mstp_per_rev = SPR * self.motor_mst_res * self.motor_ratio  # microsteps per full turntable revolution, 20,480,000
        self.mstp_per_deg = self.mstp_per_rev / 360  # microsteps per degree of rotation of full turntable
        self.mstp_hz = (16e6 * self.motor_velocity) / ((2**self.motor_vel_div) * 2048 * 32)  # microstep frequency

        #print("ratio = {}, mstp_per_rev = {}, mstp_per_deg = {}, mstp_hz = {}".format(self.motor_ratio,self.mstp_per_rev,self.mstp_per_deg,self.mstp_hz))
        #getch()

    def add_checksum(self, b):                              # b is payload bytestream
        csum = 0                                            # checksum
        for x in iter_unpack('B', b):
            csum += (x[0])                                  # first element of returned tuple is a byte
        return b + pack('B', csum % 256)                    # concat original ade checksum as bytestring

    def mot_rpm(self):
        self.mstp_hz = (16e6 * self.motor_velocity) / ( (2**self.motor_vel_div) * 2048 * 32)  # microstep frequency
        return self.mstp_hz / self.mstp_per_rev * 60  # (mStep / sec) / (mStep / rev) * 60 = rev/sec * 60 = rev/min

    def set_rotation_rate(self, speed):
        # speed must be specified in sec/rotation
        eff_rpm = 1 / speed * 60  # in rot per min
        self.mstp_hz = eff_rpm * self.mstp_per_rev / 60
        self.motor_velocity = int(self.mstp_hz * ((2**self.motor_vel_div) * 2048 * 32) / 16e6)
        if self.motor_velocity > 2047:
            self.motor_vel_div -= 1
            self.set_rotation_rate(speed)
        elif self.motor_velocity < 50:
            self.motor_vel_div += 1
            self.set_rotation_rate(speed)

        if not (0 <= self.motor_vel_div <= 13):
            print("Invalid pulse dividor calculated. Speed not achievable. Resetting to default values.")
            getch()
            self.motor_velocity = VEL 
            self.motor_vel_div = DIV 

    def gen_command(self, command, degrees=0, position=0, vel_tup=(None, None)):

        velocity = vel_tup[0]
        vel_div = vel_tup[1]

        if velocity is None:
            velocity = self.motor_velocity
        if vel_div is None:
            vel_div = self.motor_vel_div

        switcher = {
            movement.ROR: self.add_checksum(pack('>BBBBi', self.motor_address, 1, 0, 0, velocity)),
            movement.ROL: self.add_checksum(pack('>BBBBi', self.motor_address, 2, 0, 0, velocity)),
            movement.STP: self.add_checksum(pack('>BBBBi', self.motor_address, 3, 0, 0, 0)),
            movement.MV0: self.add_checksum(pack('>BBBBi', self.motor_address, 4, 0, 0, 0)),  # advance absolute to zero
            movement.ADVR: self.add_checksum( pack('>BBBBi', self.motor_address, 4, 1, 0, int(degrees * self.mstp_per_deg))),  # advance releative
            movement.ADVL: self.add_checksum( pack('>BBBBi', self.motor_address, 4, 1, 0, -1 * int( degrees * self.mstp_per_deg))),  #advance -relative
            movement.RS0: self.add_checksum(pack('>BBBBi', self.motor_address, 5, 1, 0, 0)),
            movement.RSMST: self.add_checksum( pack('>BBBBi', self.motor_address, 5, 140, 0, int(math.log(self.motor_mst_res, 2)))),
            movement.SETV: self.add_checksum( pack('>BBBBi', self.motor_address, 5, 4, 0, velocity)),  # Maximum velocity
            movement.SETVDIV: self.add_checksum(pack('>BBBBi', self.motor_address, 5, 154, 0, vel_div)),
            movement.MVP: self.add_checksum( pack('>BBBBi', self.motor_address, 4, 0, 0, int( position))),  # advance to absolute position
            movement.GETP: self.add_checksum(pack('>BBBBi', self.motor_address, 6, 1, 0, 0)),
            movement.REV: self.add_checksum( pack('>BBBBi', self.motor_address, 9, 90, 0, self.motor_dir))  # reverse motor direction
        }
        return switcher.get(command)


class trinamic_control(threading.Thread):
    response = ''  # replaced by response from motor

    def __init__(self, ser, payload, lock):
        threading.Thread.__init__(self)
        self.lock = lock
        self.ser = ser
        self.payload = payload

    def run(self):
        self.lock.acquire()
        #print("Executing payload - {}".format(self.payload))
        if not __debug__:
            self.ser.write(self.payload)
            try:
                self.response = unpack('>BBBBiB', self.ser.read(9))
            except 'SerialException':
                print("Could not read data, flushing serial port. Try whatever you just did again, perhaps?")
                self.ser.flush()

        self.lock.release()


class turntable_command:
    def __init__(self,turntable,ser_lock):
        self.turntable = turntable
        self.ser_lock = ser_lock

    def stop(self):
        t = trinamic_control(self.turntable.ser, self.turntable.gen_command(movement.STP), self.ser_lock)
        t.start()
        t.join()

    def reverse(self):
        self.turntable.motor_dir = 1 - self.turntable.motor_dir  # swap motor_dir from 0 to 1 or 1 to 0
        t = trinamic_control(self.turntable.ser, self.turntable.gen_command(movement.REV), self.ser_lock)
        t.start()
        t.join()

    def set_rot_rate(self,rate):                        # rate is float in sec/rot
        # set velocity and pulse divisor to max values (min speed) so that they are minimised during rate setting
        self.turntable.motor_vel_div = 13
        self.turntable.motor_velocity = 2047
        self.turntable.set_rotation_rate(rate)# calculate velocity and pulse divisor values and store them in turntable object 
        t = trinamic_control(self.turntable.ser, self.turntable.gen_command(movement.SETV,self.turntable.motor_velocity), self.ser_lock)
        t.start()
        t.join()
        t = trinamic_control(self.turntable.ser, self.turntable.gen_command(movement.SETVDIV, self.turntable.motor_vel_div), self.ser_lock)
        t.start()
        t.join()
        
    def rot_right(self):
        t = trinamic_control(self.turntable.ser, self.turntable.gen_command(movement.ROR), self.ser_lock)
        t.start()
        t.join()

    def rot_left(self):
        t = trinamic_control(self.turntable.ser, self.turntable.gen_command(movement.ROL), self.ser_lock)
        t.start()
        t.join()

    def immediate_sweep_180(self):
        t = trinamic_control(self.turntable.ser, self.turntable.gen_command(movement.ADVR, degrees=180), self.ser_lock)
        t.start()
        t.join()

    def get_pos(self):
        if not __debug__:
            t = trinamic_control(self.turntable.ser, self.turntable.gen_command(movement.GETP), self.ser_lock)
            t.start()
            t.join()
            return response_value(t.response)
        else:
            return 0
    
    def set_pos(self, pos):
        t = trinamic_control(self.turntable.ser, self.turntable.gen_command(movement.MVP, pos), self.ser_lock)
        t.start()
        t.join()

    def move_to_angle(self,angle=0):
        t = trinamic_control(self.turntable.ser, self.turntable.gen_command(movement.MVP, position=self.turntable.mstp_per_deg * angle), self.ser_lock)
        t.start()
        t.join()

    def inc_angle_right(self,angle=0):
        t = trinamic_control(self.turntable.ser, self.turntable.gen_command(movement.ADVR, degrees=angle), self.ser_lock)
        t.start()
        t.join()

    def set_velocity_val(self, velocity):
        t = trinamic_control(self.turntable.ser, self.turntable.gen_command(movement.SETV, velocity), self.ser_lock)
        t.start()
        t.join()

    def save_rot_rate_vals(self):        
        self.turntable.saved_veldiv = self.turntable.motor_vel_div
        self.turntable.saved_vel = self.turntable.motor_velocity

    def restore_rot_rate_vals(self):
        self.turntable.motor_vel_div = self.turntable.saved_veldiv # reset to corect value not default
        self.turntable.motor_velocity = self.turntable.saved_vel # reset velocity value to motor
        t = trinamic_control(self.turntable.ser, self.turntable.gen_command(movement.SETV), self.ser_lock)
        t.start()
        t.join()
        t = trinamic_control(self.turntable.ser, self.turntable.gen_command(movement.SETVDIV), self.ser_lock)
        t.start()
        t.join()

    def nudge_left(self,deg=1):
        self.save_rot_rate_vals()
        self.set_rot_rate(10000)
        t = trinamic_control(self.turntable.ser, self.turntable.gen_command(movement.ADVL, deg), self.ser_lock)
        t.start()
        t.join()
        self.restore_rot_rate_vals()

    def nudge_right(self,deg=1):
        self.save_rot_rate_vals()
        self.set_rot_rate(10000)
        t = trinamic_control(self.turntable.ser, self.turntable.gen_command(movement.ADVR, deg), self.ser_lock)
        t.start()
        t.join()
        self.restore_rot_rate_vals()

    def small_nudge_left(self,deg=0.1):
        self.nudge_left(deg)

    def small_nudge_right(self,deg=0.1):
        self.nudge_right(deg)

    def goto_origin(self):
        t = trinamic_control(self.turntable.ser, self.turntable.gen_command(movement.MV0), self.ser_lock)
        t.start()
        t.join()

    def reset_origin(self):
        t = trinamic_control(self.turntable.ser, self.turntable.gen_command(movement.RS0), self.ser_lock)
        t.start()
        t.join()

    def get_angle(self):
        return (self.get_pos() / self.turntable.mstp_per_deg)
    
    def prepare_to_sweep_180(self):
        self.stop()
        self.turntable.sweep_preparation = True
        self.save_rot_rate_vals()
        self.set_rot_rate(MOT_HIGH_SPEED)
        self.move_to_angle(-90)

def getch():
    if this_platform == 'Windows':
        m = msvcrt.getche()
        if (msvcrt.kbhit() > 0):  # two bytes due to control key like arrows
            m = msvcrt.getch()
        return m.decode()
    else:  # Linux, macos
        old_settings = termios.tcgetattr(0)
        new_settings = old_settings[:]
        new_settings[3] &= ~termios.ICANON & ~termios.ECHO
        try:
            termios.tcsetattr(0, termios.TCSANOW, new_settings)
            ch = sys.stdin.read(1)
            if ch == '\x1b':
                ch = sys.stdin.read(2)
                # remap chars for compatibility with windows control codes
                if ch == '[A':
                    ch = 'H'
                elif ch == '[B':
                    ch = ' '  # ignore down key
                elif ch == '[C':
                    ch = 'M'
                elif ch == '[D':
                    ch = 'K'
            return ch
        finally:
            termios.tcsetattr(0, termios.TCSANOW, old_settings)

def get_current_millis():
    return int(round(time.time() * 1000))

def response_value(response):
    if not __debug__:
        return response[4]
    else:
        return 0

def poll_serial(ser2, turntable, ser_lock):
    if not __debug__:
        global global_t2
        ser2_data = ''
        i = 0

        n = ser2.in_waiting
        if (n > 0):
            ser2_data = ser2.read(n)  # read a byte
            for i in range(n):
                ser2_val = int(chr(ser2_data[i]))  # convert the byte to integer
                # start a sweep as we have been triggered from analyser via arduino interface as long as we aren't already doing so or preparating to do so
                if (ser2_val == 1) and not turntable.remote_sweeping and not turntable.sweep_preparation:  
                    turntable_command(turntable,ser_lock).immediate_sweep_180()
                    turntable.remote_sweeping = True
                    print("remote sweep initiated")

        # now call ourselves again on timer thread
        global_t2 = threading.Timer( 0.01, poll_serial, args=[ ser2, turntable, ser_lock, ])  # set a timer for faster than 0.1 seconds as arduino is emitting data every 100 ms
        global_t2.start()


def display_menu(turntable, lock):
    global global_t  # timer 1 for display updates
    global global_start_time
    global global_start_pos

    UIstring = ("O  return to Origin (zero)\n"
                "S  Stop motor\n"
                "W  sWeep 180 deg\n"
                "E  swEep preparation (go to -90 deg)\n"
                "R  rotate Right\n"
                "L  rotate Left\n"
                "{}  advance right {} deg\n"
                "{}  advance left {} deg\n"
                "]  advance right {} deg\n"
                "[  advance left {} deg\n"
                "Z  reset origin (Zero) to current position\n"
                "P  go to Position\n"
                "N  iNcrement by (+/-)aNgle\n"
                "G  Go to (+/-)anGle (ref Origin)\n"
                "I  display system Info\n"
                "C  system Configuration\n\n"
                "1  Speed low ({} sec/rot)\n"
                "2  Speed medium ({} sec/rot)\n"
                "3  Speed high ({} sec/rot)\n\n"
                "Q  Quit\n\n>".\
                format(u'\u2192',NUDGE_DEG, u'\u2190',NUDGE_DEG, SMALL_NUDGE_DEG, SMALL_NUDGE_DEG, MOT_LOW_SPEED,MOT_MED_SPEED,MOT_HIGH_SPEED))

    # if we are moving to start sweepstart location then just let it happen, or clear flag if we are there already
    if turntable.sweep_preparation:
        if int(turntable_command(turntable,lock).get_angle()) == -90:
            turntable.sweep_preparation = False
            turntable_command(turntable,lock).restore_rot_rate_vals()
            
    global_t = threading.Timer( 1, display_menu, args=[ turntable, lock, ])
    global_t.start()

    if not __debug__:
        pos = turntable_command(turntable,lock).get_pos()
    else:
        pos = 0

    delta_pos = pos - global_start_pos  # change in mstp since last time
    current_time = get_current_millis()
    delta_time = ( current_time - global_start_time) / 1000  # duration since update in seconds

    try:
        degrees = delta_pos / turntable.mstp_per_deg
        current_speed = delta_time / ( delta_pos / turntable.mstp_per_deg) * 360 / turntable.motor_ratio  # current speed in sec/rot since we started moving
    except ZeroDivisionError:
        current_speed = 0

    if this_platform == "Windows":
        os.system("cls")
        pass
    else:
        print("\n\033[H\033[J", end='')

    try:
        degmin = 60*360/(current_speed * turntable.motor_ratio ) 
    except ZeroDivisionError:
        degmin = 0.0

    print( ("CSIRO B&K / Trinamic Turntable Control Software\n\n", "\033[3;37;44mCSIRO B&K / Trinamic Turntable Control Software\033[0;37;40m\n\n")[WINDOWS_10]+
            "Current Angle: {:.2f} degrees (ref Origin) \n"
            "Target Speed: {:.2f} sec/rev ({:.2f} rpm, {:.2f} rph, {:.2f} deg/min, {:.2f} deg/s)\n"
            "Current Speed: {:.2f} sec/rev {:.3f} deg/min\n"
            "Position: {}\n"
            "Velocity: {} (int) Velocity Divisor: {}\n"
            "Ratio: {} Microstep/deg: {:d}\n"
            "Positive direction (from above):".
                #format(pos / turntable.mstp_per_deg,
                format(turntable_command(turntable,lock).get_angle(), 
                    # sec/rev   rpm     rph     deg/min     deg/s
                    60 / turntable.mot_rpm(), turntable.mot_rpm(), turntable.mot_rpm()*60, turntable.mot_rpm()*360, turntable.mot_rpm()*360/60,
                    current_speed * turntable.motor_ratio, degmin,  
                    pos,
                    turntable.motor_velocity, turntable.motor_vel_div,
                    turntable.motor_ratio, int(turntable.mstp_per_deg)) +
                    (' Clockwise, Left', ' Counterclockwise, Right')[turntable.motor_dir > 0] + '\n' +
                    ('\n', '\033[1;32;40m Remote Sweep Activated - (S)top and reset \033[0;37;40m\n')[turntable.remote_sweeping] + 
                    ('\n', '\033[1;31;40m Moving to sweep start position, please wait... (S)top rotation or (Q)uit program.\033[0;37;40m\n')[turntable.sweep_preparation] +
                    '\n' + UIstring,end='')

def main():
    global global_t, global_t2  # refernce to global timers so we can stop them as necessary
    global global_start_time
    global global_start_pos

    # create a turntable object with motor at address 1
    t = turntable(MTR, VEL, DIV, MICROSTEPS)  
    # create serial lock on first port
    ser_lock = threading.Lock()  

    # set a default speed
    turntable_command(t,ser_lock).set_rot_rate(MOT_MED_SPEED)

    # first get our current location 
    global_start_pos = turntable_command(t,ser_lock).get_pos()

    display_menu(t, ser_lock)

    if not __debug__:
        # read from the second serial port to see if we have been triggered
        ser2 = serial.Serial(SERIAL_DEV2,baudrate=SER2_BAUDRATE)
        ser2.flush()
        poll_serial(ser2, t, ser_lock)

    while True:
        # get our current location, first value will be somewhat approxiate as turntable may still be moving
        global_start_pos = turntable_command(t,ser_lock).get_pos()
        # get current time
        global_start_time = get_current_millis()

        command = getch()
        #make sure we can stop anything or quite at any time
        if command in ('S', 's'):
            turntable_command(t,ser_lock).stop()
            if not __debug__: global_t2.cancel()
            t.remote_sweeping = False
            t.sweep_preparation = False
            if not __debug__: poll_serial(ser2, t, ser_lock)  # start polling serial port again

        elif command in ('Q', 'q'):
            global_t.cancel()
            if not __debug__: global_t2.cancel()
            print("Stopping motor")
            turntable_command(t,ser_lock).stop()
            sys.exit()  # Program exit

        # if we are sweeping because of remote control then don't interrupt
        if t.remote_sweeping or t.sweep_preparation:
            continue

        

        # otherwise let's look for other valid user input
        if command in ('M', 'K', '[', ']','1','2','3','R', 'r', 'L', 'l', 'S', 's', 'O', 'o', 'Z', 'z', 'W', 'w', 'E', 'e'):
            if command in ('M'):  # 'M' is right arrow after control sequence
                turntable_command(t,ser_lock).nudge_right()
            elif command in ('K'):  # 'K' is left arrow after control sequence
                turntable_command(t,ser_lock).nudge_left()
            elif command in (']'):  
                turntable_command(t,ser_lock).small_nudge_right()
            elif command in ('['):  
                turntable_command(t,ser_lock).small_nudge_left()
            elif command in ('1'):
                turntable_command(t,ser_lock).set_rot_rate(MOT_LOW_SPEED)
            elif command in ('2'):
                turntable_command(t,ser_lock).set_rot_rate(MOT_MED_SPEED)
            elif command in ('3'):
                turntable_command(t,ser_lock).set_rot_rate(MOT_HIGH_SPEED)
            elif command in ('R', 'r'):
                turntable_command(t,ser_lock).rot_right()
            elif command in ('L', 'l'):
                turntable_command(t,ser_lock).rot_left()
            elif command in ('W', 'w'):
                turntable_command(t,ser_lock).immediate_sweep_180()
            elif command in ('E', 'e'):
                turntable_command(t,ser_lock).prepare_to_sweep_180()             
            elif command in ('O', 'o'):
                turntable_command(t,ser_lock).goto_origin()
            elif command in ('Z', 'z'):
                # Stop first
                turntable_command(t,ser_lock).stop()
                # Then set zero as this location
                turntable_command(t,ser_lock).reset_origin()
        else:
            # stop updating the menu and info screen
            global_t.cancel()
            # stop reading from the second serial port
            if not __debug__: global_t2.cancel()
            # and now process other options that require immediate thread treatment
            if command in ('G', 'g'):
                angle = input('\tEnter Target Angle (-359 to 359)\n\t>>')
                try:
                    angle = int(angle)
                    if not (-359 <= angle <= 359):
                        raise ValueError("Value not between -359 and 359")
                except ValueError:
                    print("Invalid value!")
                    continue

                # Move to requested angle
                turntable_command(t,ser_lock).move_to_angle(angle)

            elif command in ('N', 'n'):
                # increment in right hand direction
                turntable_command(t,ser_lock).stop()
                angle = input('\tEnter Increment Angle\n\t>>')
                try:
                    angle = int(angle)
                except ValueError:
                    print("Invalid integer!")
                    continue
                turntable_command(t,ser_lock).inc_angle_right(angle)

            elif command in ('P', 'p'):
                position = input('\tEnter Absolute (microstep) position\n\t>>')
                try:
                    position = int(position)
                except ValueError:
                    print("Invalid integer!")
                    continue
                print("position is {}\n".format(position))
                if not __debug__:
                    global_start_pos = turntable_command(t,ser_lock).set_pos(position)
                else:
                    global_start_pos = 0

            elif command in ('I', 'i'):
                print("\nOperating at {:.2f} rpm ({:.2f} rph {:.2f} mstp_hz)".format(
                    t.mot_rpm(), t.mot_rpm() * 60, t.mstp_hz))
                print("Microstep resolution: {}, Microsteps per degree: {:.0f}, Velocity: {}, Pulse Divisor: {}".
                    format(t.motor_mst_res, t.mstp_per_deg, t.motor_velocity, t.motor_vel_div))
                print("\nPython software interface written by C. Preston, 2018 - 2019\n")
                print("-- Press any key to continue --")
                getch()

            elif command in ('C', 'c'):
                config = input(
                    '\n\tS: rotation Speed (rate, sec/rev)\n\tD: rotation Speed (rate, deg/min)\n\tV: Velocity (int)\n\tR: Reverse motor directions\n\t>>'
                )
                if config in ('V', 'v'):
                    velocity = input(
                        '\t\tValue (0 to 2047), d = default ({})\n\t\t>>>'.format(VEL))
                    try:
                        if velocity == 'd':
                            velocity = VEL  # Reset to default
                        velocity = int(
                            velocity
                        )  # check that user value is an integer, if not ValueError will be raised
                        if velocity not in range(1, 2048):
                            raise ValueError  # value is not within allowable limits of controller
                    except ValueError:
                        print("Velocity must be between 1 (min) and 2047 (max)")
                        continue

                    t.motor_velocity = velocity
                    turntable_command(t,ser_lock).set_velocity_val(velocity)

                elif config in ('S', 's'):
                    def_speed = MOT_MED_SPEED  # sec/rev, make this a defined constant
                    speed = input(
                        '\t\tRotation speed (second per rotation), d = default({})\n\t\t>>'.
                        format(def_speed))
                    try:
                        if speed == 'd':
                            speed = def_speed  # Reset to default
                        speed = float( speed)  # check we can conver the input to a float value, if not ValueError will be raised
                        if not ( .1 <= speed <= 3000000):  # set some sensible limits, 40 to 80000 sec/rot, otherwise raise error
                            raise ValueError
                    except ValueError:
                        print("Rotation speed must be between {} (min) and {} (max) sec/rot".  format(.1, 3000000))
                        getch()
                        continue

                    turntable_command(t,ser_lock).stop()
                    turntable_command(t,ser_lock).set_rot_rate(speed)

                elif config in ('D', 'd'):
                    def_speed = 60.0  # deg/min, make this a defined ratio of sec/rev constant
                    speed = input(
                        '\t\tRotation speed (degrees per min), d = default({})\n\t\t>>'.
                        format(def_speed))
                    try:
                        if speed == 'd':
                            speed = def_speed  # Reset to default
                        speed = float( speed)  # check we can conver the input to a float value, if not ValueError will be raised
                        if not ( .01 <= speed <= 500 ):  # set some sensible limits, 0.1 to 500 sec/rot, otherwise raise error
                            raise ValueError
                    except ValueError:
                        print("Rotation speed must be between {} (min) and {} (max) deg/min".  format(0.01, 500))
                        getch()
                        continue

                    turntable_command(t,ser_lock).stop()
                    turntable_command(t,ser_lock).set_rot_rate(60*360/speed) # convert from deg/min

                elif config in ('R', 'r'):
                    turntable_command(t,ser_lock).stop()
                    turntable_command(t,ser_lock).reverse()

            else:
                #Unknown command
                pass

            if not __debug__: poll_serial(ser2, t, ser_lock)  # start polling serial port again

            display_menu(t, ser_lock)  # Start the UI again

#########################

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        global_t.cancel()
        if not __debug__:
            global_t2.cancel()
        print("Cleaning up and exiting")
    pass
