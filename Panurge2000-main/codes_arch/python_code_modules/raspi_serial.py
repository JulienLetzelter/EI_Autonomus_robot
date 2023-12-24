#########################################################################
# CENTRALESUPELEC : ST5 53 integration week
#
# Basic human-machine interface to test robots
#
#########################################################################
# Authors : Erwan Libessart
# Modifications by Morgan Roger
# TODO : translate to English where needed
#########################################################################

from __future__ import division, print_function

import logging
import signal
import time
import numpy as np
from time import sleep
import struct

try:
    import queue
except ImportError:
    import Queue as queue

from robust_serial import write_order, Order, write_i8, write_i16, read_i16, read_i32, read_i8
from robust_serial.utils import open_serial_port
from constants import BAUDRATE
from picamera import PiCamera
from picamera.array import PiRGBArray

# return angle takes an array frame (conversion needed before) and returns the correction angle
from line_detection import return_angle
import cv2

emptyException = queue.Empty
fullException = queue.Full
serial_file = None
motor_speed = 90
step_length = 5

def main():
    test_camera()
    connect_to_arduino()

    print("Welcome to raspi_serial.py")
    print("Press enter to validate your commands")
    print("Enter h to get the list of valid commands")
    cmd_str = ''
    while cmd_str != 'q':
        cmd_str = input("Enter your command: ")
        # cmd_str =
        process_cmd(cmd_str)

    camera.close()


def test_camera():
    global camera
    camera.start_preview()
    sleep(2)
    my_file = open('test_photo.jpg', 'wb')
    camera.capture(my_file)
    # At this point my_file.flush() has been called, but the file has
    # not yet been closed
    my_file.close()
    camera.stop_preview()


def connect_to_arduino():
    global serial_file
    try:
        # Open serial port (for communication with Arduino)
        serial_file = open_serial_port(baudrate=BAUDRATE)
    except Exception as e:
        print('exception')
        raise e

    is_connected = False
    # Initialize communication with Arduino
    while not is_connected:
        print("Trying connection to Arduino...")
        write_order(serial_file, Order.HELLO)
        bytes_array = bytearray(serial_file.read(1))
        if not bytes_array:
            time.sleep(2)
            continue
        byte = bytes_array[0]
        if byte in [Order.HELLO.value, Order.ALREADY_CONNECTED.value]:
            is_connected = True

    time.sleep(2)
    c = 1
    while (c != b''):
        c = serial_file.read(1)


def process_cmd(cmd):
    global motor_speed
    global step_length
    cmd_type = {
        "[q]uit": (cmd == 'q'),
        "[h]elp": (cmd == 'h'),
        "[e]ncoder values": (cmd == 'e'),
        "[z]ero setting encoders": (cmd == 'z'),
        "(%) set motor speed percentage": (cmd.isdigit()),
        "[f]orward step": (cmd == 'f'),
        "[l] left step": (cmd == 'l'),
        "[r] right step": (cmd == 'r'),
        "[b]ackward step": (cmd == 'b'),
        "[lb] left step back": (cmd == 'lb'),
        "[rb] right step back": (cmd == 'rb'),
        "[ff]orward": (cmd == 'ff'),
        "[bb]ackward": (cmd == 'bb'),
        "[tl] turn left": (cmd == 'tl'),
        "[tr] turn right": (cmd == 'tr'),
        "[p]ause motors": (cmd == 'p'),
        "[s]ervo move": (cmd == 's'),
        "[t]urn of a given angle": (cmd == 't')
    }

    if cmd_type["[q]uit"]:
        print("Goodbye...")
    elif cmd_type["[h]elp"]:
        for key in cmd_type.keys():
            print(key)
    elif cmd_type["[e]ncoder values"]:
        print('left encoder : ', lectureCodeurGauche())
        print('right encoder : ', lectureCodeurDroit())
    elif cmd_type["[z]ero setting encoders"]:
        print("Resetting encoders...")
        write_order(serial_file, Order.RESETENC)
        print('left encoder : ', lectureCodeurGauche())
        print('right encoder : ', lectureCodeurDroit())
    elif cmd_type["(%) set motor speed percentage"]:
        motor_speed = int(cmd)
        print("Speed set to " + cmd + "%")
    elif cmd_type["[f]orward step"]:
        print("Moving forward at " + str(motor_speed) + "%...")
        write_order(serial_file, Order.MOTOR)
        write_i8(serial_file, motor_speed)  # valeur moteur droit
        write_i8(serial_file, motor_speed)  # valeur moteur gauche
        time.sleep(step_length)
        print('stop motors')
        write_order(serial_file, Order.STOP)
    elif cmd_type["[l] left step"]:
        print("Forward left at " + str(motor_speed) + "%...")
        write_order(serial_file, Order.MOTOR)
        write_i8(serial_file, 0)  # valeur moteur droit
        write_i8(serial_file, motor_speed)  # valeur moteur gauche
        time.sleep(step_length)
        print('stop motors')
        write_order(serial_file, Order.STOP)
    elif cmd_type["[r] right step"]:
        print("Forward right at " + str(motor_speed) + "%...")
        write_order(serial_file, Order.MOTOR)
        write_i8(serial_file, motor_speed)  # valeur moteur droit
        write_i8(serial_file, 0)  # valeur moteur gauche
        time.sleep(step_length)
        print('stop motors')
        write_order(serial_file, Order.STOP)
    elif cmd_type["[b]ackward step"]:
        print("Moving backward at " + str(motor_speed) + "%...")
        write_order(serial_file, Order.MOTOR)
        write_i8(serial_file, -motor_speed)  # valeur moteur droit
        write_i8(serial_file, -motor_speed)  # valeur moteur gauche
        time.sleep(step_length)
        print('stop motors')
        write_order(serial_file, Order.STOP)
    elif cmd_type["[lb] left step back"]:
        print("Backward left at " + str(motor_speed) + "%...")
        write_order(serial_file, Order.MOTOR)
        write_i8(serial_file, 0)  # valeur moteur droit
        write_i8(serial_file, -motor_speed)  # valeur moteur gauche
        time.sleep(step_length)
        print('stop motors')
        write_order(serial_file, Order.STOP)
    elif cmd_type["[rb] right step back"]:
        print("Backward right at " + str(motor_speed) + "%...")
        write_order(serial_file, Order.MOTOR)
        write_i8(serial_file, -motor_speed)  # valeur moteur droit
        write_i8(serial_file, 0)  # valeur moteur gauche
        time.sleep(step_length)
        print('stop motors')
        write_order(serial_file, Order.STOP)
    elif cmd_type["[ff]orward"]:
        print("Moving forward at " + str(motor_speed) + "%...")
        write_order(serial_file, Order.MOTOR)
        write_i8(serial_file, motor_speed)  # valeur moteur droit
        write_i8(serial_file, motor_speed)  # valeur moteur gauche
    elif cmd_type["[bb]ackward"]:
        print("Moving backward at " + str(motor_speed) + "%...")
        write_order(serial_file, Order.MOTOR)
        write_i8(serial_file, -motor_speed)  # valeur moteur droit
        write_i8(serial_file, -motor_speed)  # valeur moteur gauche
    elif cmd_type["[tl] turn left"]:
        print("Turn left at " + str(motor_speed) + "%...")
        write_order(serial_file, Order.MOTOR)
        write_i8(serial_file, motor_speed)  # valeur moteur droit
        write_i8(serial_file, -motor_speed)  # valeur moteur gauche
        time.sleep(1)
        print('stop motors')
        write_order(serial_file, Order.STOP)
    elif cmd_type["[tr] turn right"]:
        print("Turn right at " + str(motor_speed) + "%...")
        write_order(serial_file, Order.MOTOR)
        write_i8(serial_file, -motor_speed)  # valeur moteur droit
        write_i8(serial_file, motor_speed)  # valeur moteur gauche
        time.sleep(1)
        print('stop motors')
        write_order(serial_file, Order.STOP)
    elif cmd_type["[p]ause motors"]:
        print("Stopping...")
        write_order(serial_file, Order.STOP)
    elif cmd_type["[s]ervo move"]:
        print("Moving front servo...")
        write_order(serial_file, Order.SERVO)
        write_i16(serial_file, 45)  # valeur angle servo
        time.sleep(2)
        write_order(serial_file, Order.SERVO)
        write_i16(serial_file, 90)  # valeur angle servo
    elif cmd_type["[t]urn of a given angle"]:
        angle = int(input("quel angle? "))
        tourner(np.sign(angle)*(abs(angle)*192/810 - 1080/810))
    else:
        print("Invalid command")

def tourner(angle):
    print("début tourner")
    write_order(serial_file, Order.RESETENC)
    write_order(serial_file, Order.STOP)
    if angle > 0:
        write_order(serial_file, Order.MOTOR)
        write_i8(serial_file, 0)  # valeur moteur droit
        write_i8(serial_file, motor_speed)  # valeur moteur gauche
    else:

        write_order(serial_file, Order.MOTOR)
        write_i8(serial_file, motor_speed)  # valeur moteur droit
        write_i8(serial_file, 0)  # valeur moteur gauche
    Flag = False
    
    
    while not Flag:
        
        # Get new frame
        rawcapture.truncate(0)
        camera.capture(rawcapture, use_video_port=True, resize=(80, 60), format="bgr")
        frame = rawcapture.array
        
        # Recalculate angle
        temp_angle, temp_intersec = return_angle(frame)
        temp_angle = np.sign(temp_angle)*(abs(temp_angle)*0.237 - 1.33)
                                     
        #if abs(lectureCodeurGauche()) >= abs(2*angle) and angle >0:
        if 10 > temp_angle > 0:
            #print(lectureCodeurGauche())
            Flag = True
        #if abs(lectureCodeurDroit()) >= abs(2*angle) and angle <0:
        if -10 < temp_angle < 0:
            Flag = True
        if angle == 0:
            Flag = True
        
    write_order(serial_file, Order.STOP)
    write_order(serial_file, Order.MOTOR)
    write_i8(serial_file, motor_speed)  # valeur moteur droit
    write_i8(serial_file, motor_speed)  # valeur moteur gauche


def tourner_sur_place(angle):
    write_order(serial_file, Order.RESETENC)
    write_order(serial_file, Order.STOP)
    if angle > 0:
        write_order(serial_file, Order.MOTOR)
        write_i8(serial_file, -motor_speed)  # valeur moteur droit
        write_i8(serial_file, motor_speed)  # valeur moteur gauche
    else:
        write_order(serial_file, Order.MOTOR)
        write_i8(serial_file, motor_speed)  # valeur moteur droit
        write_i8(serial_file, -motor_speed)  # valeur moteur gauche
    Flag = False
    
    while not Flag:
        if abs(lectureCodeurGauche()) >= abs(angle) and abs(lectureCodeurDroit()) >= abs(angle):
            print('lecture gauche: ', lectureCodeurGauche())
            print('lecture droit: ', lectureCodeurDroit())
            print('angle: ', angle)
            Flag = True
            
    write_order(serial_file, Order.STOP)
    write_order(serial_file, Order.MOTOR)
    write_i8(serial_file, motor_speed)  # valeur moteur droit
    write_i8(serial_file, motor_speed)  # valeur moteur gauche

def lectureCodeurGauche():
    write_order(serial_file, Order.READENCODERl)
    while True:
        try:
            g = read_i16(serial_file)
            break
        except struct.error:
            pass
        except TimeoutError:
            write_order(serial_file, Order.READENCODERl)
            pass
    return g


def lectureCodeurDroit():
    write_order(serial_file, Order.READENCODERr)
    while True:
        try:
            d = read_i16(serial_file)
            break
        except struct.error:
            pass
        except TimeoutError:
            write_order(serial_file, Order.READENCODERr)
            pass
    return d
    
def lecture_distance():
    write_order(serial_file, Order.OBSTACLE)
    while True:
        try:
            d = read_i16(serial_file)
            break
        except struct.error:
            pass
        except TimeoutError:
            write_order(serial_file, Order.OBSTACLE)
            pass
    return d
    

def detect_obstacle():
    distance = lecture_distance()
    return distance < 40

def avancer(camera,rawcapture):
    write_order(serial_file, Order.MOTOR)
    write_i8(serial_file, motor_speed)  # valeur moteur droit
    write_i8(serial_file, motor_speed)  # valeur moteur gauche
    Flag_debut = False
    demitour = False
    stop = False
    compteur = 0
    while True:
        camera.capture(rawcapture, use_video_port=True,
                       resize=(80, 60), format="bgr")
        frame = rawcapture.array
        angle,is_inter = return_angle(frame)
    
        if detect_obstacle():
            print("obstacle détecté")
            if not demitour:
                Flag_debut = False
                tourner_sur_place(180*0.237037 - 1)
                demitour = True
            else:
                stop = True
        else:
            stop = False

        if not stop:
            if is_inter:
                Flag_debut = True
            elif Flag_debut:
                time.sleep(0.35)
                write_order(serial_file, Order.STOP)
                rawcapture.truncate(0)
                return demitour

            tourner(np.sign(angle)*(abs(angle)*0.237 - 1.33))
        else:
            write_order(serial_file, Order.MOTOR)
            write_i8(serial_file, 0)  # valeur moteur droit
            write_i8(serial_file, 0)  # valeur moteur gauche
            
        key = cv2.waitKey(1)
        rawcapture.truncate(0)
        if key == 27:
            write_order(serial_file, Order.STOP)
            break


if __name__ == "__main__":

    camera = PiCamera()
    rawcapture = PiRGBArray(camera, size=(80, 60))
    time.sleep(0.1)