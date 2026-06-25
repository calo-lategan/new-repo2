import sys
import threading
from ros_robot_controller.ros_robot_controller_sdk import Board

board = Board()
board.enable_reception()

def getServoID(servo_id=254, count=200):
    ret = board.bus_servo_read_id(servo_id)
    if ret is not None:
        return ret[0]

def getServoPulse(servo_id, count=200):
    ret = board.bus_servo_read_position(servo_id)
    if ret is not None:
        return ret[0]

def getServoVin(servo_id, count=200):
    ret = board.bus_servo_read_vin(servo_id)
    if ret is not None:
        return ret[0]

def getServoTemp(servo_id, count=200):
    ret = board.bus_servo_read_temp(servo_id)
    if ret is not None:
        return ret[0]

def getServoDeviation(servo_id, count=200):
    ret = board.bus_servo_read_offset(servo_id)
    if ret is not None:
        return ret[0]

def getServoTempLimit(servo_id, count=200):
    ret = board.bus_servo_read_temp_limit(servo_id)
    if ret is not None:
        return ret[0]

def getServoAngleLimit(servo_id, count=200):
    ret = board.bus_servo_read_angle_limit(servo_id)
    print("angle_lmit", ret)
    if ret is not None:
        return ret

def getServoVinLimit(servo_id, count=200):
    ret = board.bus_servo_read_vin_limit(servo_id)
    if ret is not None:
        return ret

def setServoID(old_id, new_servo_id):
    board.bus_servo_set_id(old_id, new_servo_id)

def setBusServoPulse(servo_id, pulse, use_time):
    board.bus_servo_set_position(use_time/1000.0, ((servo_id, pulse),))

def setServoPulse(servo_id, pulse, use_time):
    board.bus_servo_set_position(use_time/1000.0, ((servo_id, pulse),))

def setServoDeviation(servo_id ,dev):
    board.bus_servo_set_offset(servo_id, dev)
    
def saveServoDeviation(servo_id):
    board.bus_servo_save_offset(servo_id)
    
def setServoMaxTemp(servo_id, temp):
    board.bus_servo_set_temp_limit(servo_id, temp)
  
def setServoVinLimit(servo_id, vin_min, vin_max):
    board.bus_servo_set_vin_limit(servo_id, [vin_min, vin_max])
  
def setServoAngleLimit(servo_id, pulse_min, pulse_max):
    board.bus_servo_set_angle_limit(servo_id, [pulse_min, pulse_max])

