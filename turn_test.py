#!/usr/bin/env pybricks-micropython

#---------------------------------------------------------------------------
# BNO055 DATASHEET 89page : 4.5 Digital Interface 참조
# BNO055 DATASHEET 93page : 4.7 UART Protocol 참조
# BNO055 DATASHEET 20page : 3.3 Operation Modes 참조
# BNO055 DATASHEET  6page : 목차 어드레스 번지 참조
# BNO055 DATASHEET 98page : 5.3 Connection diagram UART 참조
#---------------------------------------------------------------------------
import struct

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile
from ucollections import namedtuple
from pybricks.iodevices import UARTDevice

# --------------------------------- 장치설정 --------------------------------- #
ev3 = EV3Brick()
lm = Motor(Port.C)
rm = Motor(Port.B)
# ul = UltrasonicSensor(Port.S2)
robot = DriveBase(lm, rm, wheel_diameter=73.21, axle_track = 257)
ev3.speaker.beep()
gyro = GyroSensor(Port.S1)
TOFData = namedtuple("TOFData", ["condition", "t1", "t2", "t3", "t4"])
ser = UARTDevice(Port.S3, 115200)
ser.clear()

null_tof = TOFData(False, 0, 0, 0, 0)

# ----------------------------------- CAM ----------------------------------- #

gyro.reset_angle(0)


def readPacket() -> bool:
    global PacketIndex
    global ObjectIndex
    global rcvPACKET
    global ObjectCount
    
    read = False

    rcvPACKET = bytearray([])
    rcvPACKET += ser.read()
    PacketIndex = 0
    if rcvPACKET[0] == 0xAA:
        PacketIndex = 1
        rcvPACKET += ser.read()
        if rcvPACKET[1] == 0xCC:
            PacketIndex = 2
            rcvPACKET += ser.read(10)

            rcvPACKET += ser.read() #CHECKSUM

            chksum = 0
            for r in rcvPACKET:     #CHECKSUM 계산
                chksum ^= r
                
            if chksum != 0:         #CHECKSUM ERROR
                print("CHECK SUM ERROR")
                return False

            return True
    return False
            
def getTOF():
    read = True
    if ser.waiting() >= 5:  # 수신 데이터가 있으면 (최소 사이즈는 5 바이트임)
        wait(1) 
        if readPacket():    # 데이터 수신에 성공했으면 Parsing 함
            tof1 = struct.unpack("<H",rcvPACKET[4:6])[0]
            tof2 = struct.unpack("<H",rcvPACKET[6:8])[0]
            tof3 = struct.unpack("<H",rcvPACKET[8:10])[0]
            tof4 = struct.unpack("<H",rcvPACKET[10:12])[0]

            # print("OK")
            # print("TOF1  : ",tof1)
            # print("TOF2  : ",tof2)
            # print("TOF3  : ",tof3)
            # print("TOF4  : ",tof4)
            # print(rcvPACKET, ser.read_all())
            # if tof1 < 50:
            #     print("TOF1  : ",tof1)
            # if tof2 < 50:
            #     print("TOF2  : ",tof2)
            return TOFData(True, tof1, tof2, tof3, tof4)
        else: return null_tof
    else:
        return null_tof

# ----------------------------------- 함수 ----------------------------------- #
yaw = 0
    
def imu_turn(speed, angle):
    wait(100)
    now_yaw = gyro.angle()
    print('now_yaw: ', now_yaw)
    yaw = now_yaw + angle
    print('yaw: ', yaw)

    # print(diff)
    robot.drive(speed, speed)



# ----------------------------------- main ----------------------------------- #
lastYaw = 0
def tof_pid_control(speed, gain, yaw, kp, kd):
    # 전역변수를 지역변수로
    global lastYaw; global max_error
    # 라인트레이싱 - PID control
    differential = yaw - lastYaw
    lastYaw = yaw
    # 적분 상수를 0으로 입력할 경우
    steering = (kp * yaw + kd * differential) * gain
    
    robot.drive(-speed, steering)

def gyro_check(a = 0):
    angle = gyro.angle()
    return -5 + a < angle and angle < 5 - a


def gTOF():
    while True:
        tof = getTOF()
        if tof.condition:
            break
    return tof



while True:
    print(gTOF())


gyro.reset_angle(0)
while not Button.CENTER in ev3.buttons.pressed():
    pass






while True:
    tof = getTOF()
    if not tof.condition:
        continue
    print(tof, gyro.angle())
    if abs(tof.t1 - tof.t2) < 5 and gyro_check() or tof.t2 < 65 and gyro.angle() < -10:
        robot.stop(Stop.BRAKE)
        break
    yaw = 53 - tof.t1
    tof_pid_control(190, 1, yaw, 1, 0)

if not gyro_check():
    print(11111)
    robot.stop()
    while True:
        tof = getTOF()
        if not tof.condition:
            continue
        print(tof, gyro.angle())
        if abs(tof.t1 - tof.t2) < 5 and gyro_check(3):
            break
        
        lm.run(-75)
        rm.run(30)
    lm.stop()
    rm.stop()