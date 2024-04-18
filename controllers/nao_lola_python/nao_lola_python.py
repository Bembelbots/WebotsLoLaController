from controller import Robot, Keyboard, AnsiCodes

import os
import socket
import argparse
import umsgpack
from imageserver import ImageServer, CamIndex

class Nao (Robot):
    SOCK_PATH = "/tmp/robocup"
    TCP_BASE_PORT = 10000
    DOF = 25
    PHALANX_MAX = 8
    ACTUATOR_PKT_SIZE = 786
    MSGPACK_READ_LENGTH = 896

    sensors = {
        "Stiffness": [ 1.0 ] * DOF,
        "Position": [ 0.0 ] * DOF,
        "Temperature": [  0.0 ] * DOF,
        "Current": [  0.0 ] * DOF,
        "Battery": [ 1.0, -32708.0, 0.0, 0.0 ],
        "Accelerometer": [ 0.0, 0.0, 0.0 ],
        "Gyroscope": [ 0.0, 0.0, 0.0 ],
        "Angles": [ 0.0, 0.0 ],
        "Sonar": [ 0.0, 0.0 ],
        "FSR": [  0.0 ] * 8,
        "Status": [ 0 ] * DOF,
        "Touch": [ 0.0 ] * 14,
        "RobotConfig": [ "P0000000000000000000", "6.0.0", "P0000000000000000000", "6.0.0" ],
    }



    def setHands(self, langle, rangle):
        self.setHand(langle, False)
        self.setHand(rangle, True)



    def setHand(self, angle, right=True):
        array = self.rphalanx;
        if not right:
            array = self.lphalanx

        for i in range(self.PHALANX_MAX):
            clampedAngle = angle
            if clampedAngle > self.maxPhalanxMotorPosition[i]:
                clampedAngle = self.maxPhalanxMotorPosition[i]
            elif clampedAngle < self.minPhalanxMotorPosition[i]:
                clampedAngle = self.minPhalanxMotorPosition[i]

            if len(array) > i and array[i] is not None:
                array[i].setPosition(clampedAngle)



    def findAndEnableDevices(self):
        # get the time step of the current world.
        self.timeStep = int(self.getBasicTimeStep())

        self.us = []
        self.fsr = []
        self.leds = {}
        self.lphalanx = []
        self.rphalanx = []
        self.maxPhalanxMotorPosition = []
        self.minPhalanxMotorPosition = []
        self.pos = []
        self.motors = []

        # get devices without spamming warnings in webots 2021
        if self.getDevice('gyro'):
            # webots 2021 (this does not work in 2020)
            self.cameraTop = self.getDevice("CameraTop")
            self.cameraBottom = self.getDevice("CameraBottom")

            self.accelerometer = self.getDevice('accelerometer')
            self.gyro = self.getDevice('gyro')
            self.inertialUnit = self.getDevice('inertial unit')

            for i in ['Sonar/Left', 'Sonar/Right']:
                self.us.append(self.getDevice(i))

            for i in ['LFsr', 'RFsr']:
                self.fsr.append(self.getDevice(i))

            self.lfootlbumper = self.getDevice('LFoot/Bumper/Left')
            self.lfootrbumper = self.getDevice('LFoot/Bumper/Right')
            self.rfootlbumper = self.getDevice('RFoot/Bumper/Left')
            self.rfootrbumper = self.getDevice('RFoot/Bumper/Right')

            self.leds["Chest"] = self.getDevice('ChestBoard/Led')
            self.leds["REar"] = self.getDevice('Ears/Led/Right')
            self.leds["LEar"] = self.getDevice('Ears/Led/Left')
            self.leds["LEye"] = self.getDevice('Face/Led/Left')
            self.leds["REye"] = self.getDevice('Face/Led/Right')
            self.leds["LFoot"] = self.getDevice('LFoot/Led')
            self.leds["RFoot"] = self.getDevice('RFoot/Led')

            # finger motors
            for i in range(self.PHALANX_MAX):
                self.lphalanx.append(self.getDevice("LPhalanx%d" % (i + 1)))
                self.rphalanx.append(self.getDevice("RPhalanx%d" % (i + 1)))

                # assume right and left hands have the same motor position bounds
                self.maxPhalanxMotorPosition.append(self.rphalanx[i].getMaxPosition())
                self.minPhalanxMotorPosition.append(self.rphalanx[i].getMinPosition())

            # get motors
            for j in [ "HeadYaw", "HeadPitch", "LShoulderPitch", "LShoulderRoll",
                      "LElbowYaw", "LElbowRoll", "LWristYaw", "LHipYawPitch", "LHipRoll",
                      "LHipPitch", "LKneePitch", "LAnklePitch", "LAnkleRoll", "RHipRoll",
                      "RHipPitch", "RKneePitch", "RAnklePitch", "RAnkleRoll", "RShoulderPitch",
                      "RShoulderRoll", "RElbowYaw", "RElbowRoll", "RWristYaw", "LPhalanx1", "RPhalanx1", "RHipYawPitch" ]:
                self.pos.append(self.getDevice(j+"S"))
                self.motors.append(self.getDevice(j))
        else:
            # webots 2020 compability
            self.cameraTop = self.getCamera("CameraTop")
            self.cameraBottom = self.getCamera("CameraBottom")

            self.accelerometer = self.getAccelerometer('accelerometer')
            self.gyro = self.getGyro('gyro')
            self.inertialUnit = self.getInertialUnit('inertial unit')

            for i in ['Sonar/Left', 'Sonar/Right']:
                self.us.append(self.getDistanceSensor(i))

            for i in ['LFsr', 'RFsr']:
                self.fsr.append(self.getTouchSensor(i))

            self.lfootlbumper = self.getTouchSensor('LFoot/Bumper/Left')
            self.lfootrbumper = self.getTouchSensor('LFoot/Bumper/Right')
            self.rfootlbumper = self.getTouchSensor('RFoot/Bumper/Left')
            self.rfootrbumper = self.getTouchSensor('RFoot/Bumper/Right')

            # there are 7 controlable LED groups in Webots
            self.leds["Chest"] = self.getLED('ChestBoard/Led')
            self.leds["REar"] = self.getLED('Ears/Led/Right')
            self.leds["LEar"] = self.getLED('Ears/Led/Left')
            self.leds["LEye"] = self.getLED('Face/Led/Left')
            self.leds["REye"] = self.getLED('Face/Led/Right')
            self.leds["LFoot"] = self.getLED('LFoot/Led')
            self.leds["RFoot"] = self.getLED('RFoot/Led')

            # finger motors
            for i in range(self.PHALANX_MAX):
                self.lphalanx.append(self.getDevice("LPhalanx%d" % (i + 1)))
                self.rphalanx.append(self.getDevice("RPhalanx%d" % (i + 1)))

                # assume right and left hands have the same motor position bounds
                self.maxPhalanxMotorPosition.append(self.rphalanx[i].getMaxPosition())
                self.minPhalanxMotorPosition.append(self.rphalanx[i].getMinPosition())

            # get motors
            for j in [ "HeadYaw", "HeadPitch", "LShoulderPitch", "LShoulderRoll",
                      "LElbowYaw", "LElbowRoll", "LWristYaw", "LHipYawPitch", "LHipRoll",
                      "LHipPitch", "LKneePitch", "LAnklePitch", "LAnkleRoll", "RHipRoll",
                      "RHipPitch", "RKneePitch", "RAnklePitch", "RAnkleRoll", "RShoulderPitch",
                      "RShoulderRoll", "RElbowYaw", "RElbowRoll", "RWristYaw", "LPhalanx1", "RPhalanx1", "RHipYawPitch" ]:
                self.pos.append(self.getPositionSensor(j+"S"))
                self.motors.append(self.getDevice(j))


        # enable devices
        if self.args.camera:
            self.cameraTop.enable(self.frametime)
            self.cameraBottom.enable(self.frametime)

        self.accelerometer.enable(self.timeStep)
        self.gyro.enable(self.timeStep)
        self.inertialUnit.enable(self.timeStep)

        for s in self.us:
            s.enable(self.timeStep)

        for f in self.fsr:
            f.enable(self.timeStep)

        self.lfootlbumper.enable(self.timeStep)
        self.lfootrbumper.enable(self.timeStep)
        self.rfootlbumper.enable(self.timeStep)
        self.rfootrbumper.enable(self.timeStep)

        for s in self.pos:
            s.enable(self.timeStep)

        self.key = -1
        self.keyboard = self.getKeyboard()
        self.keyboard.enable(10 * self.timeStep)



    def updateSensors(self):
        # increase tick, taking care it will not overflow
        # the 16 bit unsigned image timestamp
        self.tick = (self.tick + 1) % 2**16

        # update ticks (by abusing battery temperature field)
        if self.args.send_ticks:
            self.sensors["Battery"][3] = self.tick * 1.0  # convert to float

        # IMU
        a = self.accelerometer.getValues()
        self.sensors["Accelerometer"] = [ -a[0], a[1], a[2] ]
        g = self.gyro.getValues()
        self.sensors["Gyroscope"] = [ g[0], g[1], -g[2] ]
        imu = self.inertialUnit.getRollPitchYaw()
        self.sensors["Angles"] = [ imu[0], imu[1] ]

        # motors
        for i in range(self.DOF):
            self.sensors["Position"][i] = self.pos[i].getValue()

        # sonar
        self.sensors["Sonar"] = [self.us[0].getValue(), self.us[1].getValue()]

        # foot bumpers
        self.sensors["Touch"][4] = self.lfootlbumper.getValue()
        self.sensors["Touch"][5] = self.lfootrbumper.getValue()
        self.sensors["Touch"][9] = self.rfootlbumper.getValue()
        self.sensors["Touch"][10] = self.rfootrbumper.getValue()

        # FSR
        # conversion is stolen from webots nao_demo_python controller
        fsv = [self.fsr[0].getValues(), self.fsr[1].getValues()]

        a = []
        a.append(fsv[0][2] / 3.4 + 1.5 * fsv[0][0] + 1.15 * fsv[0][1])  # Left Foot Front Left
        a.append(fsv[0][2] / 3.4 + 1.5 * fsv[0][0] - 1.15 * fsv[0][1])  # Left Foot Front Right
        a.append(fsv[0][2] / 3.4 - 1.5 * fsv[0][0] - 1.15 * fsv[0][1])  # Left Foot Rear Right
        a.append(fsv[0][2] / 3.4 - 1.5 * fsv[0][0] + 1.15 * fsv[0][1])  # Left Foot Rear Left

        a.append(fsv[1][2] / 3.4 + 1.5 * fsv[1][0] + 1.15 * fsv[1][1])  # Right Foot Front Left
        a.append(fsv[1][2] / 3.4 + 1.5 * fsv[1][0] - 1.15 * fsv[1][1])  # Right Foot Front Right
        a.append(fsv[1][2] / 3.4 - 1.5 * fsv[1][0] - 1.15 * fsv[1][1])  # Right Foot Rear Right
        a.append(fsv[1][2] / 3.4 - 1.5 * fsv[1][0] + 1.15 * fsv[1][1])  # Right Foot Rear Left
        for i in range(len(a)):
            self.sensors["FSR"][i] = max(0.0, a[i]/25.0) # fix scaling of values

        # set chest button if C key is pressed
        if self.key == ord('C'):
            if self.sensors["Touch"][0] < 1:
                print(AnsiCodes.CYAN_FOREGROUND + "Chest button pressed." + AnsiCodes.RESET)
            self.sensors["Touch"][0] = 1.0
        elif self.key == ord('U'):
            if self.sensors["Touch"][2] < 1:
                print(AnsiCodes.CYAN_FOREGROUND + "Head touched." + AnsiCodes.RESET)
            for i in range(1,4):
                self.sensors["Touch"][i] = 1.0
        elif self.key == ord('A'):
            if self.sensors["Touch"][1] < 1:
                print(AnsiCodes.CYAN_FOREGROUND + "Front head & chest button pressed." + AnsiCodes.RESET)
            self.sensors["Touch"][0] = 1.0
            self.sensors["Touch"][1] = 1.0
        else:
            if self.sensors["Touch"][0] > 0:
                print(AnsiCodes.CYAN_FOREGROUND + "Chest button released." + AnsiCodes.RESET)
            elif self.sensors["Touch"][2] > 0:
                print(AnsiCodes.CYAN_FOREGROUND + "Head released." + AnsiCodes.RESET)
            elif self.sensors["Touch"][1] > 0:
                print(AnsiCodes.CYAN_FOREGROUND + "Front head & chest button released." + AnsiCodes.RESET)
            for i in range(0,4):
                self.sensors["Touch"][i] = 0.0



    def led_array2RGB(self, a):
        for i in range(len(a)):
            a[i] = max(0.0, min(a[i], 1.0))
        return int(a[0]*255)<<16 | int(a[1]*255)<<8 | int(a[2]*255)



    def updateActuators(self, actuators):
        # motors
        if b"Position" in actuators:
            for i in range(self.DOF-2):
                self.motors[i].setPosition(actuators[b"Position"][i])
            # set hands, since they have more than 1 actuator each in webots
            self.setHands(actuators[b"Position"][23], actuators[b"Position"][24])
            # set RHipYawPitch which does not exists in LoLa packet
            self.motors[25].setPosition(actuators[b"Position"][7])

        # LEDs
        if b"Chest" in actuators:
            self.leds["Chest"].set(self.led_array2RGB(actuators[b"Chest"]))
        if b"LFoot" in actuators:
            self.leds["LFoot"].set(self.led_array2RGB(actuators[b"LFoot"]))
        if b"RFoot" in actuators:
            self.leds["RFoot"].set(self.led_array2RGB(actuators[b"RFoot"]))

        # webots model has only one LED per eye, so just use the first one
        if b"LEye" in actuators:
            v = actuators[b"LEye"]
            self.leds["LEye"].set(self.led_array2RGB([v[0], v[8], v[16]]))
        if b"REye" in actuators:
            v = actuators[b"REye"]
            self.leds["REye"].set(self.led_array2RGB([v[0], v[8], v[16]]))

        # webots model has only one LED per ear, so just use the first one
        if b"LEar" in actuators:
            self.leds["LEar"].set(self.led_array2RGB([0, 0, actuators[b"LEar"][0]]))
        if b"REar" in actuators:
            self.leds["REar"].set(self.led_array2RGB([0, 0, actuators[b"REar"][0]]))



    def format_addr(self, addr):
        if len(addr) == 2:
            return "{}:{}".format(addr[0], addr[1])
        else:
            return "UNIX:{}".format(addr)



    def parse_tcp_addr(self, addr):
        x = addr.split(":")
        if len(x) == 2:
            return (x[0], int(x[1]))
        else:
            return ("localhost", int(x[0]))



    def parse_args(self):
        # parse commandline arguments
        op = argparse.ArgumentParser()
        op.add_argument("--no-camera", action="store_false",
                        dest="camera", default=True,
                        help="Disable Webots cameras and image server")
        op.add_argument("--tcp", action="store_true",
                        dest="tcp", default=False,
                        help="Listen on TCP instead of UNIX socket for LoLa communication")
        op.add_argument("--listen=", action="store",
                        dest="lola_addr", default="", metavar="ADDR",
                        help="Path for LoLa UNIX socket or address on which TCP server will listen on (\"<host>:<port>\" or \"<port>\")")
        op.add_argument("--tcam-listen=", action="store",
                        dest="tcam_addr", default="", metavar="ADDR",
                        help="Listen address for top camera image server (\"<host>:<port>\" or \"<port>\")")
        op.add_argument("--bcam-listen=", action="store",
                        dest="bcam_addr", default="", metavar="ADDR",
                        help="Listen address for bottom camera image server (\"<host>:<port>\" or \"<port>\")")
        op.add_argument("--framerate=", action="store",
                        dest="fps", default=30, type=int,
                        help="simulated camera framerate (default: 30)")
        op.add_argument("--no-ticks", action="store_false",
                        dest="send_ticks", default=True,
                        help="Do not increase the battery temperature with with each simulations step")
        args = op.parse_args()

        self.frametime = 1000 // args.fps

        # set lola listen address
        if args.tcp:
            if not args.lola_addr:
                # set default listen address
                args.lola_addr = ("localhost", self.TCP_BASE_PORT)
            else:
                args.lola_addr = self.parse_tcp_addr(args.lola_addr)
        else:
            if not args.lola_addr:
                # socket path to default
                args.lola_addr = self.SOCK_PATH

        # set top cam listen address
        if not args.tcam_addr:
            if args.tcp:
                a = args.lola_addr
                args.tcam_addr = (a[0], a[1]+1)
            else:
                args.tcam_addr = ("localhost", self.TCP_BASE_PORT + 1)
        else:
            args.tcam_addr = self.parse_tcp_addr(args.tcam_addr)

        # set bottom cam listen address
        if not args.bcam_addr:
            a = args.tcam_addr
            args.bcam_addr = (a[0], a[1]+1)
        else:
            args.bcam_addr = self.parse_tcp_addr(args.bcam_addr)

        self.args = args



    def print_status(self):
        print(AnsiCodes.YELLOW_FOREGROUND)
        print("Webots LoLa controller listening on:")
        print("    LoLa:          ", self.format_addr(self.args.lola_addr))
        if self.args.camera:
            print("    Top Camera:    ", self.format_addr(self.args.tcam_addr))
            print("    Bottom Camera: ", self.format_addr(self.args.bcam_addr))
        print("Options:  Camera={}  Framerate={}  SendTicks={}".format(self.args.camera, self.args.fps, self.args.send_ticks))
        print(" ")
        print("Button interface (3D view must have focus):")
        print("     Press 'C' to simulate chest button")
        print("     Press 'U' to simulate unstiff gesture (all head sensors touched)")
        print("     Press 'A' to simulate calibration gesture (front head sensor touched & chest button pressed)")
        print(AnsiCodes.RESET + " ")



    def __init__(self):
        Robot.__init__(self)

        self.tick = 0

        self.parse_args();

        # initialize stuff
        self.findAndEnableDevices()

        if self.args.camera:
            # initialize top image sender
            w = self.cameraTop.getWidth()
            h = self.cameraTop.getHeight()
            self.topImageServer = ImageServer(self.args.tcam_addr, w, h, CamIndex.TOP)

            # initialize bottom image sender
            w = self.cameraBottom.getWidth()
            h = self.cameraBottom.getHeight()
            self.bottomImageServer = ImageServer(self.args.bcam_addr, w, h, CamIndex.BOTTOM)



    def run(self):
        try:
            os.unlink(self.SOCK_PATH)
        except OSError:
            if os.path.exists(self.SOCK_PATH):
                raise

        if self.args.tcp:
            sock_type = socket.AF_INET
        else:
            sock_type = socket.AF_UNIX

        sock = socket.socket(sock_type, socket.SOCK_STREAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind(self.args.lola_addr)
        sock.listen()
        sock.settimeout((self.timeStep-1)/1000.0)

        self.print_status()

        conn = None
        imgCounter = self.frametime

        while True:
            if self.stepBegin(self.timeStep) == -1:
                break

            self.key = self.keyboard.getKey()

            if conn:
                self.updateSensors()
                try:
                    # send sensor data to LoLa client
                    packed = umsgpack.packb(self.sensors, force_float_precision="single")
                    if len(packed) != self.MSGPACK_READ_LENGTH:
                        print(AnsiCodes.RED_FOREGROUND + "Msgpack packet size doesn't match LoLA specifications."  + AnsiCodes.RESET)
                    conn.send(packed)
                    data = conn.recv(self.ACTUATOR_PKT_SIZE*3)
                    if data:
                        self.updateActuators(umsgpack.unpackb(data))
                except TimeoutError:
                    print(AnsiCodes.RED_FOREGROUND + "Timeout while waiting for LoLa actuators." + AnsiCodes.RESET)
                except ConnectionError:
                    conn.close()
                    conn = None
                    print(AnsiCodes.RED_FOREGROUND + "LoLa client disconnected." + AnsiCodes.RESET)

                # send images on average every self.frametime milliseconds in simulation time
                imgCounter -= self.timeStep
                if self.args.camera and imgCounter <= 0:
                    self.topImageServer.send(self.tick, self.cameraTop.getImage())
                    self.bottomImageServer.send(self.tick, self.cameraBottom.getImage())
                    imgCounter += self.frametime
            else:
                try:
                    (conn, addr) = sock.accept()
                    packed = umsgpack.packb(self.sensors, force_float_precision="single")
                    if len(packed) != self.MSGPACK_READ_LENGTH:
                        print(AnsiCodes.RED_FOREGROUND + "Msgpack packet size doesn't match LoLA specifications."  + AnsiCodes.RESET)
                    conn.send(packed)
                    print(AnsiCodes.GREEN_FOREGROUND + "LoLa client connected." + AnsiCodes.RESET)
                except:
                    conn = None
                    self.stepEnd()
                    continue

            if self.stepEnd() == -1:
                break

        # remove socket on exit
        os.unlink(self.SOCK_PATH)

        # stop image threads
        if self.args.camera:
            self.topImageServer.stop()
            self.bottomImageServer.stop()

# Enable Msgpack Old Specification Compatibility Mode, which is used in LoLA
umsgpack.compatibility = True

# create the Robot instance and run main loop
robot = Nao()
robot.run()
