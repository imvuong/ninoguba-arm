#!/usr/bin/env python3

import logging
import os
import math
import msgpack
import paho.mqtt.client as paho
import rpyc
import threading

from ev3dev2.motor import OUTPUT_A, OUTPUT_B, OUTPUT_C, OUTPUT_D, LargeMotor, MediumMotor, MoveTank


logFormatter = logging.Formatter(
    "%(asctime)s.%(msecs)03d [%(threadName)-12.12s] [%(levelname)-5.5s]:  %(message)s",
    datefmt="%Y%m%d%H%M%S",
)
logger = logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)

logPath = "."
logFilename = "output"
fileHandler = logging.FileHandler("{0}/{1}.log".format(logPath, logFilename))
fileHandler.setFormatter(logFormatter)
logger.addHandler(fileHandler)

consoleHandler = logging.StreamHandler()
consoleHandler.setFormatter(logFormatter)
logger.addHandler(consoleHandler)

# Create a RPyC connection to the remote ev3dev device.
# Use the hostname or IP address of the ev3dev device.
# If this fails, verify your IP connectivty via ``ping X.X.X.X``
conn = rpyc.classic.connect("192.168.4.90")  # change this IP address for your slave EV3 brick
remote_motor = conn.modules["ev3dev2.motor"]


class Motors:

    WAIST_MAX = 180
    WAIST_MIN = -180
    WAIST_RATIO = 7.5
    SHOULDER_MAX = -60
    SHOULDER_MIN = 50
    SHOULDER_RATIO = 7.5
    ELBOW_MAX = 90
    ELBOW_MIN = -90
    ELBOW_RATIO = 5
    ROLL_MAX = 180
    ROLL_MIN = -180
    ROLL_RATIO = 7
    PITCH_MAX = 80
    PITCH_MIN = -90
    PITCH_RATIO = 5
    SPIN_MAX = -360
    SPIN_MIN = 360
    SPIN_RATIO = 7
    GRABBER_MAX = -68
    GRABBER_MIN = 0
    GRABBER_RATIO = 24

    def __init__(self):

        self._waist_motor = LargeMotor(OUTPUT_A)
        self._shoulder_control1 = LargeMotor(OUTPUT_B)
        self._shoulder_control2 = LargeMotor(OUTPUT_C)
        self._shoulder_motor = MoveTank(OUTPUT_B, OUTPUT_C)
        self._elbow_motor = LargeMotor(OUTPUT_D)
        self._roll_motor = remote_motor.MediumMotor(remote_motor.OUTPUT_A)
        self._pitch_motor = remote_motor.MediumMotor(remote_motor.OUTPUT_B)
        self._spin_motor = remote_motor.MediumMotor(remote_motor.OUTPUT_C)
        self._grabber_motor = remote_motor.MediumMotor(remote_motor.OUTPUT_D)

        self._waist_speed = 25
        self._shoulder_speed = 20
        self._elbow_speed = 10
        self._roll_speed = 10
        self._pitch_speed = 10
        self._spin_speed = 10
        self._grabber_speed = 10

        self._waist_pos = 0
        self._shoulder_pos = 0
        self._elbow_pos = 0
        self._roll_pos = 0
        self._pitch_pos = 0
        self._spin_pos = 0

    def set_speed(
        self,
        waist_speed: int = 25,
        shoulder_speed: int = 20,
        elbow_speed: int = 10,
        roll_speed: int = 10,
        pitch_speed: int = 10,
        spin_speed: int = 10,
        grabber_speed: int = 10,
    ):
        self._waist_speed = waist_speed
        self._shoulder_speed = shoulder_speed
        self._elbow_speed = elbow_speed
        self._roll_speed = roll_speed
        self._pitch_speed = pitch_speed
        self._spin_speed = spin_speed
        self._grabber_speed = grabber_speed

    def stop_all(self):
        self._waist_motor.stop()
        self._shoulder_motor.stop()
        self._elbow_motor.stop()
        self._roll_motor.stop()
        self._pitch_motor.stop()
        self._spin_motor.stop()
        self._grabber_motor.stop()

    def reset_all_position(self):
        self._waist_motor.position = 0
        self._shoulder_motor.position = 0
        self._elbow_motor.position = 0
        self._roll_motor.position = 0
        self._pitch_motor.position = 0
        self._spin_motor.position = 0
        self._grabber_motor.position = 0

    def move(self, waist_pos, shoulder_pos, elbow_pos, roll_pos, pitch_pos, spin_pos):
        # TODO include grabber_pos and store current pos of each motor
        # TODO validate pos
        if self._waist_pos != waist_pos:
            self._waist_motor.on_to_position(
                self._waist_speed, waist_pos * self.WAIST_RATIO, True, False
            )
            self._waist_pos = waist_pos

        if self._shoulder_pos != shoulder_pos:
            self._shoulder_motor.on_for_degrees(
                self._shoulder_speed,
                self._shoulder_speed,
                shoulder_pos * self.SHOULDER_RATIO,
                True,
                False,
            )
            self._shoulder_pos = shoulder_pos

        if self._elbow_pos != elbow_pos:
            self._elbow_motor.on_to_position(
                self._elbow_speed, elbow_pos * self.ELBOW_RATIO, True, False
            )
            self._elbow_pos = elbow_pos

        if self._roll_pos != roll_pos:
            self._roll_motor.on_to_position(
                self._roll_speed, roll_pos * self.ROLL_RATIO, True, False
            )
            self._roll_pos = roll_pos

        if self._pitch_pos != pitch_pos:
            self._pitch_motor.on_to_position(
                self._pitch_speed, pitch_pos * self.PITCH_RATIO, True, False
            )
            self._pitch_pos = pitch_pos

        if self._spin_pos != spin_pos:
            self._spin_motor.on_to_position(
                self._spin_speed, spin_pos * self.SPIN_RATIO, True, False
            )
            self._spin_pos = spin_pos


class RobotArm(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)
        self._running = False
        self._motors = Motors()
        self._client = paho.Client()
        self._client.on_message = self.on_message
        self._client.on_publish = self.on_publish
        logger.info("mqtt client init!")
        self._client.connect("192.168.4.76", 1883, 60)
        topics = [("joint_states", 0)]
        self._client.subscribe(topics, 0)
        self._waist_pos = 0
        self._shoulder_pos = 0
        self._elbow_pos = 0
        self._roll_pos = 0
        self._pitch_pos = 0
        self._spin_pos = 0

    def convert(self, data):
        if isinstance(data, bytes):
            return data.decode()
        if isinstance(data, dict):
            return dict(map(self.convert, data.items()))
        if isinstance(data, tuple):
            return tuple(map(self.convert, data))
        if isinstance(data, list):
            return list(map(self.convert, data))
        return data

    def on_message(self, mosq, obj, msg):
        unpacked = msgpack.unpackb(msg.payload)
        data = self.convert(unpacked)
        if msg.topic == "joint_states":
            pose = data["position"]
            if pose != 0:
                # logger.info(pose)
                self._waist_pos = round(math.degrees(pose[0]))
                self._shoulder_pos = round(math.degrees(pose[1]))
                self._elbow_pos = round(math.degrees(pose[2]))
                self._roll_pos = round(math.degrees(pose[3]))
                self._pitch_pos = round(math.degrees(pose[4]))
                self._spin_pos = round(math.degrees(pose[5]))

    def on_publish(self, mosq, obj, mid):
        pass

    def run(self):
        logger.info("Robot running!")
        os.system("setfont Lat7-Terminus12x6")
        self._running = True
        while self._running:
            self._motors.move(
                waist_pos=self._waist_pos,
                shoulder_pos=self._shoulder_pos,
                elbow_pos=self._elbow_pos,
                roll_pos=self._roll_pos,
                pitch_pos=self._pitch_pos,
                spin_pos=self._spin_pos,
            )
            self._client.loop()

        self._motors.stop_all()
        logger.info("Robot stopping!")

    def stop(self):
        self._running = False


def main():
    """The main function of our program"""

    # Positve angle should rotate counter clockwise
    # waist_motor.position = 0
    # logger.info("Waist Position: {}".format(waist_motor.position))
    # waist_motor.on_to_position(waist_speed, waist_max * waist_ratio, True, True)
    # logger.info("Waist Position: {}".format(waist_motor.position))
    # waist_motor.on_to_position(waist_speed, -waist_max * waist_ratio, True, True)

    # Negative angle should rotate clockwise
    # waist_motor.position = 0
    # logger.info("Waist Position: {}".format(waist_motor.position))
    # waist_motor.on_to_position(waist_speed, waist_min * waist_ratio, True, True)
    # logger.info("Waist Position: {}".format(waist_motor.position))
    # waist_motor.on_to_position(waist_speed, -waist_min * waist_ratio, True, True)

    # shoulder_motor.position = 0
    # logger.info("Shoulder Position: {}".format(shoulder_motor.position))
    # shoulder_motor.on_for_degrees(
    #     shoulder_speed, shoulder_speed, shoulder_max * shoulder_ratio, True, True
    # )
    # logger.info("Shoulder Position: {}".format(shoulder_motor.position))
    # shoulder_motor.on_for_degrees(
    #     shoulder_speed, shoulder_speed, -shoulder_max * shoulder_ratio, True, True
    # )

    # shoulder_motor.position = 0
    # logger.info("Shoulder Position: {}".format(shoulder_motor.position))
    # shoulder_motor.on_for_degrees(
    #     shoulder_speed, shoulder_speed, shoulder_min * shoulder_ratio, True, True
    # )
    # logger.info("Shoulder Position: {}".format(shoulder_motor.position))
    # shoulder_motor.on_for_degrees(
    #     shoulder_speed, shoulder_speed, -shoulder_min * shoulder_ratio, True, True
    # )

    # elbow_motor.position = 0
    # logger.info("Shoulder Position: {}".format(elbow_motor.position))
    # elbow_motor.on_to_position(speed,-90*elbow_ratio,True,True)
    # logger.info("Shoulder Position: {}".format(elbow_motor.position))

    robot_arm = RobotArm()
    robot_arm.setDaemon(True)
    robot_arm.start()
    robot_arm.join()


if __name__ == "__main__":
    main()
