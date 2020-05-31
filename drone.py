"""
This whole thing is meant to be used on a raspberry pi, I'm using 3B+
but you probably could handle it with a zero. Paraphenalia is a Warthog
HOTAS + pedals for the controller, Waveshare SENSE HAT (b) for the plane
+ a Waveshare servo hat for the plane. Code needs a lot of clean up, trim
on servos adjusted to desired needs. Also a logitech webcam mounted on
the plane which streams back to the controller. On local the image is OK with
some lag but when used with wireguard the lag is quite bad, use with caution.

TODO:
 - GPS
 - stabilization when you let go - keep heading and level flight
 - Some sort of GPS autopilot (Controller interface to enter waypoints or read them from json?)
 - Redo video to keep lag under 500ms maybe stream it to a webpage?
 - optimize for speed
"""
import socket
import traceback
from threading import Thread
import imagiz
import cv2
import time
import sys
import datetime
import math
import smbus
from icm20948 import ICM20948
import numpy as np

# Controller addressing
CONTROLLER_IP = "10.102.162.242"
CONTROLLER_TELE_PORT = 8888
CONTROLLER_VIDEO_PORT = 8889
# Servo addresses
THROTTLE_SERVO  = 0
ELEVATOR_SERVO  = 1
YAW_SERVO       = 2
AILERON_SERVO_1 = 3
AILERON_SERVO_2 = 4
# get rid of this
AIL_1 = 2150
AIL_2 = 2150
# DATA INIT
start_time = time.time()  # start time
ds = 0  # data counter
TELE = ''  # telemetry
# Controll surface position
PIT = 0
ROL = 0
YAW = 0
THR = 0
# For smoothing sensor data
hdg_mean = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
c_hdg = 0
pit_mean = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
c_pit = 0
rol_mean = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
c_rol = 0
# Testing params only
heading = 92
GPS_LAT = 72.112
GPS_LON = 112.317
GPS_ALT = 721

iter = 0

# SERVO SHIT
class PCA9685:
    # Registers/etc.
    __SUBADR1 = 0x02
    __SUBADR2 = 0x03
    __SUBADR3 = 0x04
    __MODE1 = 0x00
    __PRESCALE = 0xFE
    __LED0_ON_L = 0x06
    __LED0_ON_H = 0x07
    __LED0_OFF_L = 0x08
    __LED0_OFF_H = 0x09
    __ALLLED_ON_L = 0xFA
    __ALLLED_ON_H = 0xFB
    __ALLLED_OFF_L = 0xFC
    __ALLLED_OFF_H = 0xFD

    def __init__(self, address=0x40, debug=False):
        self.bus = smbus.SMBus(1)
        self.address = address
        self.debug = debug
        if (self.debug):
            print("Reseting PCA9685")
        self.write(self.__MODE1, 0x00)

    def write(self, reg, value):
        "Writes an 8-bit value to the specified register/address"
        self.bus.write_byte_data(self.address, reg, value)
        if (self.debug):
            print("I2C: Write 0x%02X to register 0x%02X" % (value, reg))

    def read(self, reg):
        "Read an unsigned byte from the I2C device"
        result = self.bus.read_byte_data(self.address, reg)
        if (self.debug):
            print("I2C: Device 0x%02X returned 0x%02X from reg 0x%02X" % (self.address, result & 0xFF, reg))
        return result

    def setPWMFreq(self, freq):
        "Sets the PWM frequency"
        prescaleval = 25000000.0  # 25MHz
        prescaleval /= 4096.0  # 12-bit
        prescaleval /= float(freq)
        prescaleval -= 1.0
        if (self.debug):
            print("Setting PWM frequency to %d Hz" % freq)
            print("Estimated pre-scale: %d" % prescaleval)
        prescale = math.floor(prescaleval + 0.5)
        if (self.debug):
            print("Final pre-scale: %d" % prescale)

        oldmode = self.read(self.__MODE1);
        newmode = (oldmode & 0x7F) | 0x10  # sleep
        self.write(self.__MODE1, newmode)  # go to sleep
        self.write(self.__PRESCALE, int(math.floor(prescale)))
        self.write(self.__MODE1, oldmode)
        time.sleep(0.005)
        self.write(self.__MODE1, oldmode | 0x80)

    def setPWM(self, channel, on, off):
        "Sets a single PWM channel"
        self.write(self.__LED0_ON_L + 4 * channel, on & 0xFF)
        self.write(self.__LED0_ON_H + 4 * channel, on >> 8)
        self.write(self.__LED0_OFF_L + 4 * channel, off & 0xFF)
        self.write(self.__LED0_OFF_H + 4 * channel, off >> 8)
        if (self.debug):
            print("channel: %d  LED_ON: %d LED_OFF: %d" % (channel, on, off))

    def setServoPulse(self, channel, pulse):
        "Sets the Servo Pulse,The PWM frequency must be 50HZ"
        pulse = pulse * 4096 / 20000  # PWM frequency is 50HZ,the period is 20000us
        self.setPWM(channel, 0, int(pulse))

imu = ICM20948()
pwm = PCA9685()
pwm.setPWMFreq(10)

# VIDEO STREAMING
def start_stream():
    global ds
    # OSD
    font = cv2.FONT_HERSHEY_SIMPLEX
    BLC1 = (5, 475)
    BLC2 = (115, 475)
    BLC3 = (220, 475)
    BLC4 = (355, 475)
    BLC5 = (525, 475)
    UC = (290, 10)
    fontScale = 0.5
    fontColor = (255, 255, 255)
    lineType = 2
    vid = cv2.VideoCapture(0)
    vid.set(cv2.CAP_PROP_FPS, 100)
    client = imagiz.Client("cc1", server_ip=CONTROLLER_IP, server_port=CONTROLLER_VIDEO_PORT, request_retries=1000,
                           request_timeout=10000)
    encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 10]
    while True:
        # r = True
        # vs = WebcamVideoStream(src=0).start()
        # frame = vs.read()
        # frame = imutils.resize(frame, width=400)
        r, frame = vid.read()
        if r:
            et = time.time() - start_time  # MET
            dss = ds / et  # Data rate
            global heading
            global TELE

            # Construct overlay data
            eto = 'MET: ' + str(round(et, 2))
            br = 'DATA: ' + str(round(ds, 2)) + 'M'
            bat1 = 'SYSV: ' + '7.28'
            bat2 = 'ENGV:' + '13.93'
            brr = 'RATE: ' + str(round(dss, 2)) + 'MB/s'
            hdg = str(int(heading))
            # Construct telemetry data - why the fuck here?
            TELE = eto + ' ' + bat1 + ' ' + bat2 + ' ' + br + ' ' + brr
            # Whack in the overlays
            cv2.putText(frame, eto, BLC1, font, fontScale, fontColor, lineType)
            cv2.putText(frame, br, BLC5, font, fontScale, fontColor, lineType)
            cv2.putText(frame, bat1, BLC2, font, fontScale, fontColor, lineType)
            cv2.putText(frame, bat2, BLC3, font, fontScale, fontColor, lineType)
            cv2.putText(frame, brr, BLC4, font, fontScale, fontColor, lineType)
            cv2.putText(frame, hdg, UC, font, fontScale, fontColor, lineType)
            # BW conversion - no data saved, may be used later for machine vision?
            # bw = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
            bw = frame
            # Encode and send to socket
            r, image = cv2.imencode('.jpg', bw, encode_param)
            ds += round(sys.getsizeof(image), 4) / 1024 / 1024 / 8
            client.send(image)

        else:
            pass


def start_server():
    host = "0.0.0.0"  # listen on all interfaces
    port = 8888  # command and controll port

    soc = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    soc.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)  # SO_REUSEADDR to reuse socket dropped to TIME_WAIT state
    print("Socket created")

    try:
        soc.bind((host, port))
    except:
        print("Bind failed. Error : " + str(sys.exc_info()))
        sys.exit()

    soc.listen(1)  # queue up to 5 requests
    print("Waiting for controller connection...")

    # infinite loop- do not reset for every requests
    while True:
        connection, address = soc.accept()
        ip, port = str(address[0]), str(address[1])
        print("Connected with " + ip + ":" + port)

        try:
            Thread(target=client_thread, args=(connection, ip, port)).start()
        except:
            print("Thread did not start.")
            traceback.print_exc()


def client_thread(connection, ip, port, max_buffer_size=5120):
    is_active = True

    while is_active:
        try:
            client_input = receive_input(connection, max_buffer_size)
            if "--QUIT--" in client_input:
                print("Client is requesting to quit")
                connection.close()
                print("Connection " + ip + ":" + port + " closed")
                is_active = False
            else:
                # print("{}".format(client_input))
                st = datetime.datetime.fromtimestamp(time.time()).strftime('%H:%M:%S:%f ')
                global TELE
                f = open("blackbox.txt", "a")
                f.write(st + client_input + '\t' + TELE + '\n')
                f.close()
                connection.sendall("-".encode("utf8"))
        except ConnectionResetError:
            print('Uplink dropped, waiting to re-establish...')
            is_active = False


def receive_input(connection, max_buffer_size):
    client_input = connection.recv(max_buffer_size)
    client_input_size = sys.getsizeof(client_input)

    if client_input_size > max_buffer_size:
        print("The input size is greater than expected {}".format(client_input_size))

    decoded_input = client_input.decode("utf8").rstrip()  # decode and strip end of line
    result = process_input(decoded_input)
    return result


def process_input(input_str):
    global ds, PIT, ROL, YAW, THR, AIL_1, AIL_2, GPS_LAT, GPS_LON, GPS_ALT
    global hdg_mean, pit_mean, rol_mean
    global c_hdg, c_pit, c_rol
    global iter
    pi = 3.14159265359

    # Check size of input, if there is stutter and overflow just drop the frame
    ds += len(input_str) / 1024 / 1024 / 8
    rd = str(input_str).upper()
    if len(rd) > 28:
        return ''
    # Chop up the input to command axes
    else:
        proc = str(input_str).upper()
        PIT = float(proc[2:7])
        ROL = float(proc[9:14])
        THR = float(proc[16:21])
        YAW = float(proc[23:28])

        # PITCH SERVO CONTROLL
        if PIT < 0:
            PIT_POS = (PIT * 1250) - (-1250) + 850
        if (PIT > 0) | (PIT == 0):
            PIT_POS = int(round((PIT * 1250) + 1250 + 850))

        # THROTTLE SERVO CONTROLL
        THR_POS = (int(THR * 26.5) * 100) + 850

        # ROLL SERVO CONTROLL
        ROL_POS = (int(ROL * 13.25) * 100)
        if ROL_POS < 0:  # Negative - left roll
            ROL = 2175 - abs(ROL_POS)
            AIL_1 = ROL
        if ROL_POS > 0:  # Positive - right roll
            ROL = ROL_POS + 2175
            AIL_2 = ROL
        if ROL_POS == 0:
            AIL_1 = 2150
            AIL_2 = 2150

        # YAW SERVO CONTROLL
        if YAW < 0:
            YAW_POS = int((YAW * 1250) - (-1250) + 850)
        if (YAW > 0) or (YAW == 0):
            YAW_POS = int((YAW * 1250) + 1250 + 850)

        pwm.setServoPulse(ELEVATOR_SERVO, PIT_POS)
        pwm.setServoPulse(THROTTLE_SERVO, THR_POS)
        pwm.setServoPulse(YAW_SERVO, YAW_POS)
        pwm.setServoPulse(AILERON_SERVO_1, AIL_1)
        pwm.setServoPulse(AILERON_SERVO_2, AIL_2)

        # Get raw acc + gyro and mag data from IMU
        ax, ay, az, gx, gy, gz = imu.read_accelerometer_gyro_data()
        mag_x, mag_y, mag_z = imu.read_magnetometer_data()
        pitch = round((180 * math.atan2(ax, math.sqrt(ay * ay + az * az)) / 3.14), 2) * -1
        roll = round((180 * math.atan2(ay, math.sqrt(ax * ax + az * az)) / 3.14), 2) * -1

        # Compass is fuckered, can't figure out how to get heading from this IMU
        hdg = 180 * math.atan2(mag_x, mag_y) / pi
        hdg_mean[iter] = hdg
        hdg_act = np.mean(hdg_mean)
        if hdg_act < 0:
            hdg_act += 360

        # Average out sensor data for display
        pit_mean[iter] = pitch
        pitch = int(np.mean(pit_mean))
        rol_mean[iter] = roll
        roll = int(np.mean(rol_mean))

        # Averaging + telemetry clock
        if iter < 20:
            iter += 1
        elif iter == 20:
            iter = 0
            et = time.time() - start_time  # MET
            ct = datetime.datetime.now()
            send_telemetry("MET=" + str(round(et, 2)) + str(pitch) + "," + str(roll) + "," + str(GPS_LAT) + "," +
                      str(GPS_LON) + "," + str(GPS_ALT) + "Cycle= " + str(ct.strftime("%M:%S:%f")))
        return proc

def send_telemetry(data):
    try:
        s.send(bytes(data, 'utf-8'))
    except ConnectionResetError:
        print('Connection dropped, trying to reconnect.')
        establish_telemetry()

def establish_telemetry():
    global s
    established = False

    while not established:
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            s.connect((CONTROLLER_IP, CONTROLLER_TELE_PORT))
            established = True
        except ConnectionRefusedError:
            print('Reconnecting...')

def main():

    Thread(target=start_stream, daemon=False).start()
    Thread(target=start_server, daemon=False).start()
    Thread(target=establish_telemetry, daemon=False).start()


if __name__ == "__main__":
    main()
