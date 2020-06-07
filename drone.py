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
 - Keep video lag under 500ms
    - wireguard bottleneck probable
 - write telemetry to file - this is to work around the fact when the controller connection drops we still need
 - upon detecting a crash try to get as much sensor data as possible, push it to a websocket for recovery every 10s
     - apart from this try to modulate beeping noise with motor controller not spinning the motor
 - battery monitoring, also include in OSD

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
from servoctl import PCA9685
from baro import *

CONTROLLER_IP = "10.102.162.242"
# CONTROLLER_IP = "10.253.0.3"
CONTROLLER_TELE_PORT = 8888
CONTROLLER_VIDEO_PORT = 8890

imu = ICM20948()

THROTTLE_SERVO = 0
ELEVATOR_SERVO = 1
YAW_SERVO = 2
AILERON_SERVO_1 = 3
AILERON_SERVO_2 = 4

AIL_1 = 2150
AIL_2 = 2150

# DATA INIT
start_time = time.time()  # start time
ds = 0  # data counter
ds_mean = [0, 0, 0, 0, 0]
ds_iter = 0

TELE = ''  # telemetry
et = 0
# Initialize OpenCV lib for recording


# OSD
font = cv2.FONT_HERSHEY_SIMPLEX
BLC1 = (5, 475)
BLC2 = (115, 475)
BLC3 = (220, 475)
BLC4 = (355, 475)
BLC5 = (525, 475)
LC1 = (5, 15)
LC2 = (5, 35)
LC3 = (5, 55)
UC = (290, 10)
fontScale = 0.5
fontColor = (5, 50, 255)
lineType = 2
# CTL SURFACES
PIT = 0
ROL = 0
YAW = 0
THR = 0

pit_mean = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
c_pit = 0
rol_mean = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
c_rol = 0

iter = 0

altitude = 0

pwm = PCA9685()
pwm.setPWMFreq(10)
altimeter = LPS22HB()


# VIDEO STREAMING
def start_stream():
    global ds
    global ds_mean
    global ds_iter
    global altitude
    global et
    vid = cv2.VideoCapture(0)
    vid.set(cv2.CAP_PROP_FPS, 1000)
    client = imagiz.Client("cc1", server_ip=CONTROLLER_IP, server_port=CONTROLLER_VIDEO_PORT, request_retries=100000,
                           request_timeout=5000)
    encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 80]
    while True:
        r, frame = vid.read()
        pd = ''
        rd = ''
        if r:
            dss = np.mean(ds_mean) / 1024 / 8 / 10  # Data rate
            global c_hdg
            # Construct overlay data
            met = 'MET: ' + str(round(et, 2))
            dat = 'DATA: ' + str(round(ds, 2)) + 'M'
            bat1 = 'SYSV: ' + '7.28'
            bat2 = 'ENGV:' + '13.93'
            bit = 'RATE: ' + str(round(dss, 2)) + 'MBit/s'
            alt = 'ALT   : ' + str(round(altitude, 2)) + " m"

            if c_pit < 0:
                pd = 'PIT  D: '
            if c_pit >= 0:
                pd = 'PIT  U: '

            if c_rol < 0:
                rd = 'ROL L: '
            if c_rol >= 0:
                rd = 'ROL R: '

            pitc = pd + str(round(c_pit, 2))
            roll = rd + str(round(c_rol, 2))

            cv2.putText(frame, met, BLC1, font, fontScale, fontColor, lineType)
            cv2.putText(frame, dat, BLC5, font, fontScale, fontColor, lineType)
            cv2.putText(frame, bat1, BLC2, font, fontScale, fontColor, lineType)
            cv2.putText(frame, bat2, BLC3, font, fontScale, fontColor, lineType)
            cv2.putText(frame, bit, BLC4, font, fontScale, fontColor, lineType)
            cv2.putText(frame, pitc, LC1, font, fontScale, fontColor, lineType)
            cv2.putText(frame, roll, LC2, font, fontScale, fontColor, lineType)
            cv2.putText(frame, alt, LC3, font, fontScale, fontColor, lineType)
            # Encode and send to socket
            r, image = cv2.imencode('.jpg', frame, encode_param)
            ds += round(sys.getsizeof(image), 4) / 1024 / 1024
            ds_mean[ds_iter] = round(sys.getsizeof(image), 4)
            if ds_iter < 5:
                ds_iter += 1
            if ds_iter >= 5:
                ds_iter = 0
            client.send(image)

        else:
            pass


def start_server():
    host = "0.0.0.0"  # listen on all interfaces
    port = 8888  # command and controll port

    soc = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    soc.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)  # SO_REUSEADDR to reuse socket dropped to TIME_WAIT state
    print("Controller socket created")

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
        print("Controller downlink @ " + ip + ":" + port)

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
                print("Controller disconnecting...")
                connection.close()
                print("Controller @ " + ip + ":" + port + " closed")
                is_active = False
            else:
                # print("{}".format(client_input))
                st = datetime.datetime.fromtimestamp(time.time()).strftime('%H:%M:%S:%f ')
                connection.sendall("-".encode("utf8"))
        except ConnectionResetError:
            print('Controller downlink dropped, waiting to re-establish...')
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
    global iter
    global altitude

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
        PIT_POS = 0
        if PIT < 0:
            PIT_POS = (PIT * 1250) - (-1250) + 850
        if (PIT > 0) | (PIT == 0):
            PIT_POS = int(round((PIT * 1250) + 1250 + 850))

        # THROTTLE CONTROLL
        """
        Aparently it is a thing to controll the ESC with the servo hat since it is PWM
        Needs testing and range definition
        """
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
        YAW_POS = 0
        if YAW < 0:
            YAW_POS = int((YAW * 1250) - (-1250) + 850)
        if (YAW > 0) or (YAW == 0):
            YAW_POS = int((YAW * 1250) + 1250 + 850)

        pwm.setServoPulse(ELEVATOR_SERVO, PIT_POS)
        pwm.setServoPulse(THROTTLE_SERVO, THR_POS)
        pwm.setServoPulse(YAW_SERVO, YAW_POS)
        pwm.setServoPulse(AILERON_SERVO_1, AIL_1)
        pwm.setServoPulse(AILERON_SERVO_2, AIL_2)

        return proc


def send_telemetry():
    global TELE
    while True:
        try:
            s.send(bytes(TELE, 'utf-8'))
            time.sleep(1)
        except ConnectionResetError:
            print('Telemetry uplink disconnect, trying to reconnect...')
            time.sleep(1)
            establish_telemetry()
        except BrokenPipeError:
            print('General failure, reconnecting...')
            time.sleep(1)
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
            print('Telemetry uplink established.')
        except ConnectionRefusedError:
            time.sleep(5)
            print('Telemetry reconnecting...')


def log_telemetry():
    global TELE, et, start_time
    global c_rol, c_pit, c_hdg, altitude

    while True:
        et = time.time() - start_time  # MET
        TELE = 'ET=' + str(round(et, 2)) + ',PITCH=' + str(c_pit) + ',ROLL=' + str(c_rol) \
               + ',ALT=' + str(round(altitude, 2))
        # print(TELE)
        try:
            f = open("blackbox.txt", "a")
            f.write(TELE + '\n')
            f.close()
        except:
            print('Telemetry saving error!')

        time.sleep(0.05)


def gyro():
    global iter
    global c_rol, c_pit
    global pit_mean, rol_mean
    calibrated = False

    print('Calibrating gyro...')

    while True:
        # Get raw acc + gyro and mag data from IMU
        ax, ay, az, gx, gy, gz = imu.read_accelerometer_gyro_data()
        mag_x, mag_y, mag_z = imu.read_magnetometer_data()
        pitch = round((180 * math.atan2(ax, math.sqrt(ay * ay + az * az)) / 3.14), 2) * -1
        roll = round((180 * math.atan2(ay, math.sqrt(ax * ax + az * az)) / 3.14), 2) * -1

        # Compass is fuckered, can't figure out how to get heading from this IMU

        # Average out sensor data for display
        pit_mean[iter] = pitch
        pitch = int(np.mean(pit_mean))
        rol_mean[iter] = roll
        roll = int(np.mean(rol_mean))

        # Averaging + telemetry clock
        if iter < 21:
            iter += 1
            c_rol = round(float(np.mean(rol_mean)), 2)
            c_pit = round(float(np.mean(pit_mean)), 2)
        elif iter == 21:
            iter = 0
            if calibrated == False:
                print('Gyro calibrated...')
                calibrated = True


def baro():
    PRESS_DATA = 0.0
    TEMP_DATA = 0.0
    u8Buf = [0, 0, 0]
    cal_iter = 0
    p0t = 0
    p0 = 0
    calibrated = False
    print("Calibrating altimeter...")
    lps22hb = LPS22HB()
    while True:
        time.sleep(0.1)
        lps22hb.LPS22HB_START_ONESHOT()
        if (lps22hb._read_byte(LPS_STATUS) & 0x01) == 0x01:  # a new pressure data is generated
            u8Buf[0] = lps22hb._read_byte(LPS_PRESS_OUT_XL)
            u8Buf[1] = lps22hb._read_byte(LPS_PRESS_OUT_L)
            u8Buf[2] = lps22hb._read_byte(LPS_PRESS_OUT_H)
            PRESS_DATA = ((u8Buf[2] << 16) + (u8Buf[1] << 8) + u8Buf[0]) / 4096.0
        if (lps22hb._read_byte(LPS_STATUS) & 0x02) == 0x02:  # a new pressure data is generated
            u8Buf[0] = lps22hb._read_byte(LPS_TEMP_OUT_L)
            u8Buf[1] = lps22hb._read_byte(LPS_TEMP_OUT_H)
            TEMP_DATA = ((u8Buf[1] << 8) + u8Buf[0]) / 100.0
        tempK = round((TEMP_DATA + 273.15), 2)
        if calibrated == False:
            if cal_iter < 20:
                p0t += PRESS_DATA
                cal_iter += 1
            if cal_iter == 20:
                calibrated = True
                p0 = p0t / 20
                print("Altimeter calibrated...")
        if calibrated == True:
            global altitude
            altitude = ((((p0 / PRESS_DATA) ** (1 / 5.275)) - 1) * tempK) / 0.0065


def main():
    print('\n#######################################')
    print("             Icarus v 0.2              ")
    print('#######################################\n')

    Thread(target=log_telemetry, daemon=False).start()
    time.sleep(2)
    Thread(target=baro, daemon=False).start()
    time.sleep(2)
    Thread(target=gyro, daemon=False).start()
    time.sleep(1)
    Thread(target=start_stream, daemon=False).start()
    time.sleep(1)
    Thread(target=start_server, daemon=False).start()
    time.sleep(1)
    Thread(target=establish_telemetry, daemon=False).start()
    time.sleep(1)
    Thread(target=send_telemetry, daemon=False).start()


if __name__ == "__main__":
    main()
