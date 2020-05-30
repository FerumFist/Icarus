"""
This whole thing is meant to be used on a raspberry pi, I'm using 3B+
but you probably could handle it with a zero. Paraphenalia is a Warthog
HOTAS + pedals for the controller, Waveshare SENSE HAT (b) for the plane
+ a Waveshare servo hat for the plane. Code needs a lot of clean up, trim
on servos adjusted to desired needs. Also a logitech webcam mounted on
the plane which streams back to the controller. On local the image is OK with
some lag but when used with wireguard the lag is quite bad, use with caution.

TODO:
 - send telemetry back to controller
 - GPS
 - stabilization when you let go - keep heading and level flight
 - Some sort of GPS autopilot (Controller interface to enter waypoints or read them from json?)
 - Redo video to keep lag under 500ms maybe stream it to a webpage?
 - optimize for speed
"""
import pygame
import time
import datetime
import socket
import imagiz
import cv2
from threading import Thread

#########################################################################
print('#######################################')
print("             Icarus v 0.1              ")
print('#######################################')
#########################################################################

HOST = '10.102.162.205'
PORT = 8888
CONNECTED = False

timestamp = str(datetime.datetime.fromtimestamp(time.time()).strftime('%Y-%m-%d_%H-%M-%S_'))

def video_recv():
    server = imagiz.Server(port=8889)
    outname = timestamp + 'output.avi'
    print('Session video output file: \n' + outname)
    print('#######################################')
    out = cv2.VideoWriter(outname, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'), 14, (640, 480))
    while True:
        message = server.receive()
        frame = cv2.imdecode(message.image, 1)
        out.write(frame)
        cv2.imshow("", frame)
        cv2.waitKey(1)

# # # # # # TELEMETRY RECVR # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #
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

    soc.close()


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
    rd = str(input_str).upper()

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #
def establish():
    global CONNECTED
    global s
    established = False

    if CONNECTED:
        s.close()
    else:
        CONNECTED = True

    while not established:
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            s.connect((HOST, PORT))
            established = True
        except ConnectionRefusedError:
            print('Reconnecting...')


def controlls():

    global s

    pygame.display.init()
    pygame.joystick.init()
    pygame.init()

    pygame.joystick.Joystick(1).init()  # Joystick
    pygame.joystick.Joystick(0).init()  # Throttle
    pygame.joystick.Joystick(2).init()  # Rudder

    yt = 0  # roll trim
    zt = 0  # yaw trim
    pt = 0  # pitch trim

    while True:

        pygame.event.pump()

        # Init axes
        x = pygame.joystick.Joystick(1).get_axis(1)  # Pitch
        y = pygame.joystick.Joystick(1).get_axis(0)  # Roll
        t = pygame.joystick.Joystick(0).get_axis(2)  # Throttle
        pt = pygame.joystick.Joystick(0).get_axis(4)  # Pitch trim
        z = pygame.joystick.Joystick(2).get_axis(2) # Yaw

        j = pygame.joystick.Joystick(1)
        events = pygame.event.get()
        for event in events:
            if event.type == pygame.JOYBUTTONDOWN:

                # ROLL TRIM #
                if j.get_button(15):
                    yt += 0.05
                if j.get_button(17):
                    yt -= 0.05
                if j.get_button(16) or j.get_button(14):
                    yt = 0

                # YAW TRIM #
                if j.get_button(7):
                    zt += 0.05
                if j.get_button(9):
                    zt -= 0.05
                if j.get_button(8) or j.get_button(6):
                    zt = 0

                # PITCH TRIM #
                if j.get_button(12):
                    pt += 0.05
                if j.get_button(10):
                    pt -= 0.05
                if j.get_button(2):
                    pt = 0

        # Pitch trim deadzone
        if abs(pt) < 0.03:
            pt = 0

        # Pitch
        if abs(x) < 0.035:
            x = 0 + pt
        else:
            x = x + pt
        if x < -1:
            x = -1
        if x > 1:
            x = 1

        # Roll
        if abs(y) < 0.03:
            y = 0 + yt
        else:
            y = y + yt

        # Throttle
        if (t * (-1) + 1) < 0.02:
            t = 0
        else:
            t = round(((t * (-1)) + 1) / 2, 3)

        # Yaw
        if abs(z) < 0.03:
            z = 0 + zt
        else:
            z = z + zt

        if x < -1:
            x = -1
        if x > 1:
            x = 1

        if y < -1:
            y = -1
        if y > 1:
            y = 1

        if z < -1:
            z = -1
        if z > 1:
            z = 1
        # Send the values
        cmd = 'X:' + '{:05.2f}'.format(x) + 'Y:' + '{:05.2f}'.format(y) + 'T:' + '{:05.2f}'.format(t) + 'Z:' + '{:05.2f}'.format(z)
        try:
            s.send(bytes(cmd, 'utf-8'))
        except ConnectionResetError:
            s.shutdown(socket.SHUT_RDWR)
            print('Connection dropped, trying to reconnect.')
            establish()
        # Limit sample rate to avoid overflow / buffering
        time.sleep(0.04)


def main():
    Thread(target=establish, daemon=False).start()
    Thread(target=video_recv, daemon=False).start()
    Thread(target=controlls, daemon=False).start()
    video_recv()

if __name__ == "__main__":
    main()
