"""
This whole thing is meant to be used on a raspberry pi, I'm using 3B+
but you probably could handle it with a zero. Paraphenalia is a Warthog
HOTAS + pedals for the controller, Waveshare SENSE HAT (b) for the plane
+ a Waveshare servo hat for the plane. Code needs a lot of clean up, trim
on servos adjusted to desired needs. Also a logitech webcam mounted on
the plane which streams back to the controller. On local the image is OK with
some lag but when used with wireguard the lag is quite bad, use with caution.

TODO:
 - GUI - video + telemetry data with interpretation ( art. horizon, map position overlay)
 - ESC controlls - neeed to get an engine and an esc
 - servo readback if possible - calibration mode
 - config file to bind joystick, gamepad and network
 - baro sensor altitude correction from GPS - needed due to possible pressure change

"""
import pygame
import time
import datetime
import socket
import imagiz
import cv2
import sys
from threading import Thread
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib import style
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import random
from threading import Thread
import time

#########################################################################
print('#######################################')
print("             Icarus v 0.2              ")
print('#######################################')
#########################################################################

HOST = '10.102.162.218'
PORT = 8888
CONNECTED = False

timestamp = str(datetime.datetime.fromtimestamp(time.time()).strftime('%Y-%m-%d_%H-%M-%S_'))
start_time = time.time()

telelap = 0
telepit = 0
telerol = 0
telealt = 0

# Create figure for plotting
fig1 = plt.figure()
ax1 = fig1.add_subplot(1, 1, 1)
xs1 = []
ys1 = []

alt = 0


def video_recv():
    server = imagiz.Server(port=8890)
    outname = timestamp + 'output.avi'
    print('VID: Session video output file: \n' + outname)
    print('#######################################\n')
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
    print("TEL: Socket created")

    try:
        soc.bind((host, port))
    except:
        print("TEL: Bind failed. Error : " + str(sys.exc_info()))
        sys.exit()

    soc.listen(1)  # queue up to 5 requests
    print("TEL: Waiting for drone telemetry...")

    # infinite loop- do not reset for every requests
    while True:
        connection, address = soc.accept()
        ip, port = str(address[0]), str(address[1])
        print("TEL: Drone at:  " + ip + ":" + port)

        try:
            Thread(target=client_thread, args=(connection, ip, port)).start()
        except:
            print("TEL: Thread did not start.")
            traceback.print_exc()

    soc.close()


def client_thread(connection, ip, port, max_buffer_size=5120):
    is_active = True

    while is_active:
        try:
            client_input = receive_input(connection, max_buffer_size)

            if "--QUIT--" in client_input:
                print("TEL: Client is requesting to quit")
                connection.close()
                print("TEL: Connection " + ip + ":" + port + " closed")
                is_active = False
            else:
                curr_time = round((time.time() -start_time), 1)
                global telealt, telerol, telepit, teleelap
                try:
                    telesplit = client_input.split(",")
                    teleelapt  = telesplit[0].split("=")
                    teleelap   = float(teleelapt[1])
                    telepitct  = telesplit[1].split("=")
                    telepitc   = float(telepitct[1])
                    telerolt   = telesplit[2].split("=")
                    telerol    = float(telerolt[1])
                    telealtt   = telesplit[3].split("=")
                    telealt    = round(float(telealtt[1]),2)
                except:
                    pass
                st = datetime.datetime.fromtimestamp(time.time()).strftime('%H:%M:%S:%f ')
                connection.sendall("-".encode("utf8"))
        except ConnectionResetError:
            print('TEL: Telemetry dropped, waiting to re-establish...')
            is_active = False


def receive_input(connection, max_buffer_size):
    client_input = connection.recv(max_buffer_size)
    client_input_size = sys.getsizeof(client_input)

    if client_input_size > max_buffer_size:
        print("TEL: The input size is greater than expected {}".format(client_input_size))

    decoded_input = client_input.decode("utf8").rstrip()  # decode and strip end of line
    result = process_input(decoded_input)
    return result

def process_input(input_str):
    rd = str(input_str).upper()
    return rd


def animate(i, xs1, ys1):
    global telealt, ax1, fig1
    # Read temperature (Celsius) from TMP102

    # Add x and y to lists
    xs1.append(datetime.datetime.now().strftime('%S'))
    ys1.append(telealt)

    # Limit x and y lists to 20 items
    xs1 = xs1[-100:]
    ys1 = ys1[-100:]

    # Draw x and y lists
    ax1.clear()
    ax1.plot(xs1, ys1   )

    # Format plot
    plt.xticks(rotation=90, ha='right')
    plt.subplots_adjust(bottom=0.30)
    plt.title('Altitude over time')
    plt.ylabel('Altitude in meters')

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #
def establish():
    global CONNECTED
    global s
    established = False

    try:
        s.close()
    except:
        pass

    while not established:
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            s.connect((HOST, PORT))
            established = True
            print('CTL: Connected - drone @ ' + str(s.getpeername()[0]) + ',' + str(s.getpeername()[1]))
        except ConnectionRefusedError:
            print('CTL: Reconnecting...')


def controlls():

    global s

    pygame.display.init()
    pygame.joystick.init()
    pygame.init()

    """pygame.joystick.Joystick(1).init()  # Joystick
    pygame.joystick.Joystick(0).init()  # Throttle
    pygame.joystick.Joystick(2).init()  # Rudder"""

    yt = 0  # roll trim
    zt = 0  # yaw trim
    pt = 0  # pitch trim

    while True:

        """pygame.event.pump()

        # Init axes
        x = pygame.joystick.Joystick(1).get_axis(1)  # Pitch
        y = pygame.joystick.Joystick(1).get_axis(0)  # Roll
        t = pygame.joystick.Joystick(0).get_axis(2)  # Throttle
        z = pygame.joystick.Joystick(2).get_axis(2) # Yaw
        #pt = pygame.joystick.Joystick(0).get_axis(4)  # Pitch trim on the throttle wheel

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
        #if abs(pt) < 0.03:
        #    pt = 0

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
        # Send the values"""
        #cmd = 'X:' + '{:05.2f}'.format(x) + 'Y:' + '{:05.2f}'.format(y) + 'T:' + '{:05.2f}'.format(t) + 'Z:' + '{:05.2f}'.format(z)
        cmd = 'X:' + '{:05.2f}'.format(0) + 'Y:' + '{:05.2f}'.format(1) + 'T:' + '{:05.2f}'.format(
            2) + 'Z:' + '{:05.2f}'.format(3)


        try:
            s.send(bytes(cmd, 'utf-8'))
        except:
            try:
                s.shutdown(socket.SHUT_RDWR)
            except:
                continue
            print("CTL: " + 'Connection dropped, trying to reconnect.')
            establish()
        # Limit sample rate to avoid overflow / buffering
        time.sleep(0.08)


def main():
    global line
    global telepitc
    global st

    Thread(target=video_recv, daemon=False).start()
    time.sleep(2)
    Thread(target=establish, daemon=False).start()
    time.sleep(1)
    Thread(target=controlls, daemon=False).start()
    time.sleep(1)
    Thread(target=start_server, daemon=False).start()

    while True:
        global xs1, ys1, fig1
        ani = animation.FuncAnimation(fig1, animate, fargs=(xs1, ys1), interval=1000)
        plt.show()

if __name__ == "__main__":
    main()

