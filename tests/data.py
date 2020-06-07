import datetime as dt
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import random
from threading import Thread
import time
# Create figure for plotting
fig = plt.figure()
ax = fig.add_subplot(1, 1, 1)
xs = []
ys = []
alt = 0
# Initialize communication with TMP102

def animate(i, xs, ys):
    global alt
    # Read temperature (Celsius) from TMP102
    temp_c = alt

    # Add x and y to lists
    xs.append(dt.datetime.now().strftime('%H:%M:%S.%f'))
    ys.append(temp_c)

    # Limit x and y lists to 20 items
    xs = xs[-50:]
    ys = ys[-50:]

    # Draw x and y lists
    ax.clear()
    ax.plot(xs, ys)

    # Format plot
    plt.xticks(rotation=45, ha='right')
    plt.subplots_adjust(bottom=0.30)
    plt.title('Altitude over time')
    plt.ylabel('Altitude in meters')


def chgval():
    global alt
    while True:
        alt = random.randint(10,15)
        print(alt)
        time.sleep(0.1)

chgthread  = Thread(target=chgval, daemon=False)
chgthread.start()

ani = animation.FuncAnimation(fig, animate, fargs=(xs, ys), interval=1000)
plt.show()