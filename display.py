import rospy
from itertools import count
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from xela_server.srv import XelaSensorXYZ
from xela_server.msg import xServerMsg

display_length = 100
max_length = 80
sensor_1 = []
sensor_2 = []
k = 4

# ll /dev/ | grep ttyUSB
# sudo slcand -o -s8 -t hw -S 3000000 /dev/ttyUSB0
# sudo ifconfig slcan0 up


def callback(xelaData):
    if xelaData.sensor == 1:
        sum_1 = 0
        for i in range(16):
            sum_1 = sum_1 + xelaData.points[i].point.x
        sensor_1.append(abs(sum_1))

    if xelaData.sensor == 2:
        sum_2 = 0
        for i in range(16):
            sum_2 = sum_2 + xelaData.points[i].point.x
        sensor_2.append(abs(sum_2))

    if len(sensor_1) > max_length:
        sensor_1.pop(0)
    if len(sensor_2) > max_length:
        sensor_2.pop(0)


rospy.init_node("xela_use_rostopic")
rospy.Subscriber("/xServTopic", xServerMsg, callback)


def animate(i):
    # font_red = {
    #     "family": "serif",
    #     "color": "red",
    #     "weight": "normal",
    #     "size": 10,
    # }
    # font_blue = {
    #     "family": "serif",
    #     "color": "blue",
    #     "weight": "normal",
    #     "size": 10,
    # }
    # font_green = {
    #     "family": "serif",
    #     "color": "green",
    #     "weight": "normal",
    #     "size": 10,
    # }
    # font_black = {
    #     "family": "serif",
    #     "color": "black",
    #     "weight": "normal",
    #     "size": 10,
    # }

    plt.subplot(211)
    plt.cla()  # clear the picture of 211
    plt.plot(sensor_1, "r")
    plt.ylim(-200, 8000)
    plt.xlim(0, display_length)
    plt.title("Right Sensor")
    plt.grid(True)
    plt.xlabel("Timeline")
    plt.ylabel("Value")

    plt.subplot(212)
    plt.cla()  # clear the picture of 211
    plt.plot(sensor_2, "b")
    plt.ylim(-200, 8000)
    plt.xlim(0, display_length)
    plt.title("Left Sensor")
    plt.grid(True)
    plt.xlabel("Timeline")
    plt.ylabel("Value")

    plt.subplots_adjust(
        top=0.92, bottom=0.08, left=0.10, right=0.95, hspace=0.25, wspace=0.35
    )


# fig = plt.figure()
# # ani = FuncAnimation(plt.gcf(), animate, interval=1) # interval = 1 means that 'Delay between frames in 1 millisecond.' Default value is 200, but set to 1 here.
# ani = FuncAnimation(fig, animate, interval=1) # interval = 1 means that 'Delay between frames in 1 millisecond.' Default value is 200, but set to 1 here.

fig = plt.figure()
ani = FuncAnimation(
    fig, animate, interval=1
)  # interval = 1 means that 'Delay between frames in 1 millisecond.' Default value is 200, but set to 1 here.

plt.tight_layout()
plt.show()
