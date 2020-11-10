import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import numpy as np
import math

count = 1
scale = 1  # 1 px = scale cm
step = 1
# LORA_RSSI_MAP = {0:255, 1:239, 2:192, 3:181, 4: 175, 5: 165, 6:155, 7: 140, 8:125, 9: 110, 10: 100, 11:95, 12:90}
# LORA_RSSI_MAP = {0:255, 1:239, 2:209, 3:191, 4: 171, 5: 141, 6:121, 7: 100, 8:80, 9: 60, 10: 50, 11:0, 12:90}
# LORA_RSSI_MAP = {0:231, 10:176, 20:175, 30:171, 40: 170, 50: 166, 60:165, 70: 165, 80:165, 90: 164, 100: 164, 110:140}
LORA_RSSI_MAP = {}
MAP = []
x = []
A = (140,71)
B = (255, 191)
C = (395,116)
c = []
def addCircle(cord, rssi, name, axes, col='blue',pad=5):
    cent = cord
    x,y = cord
    
    r = get_distance(rssi) * scale
    # print(rssi, "=", r, "meters")
    draw_circle = plt.Circle(cent, r, fill=True, alpha=0.5, color=col, clip_on=False)

    axes.text(x,y, name[0], wrap=True, color='white')
    
    axes.set_aspect(1)
    axes.add_artist(draw_circle)

def fillValues(start, finish, num):
    arr = []
    # Linear
    div = (finish - start) / num
    for i in range(0, num):
        arr.append(start + (i * div))
    
    return arr

def get_keys():
    l = []
    keys = LORA_RSSI_MAP.keys()
    for i in keys:
        l.append(i)
    l.sort()

    return l

def fillRange(start, finish, num):
    x_v = []
    d = round((finish - start)/num,2)
    for i in range(num):
        x_v.append(start + (d*i))
    return x_v

def make_rssi():
    r = open("range.csv", 'r')
    for i in r:
        i = i.replace('\n', '')
        if ',' not in i:
            continue
        d = i.split(',')
        LORA_RSSI_MAP[int(d[0])] = int(d[1])
    r.close()
    graph = []
    x_vals = []
    keys = get_keys()
    for ptr,val in enumerate(keys):
        if (ptr == 0):
            continue
        if (ptr == len(keys)):
            break
        mini = fillValues(LORA_RSSI_MAP[keys[ptr-1]], LORA_RSSI_MAP[keys[ptr]], 100)
        min_x = fillRange(keys[ptr-1], keys[ptr], 100)
        for x in mini:
            graph.append(x)
        for t in min_x:
            x_vals.append(t)

    return (graph,x_vals)

def show_distance_graph():
    c = MAP
    plt.plot(x,c)
    plt.ylabel("RSSI")
    plt.xlabel("Distance (m)")

def get_distance(rssi):
    ctr = 0
    last = 255
    c = MAP
    for i in c:
        if rssi <= last and rssi > i:
            return x[ctr]
        last = i
        ctr += 1
    return -1

def calc(a_value, b_value, c_value, label):
    # if str(label) != "100":
    #     return
    vals = (a_value,b_value,c_value)
    f, axes = plt.subplots(1)
    s = plt.imread("figures/uct_overhead.png")
    plt.imshow(s)
    plt.title("Location " + label)
    plt.xlabel("Distance (m)")
    plt.ylabel("Distance (m)")
    addCircle(C, int(c_value),'C', axes,'yellow')
    addCircle(B, int(b_value), 'B', axes, 'red')
    addCircle(A, int(a_value), 'A', axes, 'aqua')
    plt.savefig("saves/"+label+".png")

def plot_data():

    f, axes = plt.subplots(1)
    s = plt.imread("figures/uct_overhead.png")
    plt.imshow(s)
    plt.title("Test Locations")
    plt.xlabel("Distance (m)")
    plt.ylabel("Distance (m)")

    f = open("positions.csv", 'r')
    for i in f:
        i = i.replace('\n', '')
        d = i.split(',')

        for j in d:
            addCircle((int(d[1]), int(d[2])),254, d[0],axes,'red',0)

MAP,x = make_rssi()

# show_distance_graph()
plot_data()
# f = open("data.txt", 'r')
# for i in f:
#     i = i.replace('\n', '')
#     d = i.split(',')
#     for i in d:
#         i = int(i)
#     calc(d[1], d[2], d[3], d[0])
#     count += 1
# f.close()
plt.show()