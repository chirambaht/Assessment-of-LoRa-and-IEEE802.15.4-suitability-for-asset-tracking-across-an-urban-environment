import matplotlib.pyplot as plt
import numpy as np
LoRA = [0,72, 89, 71, 55, 67, 14, 21, 75, 79, 53, 55, 55]
GPS = [0,18, 5, 7, 60, 30, 8, 5, 5, 50, 110, 4, 50]
MAP = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12]
lor_prec = [0,117.64, 64.20, 75.01, 200.01, 152.07, 30.81, 113.30, 172.65, 224.56, 144.46, 132.91, 61.61]
def get_average(arr):
    sum = 0
    for i in arr:
        sum += int(i)
    res = []
    for i in range(len(arr)):
        res.append(sum/len(arr))
    return res

data = [GPS, LoRA]
# plt.figure("GPS vs LoRa Accuracy")
# plt.bar(MAP,res)

X = np.arange(13)
fig = plt.figure("GPS and LoRa Accuracy")
plt.xlabel("Location point")
plt.ylabel("Accuracy (m)")
plt.bar(X + 0.00, data[0], color = 'aqua', width = 0.33)
plt.bar(X + 0.25, data[1], color = 'red', width = 0.33)
plt.plot(MAP, get_average(LoRA)[1:], '-y')
plt.plot(MAP, get_average(GPS)[1:], '-b')

fig = plt.figure("LoRa Precision")
plt.xlabel("Location point")
plt.ylabel("Precision (m)")
X = np.arange(13)
plt.bar(X + 0.00, lor_prec, color = 'purple', width = 0.75)
plt.show()