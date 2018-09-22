from communication import Sensor
from time import sleep
from communication import float_to_hex
import matplotlib.pyplot as mp
import numpy as np

A = np.array([
14153.528850,
9529.437298,
3312.924592,
7527.679666,
10237.500000,
5360.424592,
6998.588114,
1191.411886,
1265.424592,
2047.500000,
662.320334,
782.075408,
1339.437298,
5963.528850,
4095.000000,
5963.528850,
1339.437298,
782.075408,
662.320334,
2047.500000,
1265.424592,
1191.411886,
6998.588114,
5360.424592,
10237.500000,
7527.679666,
3312.924592,
9529.437298,
14153.528850])
mp.plot(A)
mp.show()
# sensor = Sensor()

# sensor.connect("30:AE:A4:06:4F:46")
# # sensor.trigger_measurement(2,2.1)
# sleep(5)
# print(sensor.read_signal())
# print(sensor.read_calculated_value("amplitude"))
# print(sensor.read_calculated_value("rms"))
# print(sensor.read_calculated_value("max_val"))
# print(sensor.read_calculated_value("min_val"))
# print(sensor.read_calculated_value("crest_factor"))
# print(sensor.read_calculated_value("average"))
# sensor.disconnect()

