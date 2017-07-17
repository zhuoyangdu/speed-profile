import numpy as np
import time
import math
from node import Node

n = []
for i in range(0,200):
    n.append(Node(2,3,4))
array_n = np.array(n)
t1 = time.clock()
array_s = np.zeros(array_n.shape)
for i in range(0,200):
    array_s[i] = n[i].distance
print "t0:", time.clock()-t1

s = []
t = []
for i in range(2,202):
    s.append(i*i);
    t.append(i);

dis0 = 1
time0 = 1
t0 = time.clock()
for i in range(0,200):
    delta_s = dis0 - s[i]
    delta_t = time0 - t[i]
    v = delta_s/delta_t
    dis = math.sqrt(math.pow(delta_s,2)+math.pow(delta_t,2))
print "t1:", time.clock()-t0

t1 = time.clock()
array_s = np.array(s)
array_t = np.array(t)
array_v = array_s/array_t
dis = np.sqrt(array_s**2 + array_t**2)
print "t2:", time.clock()-t1
