# Script for calculating the averaged elapsed time of the optimizer.
# Use output of the estimator node as the input file "./out.txt" 
# eg. rosrun us_image_processing estimator_node >> out.txt

import re
from statistics import mean

f = open("./out.txt", "r")
buf = f.read()

pattern_r = 'target: r, elapsed time:'
pattern_n = 'target: n, elapsed time:'

lr = [m.end()+1 for m in re.finditer(pattern_r, buf)]
ln = [m.end()+1 for m in re.finditer(pattern_n, buf)]

dt_r = [float(buf[idx:idx+5]) for idx in lr]

dt_n = [float(buf[idx:idx+5]) for idx in ln]

print("avg elapsed time r: ", mean(dt_r), "ms")
print("avg elapsed time n: ", mean(dt_n), "ms")
