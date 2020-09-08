#!/usr/bin/env python3
#
# template for ev3dev2-based code
#
import time, random
goal_DPS = 400
j = 0.0
j_diff = 0
best_match = 99999.99
optimal_i = 99999.99

for i in range(1, 26):
    j = random.randrange(40*i)
    j_diff = abs((goal_DPS - j))
    if j_diff < best_match:
        best_match = j_diff
        optimal_i = j
    print ("j = ", j, ";   j_diff = ", j_diff)
    print ("current best_match = ", best_match, "; current optimal_i", optimal_i)
print ("Final data:")
print ("best_match = ", best_match, "; optimal_i", optimal_i)

