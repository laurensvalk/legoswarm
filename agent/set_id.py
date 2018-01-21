#!/usr/bin/env python3

import ev3dev.auto as ev3
import time
import ev3dev.fonts as fonts

TOTAL_IDS = 16
DEG_PER_ID = 20

dial = ev3.Motor(ev3.OUTPUT_C)
screen = ev3.Screen()
btn = ev3.Button()

try:
    from id import MY_ID
except:
    MY_ID = 0

dial.position = MY_ID * DEG_PER_ID + DEG_PER_ID//2
while not btn.enter:
    p = dial.position
    id = (p % (TOTAL_IDS * DEG_PER_ID)) // DEG_PER_ID
    screen.clear()
    screen.draw.text((70, 50), str(id), font=fonts.load('luBS24'))
    screen.update()
    time.sleep(0.01)

id_file = open("id.py", 'w')
id_file.write("MY_ID = {0}\n".format(id))
