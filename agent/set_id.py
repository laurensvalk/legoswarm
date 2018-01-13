import ev3dev.auto as ev3
import time
# from PIL import Image, ImageDraw, ImageFont
import ev3dev.fonts as fonts

TOTAL_IDS = 16
DEG_PER_ID = 20

dial = ev3.Motor(ev3.OUTPUT_B)
screen = ev3.Screen()

while True:
    p = dial.position
    id = (p % (TOTAL_IDS * DEG_PER_ID))//TOTAL_IDS
    screen.draw.text((10, 10), str(id), font=fonts.load('luBS24'))
    screen.update()
    # print(id)
    time.sleep(0.3)