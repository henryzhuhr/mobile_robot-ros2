from time import sleep
import pygame

pygame.init()
pygame.joystick.init()
done=False

# class HandleControl:
#     def __init__(self) -> None:
#         sel

axis_define=["左转向","左垂直","右前进","右左右",]
axis_values=[0]*len(axis_define)
print(axis_values)

while (done != True):
    for event in pygame.event.get():  # User did something
        if event.type == pygame.QUIT:  # If user clicked close
            done = True  # Flag that we are done so we exit this loop
    joystick_count = pygame.joystick.get_count()
    for i in range(joystick_count):
        joystick = pygame.joystick.Joystick(i)
        joystick.init()
        
        numaxes = joystick.get_numaxes()
        assert numaxes == 4
        for i in range(numaxes):
            axis = joystick.get_axis(i)
            scale = 100# 截断小数点后两位
            axis = int(axis * scale) / scale
            axis_values[i]=axis
        print("    ".join([
            f"{axis_define[i]}:{axis_values[i]}"
            for i in range(len(axis_define))
        ]))
        sleep(1)

