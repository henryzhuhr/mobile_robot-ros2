import pygame

# 初始化PyGame
pygame.init()

# 设置游戏窗口
screen = pygame.display.set_mode((640, 480))

# 创建一个时钟对象，用于控制游戏的更新速度
clock = pygame.time.Clock()

# 主循环
running = True
while running:
    # 处理事件
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    # 读取USB控制器的输入
    joystick_count = pygame.joystick.get_count()
    print("%d"%joystick_count)
    for i in range(joystick_count):
        print("loop")
        joystick = pygame.joystick.Joystick(i)
        joystick.init()

        # 获取控制器的轴输入
        num_axes = joystick.get_numaxes()
        for axis_id in range(num_axes):
            axis_value = joystick.get_axis(axis_id)
            print(f"Axis {axis_id}: {axis_value}")

        # 获取控制器的按钮输入
        num_buttons = joystick.get_numbuttons()
        for button_id in range(num_buttons):
            button_value = joystick.get_button(button_id)
            print(f"Button {button_id}: {button_value}")

    # 更新游戏画面
    pygame.display.flip()

    # 控制游戏更新速率为30帧每秒
    clock.tick(30)

# 退出PyGame
pygame.quit()

