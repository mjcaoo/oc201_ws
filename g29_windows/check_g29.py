import pygame

pygame.init()
pygame.joystick.init()

count = pygame.joystick.get_count()
print(f"检测到 {count} 个游戏手柄设备")

for i in range(count):
    joy = pygame.joystick.Joystick(i)
    joy.init()
    print(f"\n设备 {i}:")
    print(f"  名称: {joy.get_name()}")
    print(f"  轴数量: {joy.get_numaxes()}")
    print(f"  按钮数量: {joy.get_numbuttons()}")
    print(f"  Hat数量: {joy.get_numhats()}")

pygame.quit()
