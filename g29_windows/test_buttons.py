import pygame

pygame.init()
pygame.joystick.init()

joy = pygame.joystick.Joystick(0)
joy.init()

print(f"设备: {joy.get_name()}")
print(f"轴: {joy.get_numaxes()}, 按钮: {joy.get_numbuttons()}")
print("按下方向盘上的拨片/按钮，看哪个 index 变化...")
print("按 Ctrl+C 退出")

prev_buttons = [False] * joy.get_numbuttons()

try:
    while True:
        pygame.event.pump()
        changed = []
        for i in range(joy.get_numbuttons()):
            state = joy.get_button(i)
            if state != prev_buttons[i]:
                changed.append(i)
                prev_buttons[i] = state
        if changed:
            for i in range(joy.get_numbuttons()):
                if prev_buttons[i]:
                    if i in changed:
                        print(f"  [PRESSED ] 按钮 {i}")
                    else:
                        pass  # already held
        pygame.time.wait(50)
except KeyboardInterrupt:
    pass
finally:
    pygame.quit()
