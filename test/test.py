import pygame

class JoystickData:
    def __init__(self):
        # Button byte-1: Square Triangle Cross Circle Up Down L1 R1
        self.button1 = 0
        # Button byte-2: Share Options PS4 Left right Left-stick-press Right-stick-press Touch_Pad_Click
        self.button2 = 0
        # L2
        self.l2 = 0
        # R2
        self.r2 = 0
        # Left-stick-horizontal(x)
        self.lx = 0
        # Left-stick-vertical(y)
        self.ly = 0
        # Right-stick-horizontal(x)
        self.rx = 0
        # Right-stick-vertical(y)
        self.ry = 0


def connect_ps4_controller():
    pygame.init()
    pygame.joystick.init()

    num_joysticks = pygame.joystick.get_count()

    if num_joysticks == 0:
        print("No joystick found. Make sure your PS4 controller is connected.")
        return None

    joystick = pygame.joystick.Joystick(0)
    joystick.init()

    print("Connected to PS4 controller")

    return joystick


def parse_and_print_data(joystick):
    joystick_data = JoystickData()

    try:
        while True:
            pygame.event.pump()

            # for event in pygame.event.get():
            #     if event.type == pygame.JOYBUTTONDOWN:
            print(joystick.get_button(0),
                  joystick.get_button(1),
                  joystick.get_button(2),
                  joystick.get_button(3),
                  joystick.get_button(4),
                  joystick.get_button(5),
                  joystick.get_button(6),
                  joystick.get_button(7),
                  joystick.get_button(8),
                  joystick.get_button(9),
                  joystick.get_button(10),
                  joystick.get_button(11),
                  joystick.get_button(12),
                  joystick.get_button(13),
                  joystick.get_button(14),
                  joystick.get_button(15))

            #     joystick_data.l2 = joystick.get_axis(2)
            #     joystick_data.r2 = joystick.get_axis(5)
            #     joystick_data.lx = joystick.get_axis(0)
            #     joystick_data.ly = joystick.get_axis(1)
            #     joystick_data.rx = joystick.get_axis(3)
            #     joystick_data.ry = joystick.get_axis(4)

            # Print data in the specified format
            # print("Button1: {}, Button2: {}, L2: {}, R2: {}, LX: {}, LY: {}, RX: {}, RY: {}".format(
            #     joystick_data.button1, joystick_data.button2, joystick_data.l2, joystick_data.r2,
            #     joystick_data.lx, joystick_data.ly, joystick_data.rx, joystick_data.ry
            # ))

    except KeyboardInterrupt:
        print("Program terminated by user")
    finally:
        pygame.quit()


def main():
    ps4_controller = connect_ps4_controller()

    if ps4_controller is not None:
        parse_and_print_data(ps4_controller)


if __name__ == "__main__":
    main()
