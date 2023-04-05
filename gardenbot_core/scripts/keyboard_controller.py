import time
import serial
import rospy
import sys
# import keyboard
from pynput import keyboard

# from snek_core.common_utils import say_it_works
# from snek_core.common_utils import *
# from snek_msgs.srv import *
# from snek_msgs.msg import *
# from std_srvs.srv import Trigger
# from std_msgs.msg import Float32
from std_msgs.msg import Int16MultiArray

def on_press(key):
    try:
        print('alphanumeric key {0} pressed'.format(
            key.char))
    except AttributeError:
        print('special key {0} pressed'.format(
            key))

def on_release(key):
    print('{0} released'.format(
        key))
    if key == keyboard.Key.esc:
        # Stop listener
        return False

class KeyboardController(object):

# # #{ INIT
    def __init__(self):
        # self.control_freq = rospy.get_param('~control_freq', 10.0) # control loop frequency (Hz)
        self.control_rate = rospy.Rate(20)
        self.vel_r = 0
        self.vel_l = 0

        rospy.loginfo('Keyboard controller initialized.')
        self.keyboard_reading_freq = 100
        self.keyboard_reading_rate = rospy.Rate(self.keyboard_reading_freq)
        self.should_stop = False
        self.is_pressed_w = False
        self.key_press_cache = {}

        self.internal_setpoints = []

        # WAIT FOR CORE AND GET BODY CONFIG

        self.bangbang_bendies_angle = 90.0
        self.bangbang_wheels_speed = 0

        self.selected_bendy = 0

        self.mode_button_pressed = False
        self.mode = "BB"

        self.mainloop()
# # #}


# # #{ KEY PRESS HANDLING
    def is_pressed(self, c):
        if c in self.key_press_cache:
            return self.key_press_cache[c]
        return False
    
    def on_press(self, key):
        try:
            # print('alphanumeric key {0} pressed'.format(
                # key.char))
            self.key_press_cache [key.char] = True
        except AttributeError:
            print('special key {0} pressed'.format(
                key))
    
    def on_release(self, key):
        try:
            self.key_press_cache [key.char] = False
            if key.char == "q":
                # Stop listener
                print("setting stop true")
                self.should_stop = True
                return False
        except AttributeError:
            print('special key {0} released'.format(
                key))

# # #}

    def publish_setpoints(self):
        # self.cmd_pub1 = rospy.Publisher('wheel_left_vel_reference', Float32, queue_size=20) # Publisher of velocity commands
        # self.cmd_pub2 = rospy.Publisher('wheel_right_vel_reference', Float32, queue_size=20) # Publisher of velocity commands
        self.cmd_pub = rospy.Publisher('motor_voltages_raw', Int16MultiArray, queue_size=20) # Publisher of velocity commands
        # msg = ByteMultiArray()
        # signbyte1 = 1 if self.vel_l < 0 else 0
        # signbyte2 = 1 if self.vel_r < 0 else 0
        # velbyte1 = abs(self.vel_l)
        # if velbyte1 > 255:
        #     velbyte1 = 255
        # elif velbyte1 < 0:
        #     velbyte1 = 0
        # velbyte2 = abs(self.vel_r)
        # if velbyte2 > 255:
        #     velbyte2 = 255
        # elif velbyte2 < 0:
        #     velbyte2 = 0
        # msg.data = [signbyte1, int(velbyte1), signbyte2, int(velbyte2)]
        msg = Int16MultiArray()
        msg.data = [int(self.vel_l), int(self.vel_r)]

        # msg1 = Float32()
        # msg1.data = self.vel_l

        # msg2 = Float32()
        # msg2.data = self.vel_r

        # self.cmd_pub1.publish(msg1)
        # self.cmd_pub2.publish(msg2)

        self.cmd_pub.publish(msg)
    
    def update_internal_values_based_on_input(self):
        # main_speed = 600
        main_speed = 120

        if self.mode == "BB":
            if self.is_pressed("w") and not self.is_pressed("d") and not self.is_pressed("a"):
                self.vel_r = main_speed
                self.vel_l = main_speed
            elif self.is_pressed("s") and not self.is_pressed("d") and not self.is_pressed("a"):
                self.vel_r = -main_speed
                self.vel_l = -main_speed
            elif self.is_pressed("a") and not self.is_pressed("d") and not self.is_pressed("w") and not self.is_pressed("s"):
                self.vel_r = main_speed
                self.vel_l = -main_speed
            elif self.is_pressed("d") and not self.is_pressed("a") and not self.is_pressed("w") and not self.is_pressed("s"):
                self.vel_r = -main_speed
                self.vel_l = main_speed
            elif self.is_pressed("w") and self.is_pressed("d"):
                self.vel_r = main_speed * 0.5
                self.vel_l = main_speed 
            elif self.is_pressed("w") and self.is_pressed("a"):
                self.vel_r = main_speed 
                self.vel_l = main_speed * 0.5
            elif self.is_pressed("s") and self.is_pressed("d"):
                self.vel_r = -main_speed * 0.5
                self.vel_l = -main_speed 
            elif self.is_pressed("s") and self.is_pressed("a"):
                self.vel_r = -main_speed 
                self.vel_l = -main_speed * 0.5
            elif not self.is_pressed("w") and not self.is_pressed("a") and not self.is_pressed("s") and not self.is_pressed("d"):
                self.vel_r = 0
                self.vel_l = 0
            # if self.is_pressed("a") and not self.is_pressed("d") and not self.is_pressed("w"):
            #     self.vel_r = main_speed
            #     self.vel_l = main_speed
            # elif self.is_pressed("d") and not self.is_pressed("a") and not self.is_pressed("w"):
            #     self.vel_r = -main_speed
            #     self.vel_l = -main_speed
            # # elif self.is_pressed("w") and not 
            # else:
            #     pass
            #     # self.bangbang_bendies_angle = 90.0

    def get_nonredundant_setpoint_msgs(self, current_setpoints, last_sent_setpoints):
        setpoints_to_send = []
        blocked_msgs = []
        for msg in current_setpoints:
            is_redudnant = False
            for msg2 in last_sent_setpoints:
                if msg2.module_address != msg.module_address:
                    continue
                if msg2.control_type != msg.control_type:
                    continue
                if abs(msg2.setpoint - msg.setpoint) > 0.00001:
                    continue
                is_redudnant = True
                blocked_msgs.append(msg2)
                break
            if not is_redudnant:
                setpoints_to_send.append(msg)
        return setpoints_to_send, blocked_msgs

    def mainloop(self):
        # NONBLOCKING LISTENER
        listener = keyboard.Listener(
            on_press=self.on_press,
            on_release=self.on_release)
        listener.start()

        # LOOP
        while not self.should_stop:
            # print(self.is_pressed("w"), self.is_pressed("a"))
            self.update_internal_values_based_on_input()
            # orig_msgs = self.generate_setpoints_for_all_modules()
            # reduced_msgs, blocked_msgs = self.get_nonredundant_setpoint_msgs(orig_msgs, self.last_sent_setpoints)
            self.publish_setpoints()
            # if len(reduced_msgs) > 0:
            #     rospy.loginfo("SENDING %d MSGS, REDUCED FROM %d COMMANDS", len(reduced_msgs), len(orig_msgs))
            self.control_rate.sleep()

            self.keyboard_reading_rate.sleep()
        rospy.loginfo("exiting keyboard control")


if __name__ == '__main__':
    rospy.init_node('keyboard_controller', log_level=rospy.INFO)
    node = KeyboardController()


