#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32MultiArray
import pygame
import numpy as np

# Initialize Pygame
pygame.init()
screen_width = 800
screen_height = 600
screen = pygame.display.set_mode((screen_width, screen_height))

def callback(data):
    # Clear the screen
    screen.fill((0, 0, 0))
    
    # Convert Float32MultiArray data to a numpy array
    array_data = np.array(data.data)
    
    # Assuming the data format is [r, g, b, x, y] for a single pixel
    if len(array_data) == 5:
        r, g, b, x, y = array_data
        
        # Clip the coordinates to be within screen bounds
        x = int(np.clip(x, 0, screen_width - 1))
        y = int(np.clip(y, 0, screen_height - 1))
        
        # Draw the pixel
        color = (int(r), int(g), int(b))
        screen.set_at((x, y), color)
        
        # Update the display
        pygame.display.flip()

def listener():
    rospy.init_node('pixel_controller', anonymous=True)
    rospy.Subscriber("pixel_data", Float32MultiArray, callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
    finally:
        pygame.quit()

