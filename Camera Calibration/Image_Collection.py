import time
import picamera
import os

with picamera.PiCamera() as camera:
    camera.start_preview()
    camera.resolution = (1280, 720)
    camera.framerate = 30
    # Wait for the automatic gain control to settle
    time.sleep(5)
    # Now fix the values
    camera.shutter_speed = camera.exposure_speed
    camera.exposure_mode = 'off'
    g = camera.awb_gains
    camera.awb_mode = 'off'
    camera.awb_gains = g
    camera.color_effects = (128,128)
    # Finally, take several photos with the fixed settings
   # camera.capture_sequence(['frame%02d.jpg' % i for i in range(21)])
   
    for i in range(1, 20):
       camera.capture('frame{0:04d}.jpg'.format(i))
       time.sleep(5)
       
    system('convert -delay 10 -loop 0 image*.jpg')
