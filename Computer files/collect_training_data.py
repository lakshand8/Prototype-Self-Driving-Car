#from Adafruit_MotorHAT import Adafruit_MotorHAT, Adafruit_DCMotor
#import RPi.GPIO as GPIO
import numpy as np
import cv2
import serial
import pygame
from pygame.locals import *
import socket
import time
import os
import atexit


class CollectTrainingData(object):

    def __init__(self):
        pygame.init()
        pygame.display.set_mode((100, 100), HWSURFACE | DOUBLEBUF | RESIZABLE)
        self.server_socket = socket.socket()
        self.server_socket.bind(('192.168.x.xxx', 8070)) #localhost ip address
        self.server_socket.listen(0)

        # accept a single connection
        self.connection = self.server_socket.accept()[0].makefile('rb')

        # connect to a serial port
        self.send_inst = True

        # create labels
        self.k = np.zeros((4, 4), 'float')
        for i in range(4):
            self.k[i, i] = 1
        self.temp_label = np.zeros((1, 4), 'float')

        self.collect_image()

    def collect_image(self):

        saved_frame = 0
        total_frame = 0

        # collect images for training
        print 'Start collecting images...'
        e1 = cv2.getTickCount()
        image_array = np.zeros((1, 38400))
        label_array = np.zeros((1, 4), 'float')

        # stream video frames one by one
        try:
            stream_bytes = ' '
            frame = 1
            while self.send_inst:
                stream_bytes += self.connection.read(1024)
                first = stream_bytes.find('\xff\xd8')
                last = stream_bytes.find('\xff\xd9')
                if first != -1 and last != -1:
                    jpg = stream_bytes[first:last + 2]
                    stream_bytes = stream_bytes[last + 2:]
                    image = cv2.imdecode(np.fromstring(jpg, dtype=np.uint8), cv2.IMREAD_GRAYSCALE)

                    # select lower half of the image
                    roi = image[120:240, :]

                    # save streamed images
                    cv2.imwrite('training_images/frame{:>05}.jpg'.format(frame), image)

                    #cv2.imshow('roi_image', roi)
                    cv2.imshow('image', image)

                    # reshape the roi image into one row array
                    temp_array = roi.reshape(1, 38400).astype(np.float32)

                    frame += 1
                    total_frame += 1

                    # get input from human driver
                    for event in pygame.event.get():
                        if event.type == KEYDOWN:
                            key_input = pygame.key.get_pressed()

                            # complex orders
                            if key_input[pygame.K_UP] and key_input[pygame.K_RIGHT]:
                                print("Forward Right")
                                image_array = np.vstack((image_array, temp_array))
                                label_array = np.vstack((label_array, self.k[1]))
                                saved_frame += 1
                                # rearMotor.run(Adafruit_MotorHAT.FORWARD)
                                # frontMotor.run(Adafruit_MotorHAT.FORWARD)

                            elif key_input[pygame.K_UP] and key_input[pygame.K_LEFT]:
                                print("Forward Left")
                                image_array = np.vstack((image_array, temp_array))
                                label_array = np.vstack((label_array, self.k[0]))
                                saved_frame += 1
                                # rearMotor.run(Adafruit_MotorHAT.FORWARD)
                                # frontMotor.run(Adafruit_MotorHAT.BACKWARD)

                            elif key_input[pygame.K_DOWN] and key_input[pygame.K_RIGHT]:
                                print("Reverse Right")
                                # rearMotor.run(Adafruit_MotorHAT.BACKWARD)
                                # frontMotor.run(Adafruit_MotorHAT.FORWARD)

                            elif key_input[pygame.K_DOWN] and key_input[pygame.K_LEFT]:
                                print("Reverse Left")
                                # rearMotor.run(Adafruit_MotorHAT.BACKWARD)
                                # frontMotor.run(Adafruit_MotorHAT.BACKWARD)

                            # simple orders
                            elif key_input[pygame.K_UP]:
                                print("Forward")
                                saved_frame += 1
                                image_array = np.vstack((image_array, temp_array))
                                label_array = np.vstack((label_array, self.k[2]))
                                #rearMotor.run(Adafruit_MotorHAT.FORWARD)

                            elif key_input[pygame.K_DOWN]:
                                print("Reverse")
                                saved_frame += 1
                                image_array = np.vstack((image_array, temp_array))
                                label_array = np.vstack((label_array, self.k[3]))
                                #rearMotor.run(Adafruit_MotorHAT.BACKWARD)

                            elif key_input[pygame.K_RIGHT]:
                                print("Right")
                                image_array = np.vstack((image_array, temp_array))
                                label_array = np.vstack((label_array, self.k[1]))
                                saved_frame += 1
                               # frontMotor.run(Adafruit_MotorHAT.FORWARD)

                            elif key_input[pygame.K_LEFT]:
                                print("Left")
                                image_array = np.vstack((image_array, temp_array))
                                label_array = np.vstack((label_array, self.k[0]))
                                saved_frame += 1
                               # frontMotor.run(Adafruit_MotorHAT.BACKWARD)

                            elif key_input[pygame.K_x] or key_input[pygame.K_q]:
                                print 'exit'
                                self.send_inst = False
                                break

                        elif event.type == pygame.KEYUP:
                            #rearMotor.run(Adafruit_MotorHAT.RELEASE)
                            #frontMotor.run(Adafruit_MotorHAT.RELEASE)

                            # save training images and labels
                            train = image_array[1:, :]
                            train_labels = label_array[1:, :]

                            # save training data as a numpy file
                            file_name = str(int(time.time()))
                            directory = "training_data"
                            if not os.path.exists(directory):
                                os.makedirs(directory)
                            try:
                                np.savez(directory + '/' + file_name + '.npz', train=train, train_labels=train_labels)
                            except IOError as e:
                                print(e)

                            e2 = cv2.getTickCount()
                            # calculate streaming duration
                            time0 = (e2 - e1) / cv2.getTickFrequency()
                            print 'Streaming duration:', time0

                            print(train.shape)
                            print(train_labels.shape)
                            print 'Total frame:', total_frame
                            print 'Saved frame:', saved_frame
                            print 'Dropped frame', total_frame - saved_frame

        finally:
            self.connection.close()
            self.server_socket.close()

if __name__ == '__main__':
    CollectTrainingData()
