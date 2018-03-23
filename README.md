# Prototype-Self-Driving-Car
Self-driving toy car powered by an artificial neural network running on a Raspberry Pi

This project aims to develop a similar concept through a low powered Raspberry Pi running an
Artificial Neural Network (ANN). A Multilayer Perceptron (MLP) Neural Network was created
to train the toy car in order to facilitate the recognition of lane marking and predict the steering
outcome based on the visual information received. An onboard PiCamera was used to detect
street signs and traffic lights. The ‘Haar-like feature’ based cascade classifier was chosen for
object recognition task, which exploits the fact that most similar objects share common
regularities such as shapes and sizes that can be classified into distinct categories. The ultrasonic
sensor complements these machine vision systems to work collectively to perform the driving
behaviours of a self-driving car. ANN based autonomous driving was successfully achieved,
despite the limitations of the Raspberry Pi’s computing power and the car’s mechanical
shortcomings.

This project still needs a lot of work to fully succeed and hopefully with the help of the GitHub community this project can be completed or improved with a different design concept. I hope my detailed report would be of any help to anyone trying to recreate this project.

Lastly, I would like to thank Zheng Wang AKA Hamuchiwa for his guides, support and the inspiration for this project and also Aleonnet for his help and support on some of the problems I faced throughout this project.

Links:
https://zhengludwig.wordpress.com/projects/self-driving-rc-car/
https://github.com/hamuchiwa/AutoRCCar
https://github.com/aleonnet/AutoRCCar-1 

![poster](https://github.com/lakshand8/Prototype-Self-Driving-Car/blob/master/Poster_CM_14148781_Lakshan_Dadigamuwa.png)
