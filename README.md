
Pimoroni Inky Frame 5.7 with Adafruit Gamepad QT using Micropython and a modified seesaw module

This small project provides an addition to an existing script to display images on the [ Pimoroni Inkey Frame 5.7](https://shop.pimoroni.com/products/inky-frame-5-7?variant=40048398958675) to communicate via I2C with an [Adafruit Gamepad QT](https://www.adafruit.com/product/5743). 
The buttons ```X, Y, A and B``` on the ```Gamepad QT``` are used to switch between two (or more if you want) groups of five images to display on the Inky Frame.
As said above, the communication between the Inky Frame (read: RPi Pico W) and the Gamepad QT is via I2C. See the images in ´/doc_images' for the wiring I used. For the Raspberry Pi Pico W be able to 'talk' with the Gamepad QT this project uses a modified version of the Adafruit Seesaw driver module for Circuitpython, ported to Micropython by Mihai Dinculescu: [seesaw.py](https://github.com/mihai-dinculescu/micropython-adafruit-drivers/blob/master/seesaw/seesaw.py). Mihai's version I modified further to work
with the Adafruit Gamepad QT in Micropython. I had to add some functions from the Adafruit seesaw module. On the other hand I had to take some measures to have this project running well in limited memory space. For this, in the file: ```seesaw_gamepad_qt_mpy_v2.py```, in functions: ```pin_mode_bulk()``` and ```digital_read_bulk()```, I had to add defaults to parameters ```pin``` and ```mode```, so that the calling program doesn't need to hand over these parameters (no need to use stack space).
The buttons ```A``` and ```X``` of the Gamepad QT are used to increase the ```selected group``` of images (there are 5 images per group). The buttons ```B``` and ```Y``` of the Gamepad QT are used to decrease the ```selected group```. The five buttons 'A', 'B', 'C', 'D' and 'E' on the front of the Inky Frame are used to load and display one of the five images of the ```selected group```.

This repo has 10 images in the folder ```/image```. The ```main.py``` script expects the images to be in the SD-Card in folder ```/sd/images```.

Note that, to save memory in the Raspberry Pi Pico W, I put as many as possible files and images onto the SD-Card.
In the main folder (```/Raspberry Pi Pico```) there is also a script named 'boot.py' which connects the SD-Card at boot time.
The script ```main.py``` loads also ```boot.py```.

REQUIREMENTS

Before to be able to run ´main.py´ you need to flash the Raspberry Pi Pico W on the Pimoroni Inky Frame 5.7 with Pimoroni's customized version of [MicroPython micropython-pico](https://github.com/pimoroni/pimoroni-pico/releases/latest/). To download click: [pimoroni-pico micropython](https://github.com/pimoroni/pimoroni-pico/releases/download/v1.21.0/pimoroni-pico-v1.21.0-micropython.uf2).

TROUBLESHOOTING

If, for some reason, after running ```main.py``` you want and don't get access to the SD-Card: run ```boot.py``` from within your IDE (I use: Thonny) and issue 
```
(Thonny) '%cd '/sd' <Enter>
```

Suggestions are appreciated.

