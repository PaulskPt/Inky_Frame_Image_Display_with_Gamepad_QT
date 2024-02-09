# SPDX-FileCopyrightText: 2024 Paulus Schulinck
#
# SPDX-License-Identifier: MIT
###############################
# An offline image gallery that switches between groups of five .jpg images
# on your SD card (copy them across by plugging your SD into a computer).
#
import gc, sys, time
from pimoroni import ShiftRegister
from picographics import PicoGraphics, DISPLAY_INKY_FRAME
from machine import ADC, Pin
from jpegdec import JPEG
import uos
from boot import *
import inky_frame

try:
    from micropython import const
except ImportError:

    def const(x):
        return x

sys.path.append('/sd')
from seesaw.seesaw_gamepad_qt_mpy_v2 import Seesaw

try:
    i2c = inky_frame.I2C(0)
    seesaw = Seesaw(i2c, addr=0x50)
    seesaw.pin_mode_bulk() # (seesaw.INPUT_PULLUP)
    qt_btns_present = True
    print(f"global: seesaw.chip_id= {hex(seesaw.chip_id)}")
except ValueError as e:
    qt_btns_present = False
    raise
except Exception as e:
    print(f"global(): Error while creating an instance seesaw class: {e}")
    qt_btns_present = False
    raise

print(f"global: qt_btns_present = {qt_btns_present}")
# set up the display
gc.collect()
disp = PicoGraphics(display=DISPLAY_INKY_FRAME)

# these are our reference voltages for a full/empty battery, in volts
# the values could vary by battery size/manufacturer so you might need to adjust them
# e.g. for 2xAA or AAA batteries, try max 3.4 min 3.0
full_battery = 4.2
empty_battery = 2.8

gc.collect()

# Create a new JPEG decoder for our PicoGraphics
j = JPEG(disp)  # was (display)
#j = None

# Change to root directory
uos.chdir('/')
"""
fdir = '/sd'
lst = uos.listdir(fdir)
if len(lst) > 0:
    print(f"files on sdCard: {lst}")
else:
    print(f"no files found on {fdir}")
"""

gc.collect()

img_dict = {  # Hard-coded list of image file names
    0: 'jwst1',
    1: 'jwst2',
    2: 'jwst3',
    3: 'jwst4',
    4: 'jwst5',
    5: 'MSFS2020_C337H_',
    6: 'MSFS2020_C337H_2',
    7: 'MSFS2020_KittyHawk_E3',
    8: 'MSFS2020_Pilatus_Porter_v2',
    9: 'MSFS2020_Porter_twilight'
}
nr_groups = len(img_dict) // 5

selected_group = 0 

WIDTH, HEIGHT = disp.get_bounds()
disp.set_font("bitmap8")


# Inky Frame uses a shift register to read the buttons
SR_CLOCK = 8
SR_LATCH = 9
SR_OUT = 10
selected_group = 0

on_battery = None

hold_vsys_en_pin = None

sr = ShiftRegister(SR_CLOCK, SR_LATCH, SR_OUT)

# set up the button LEDs
button_a_led = Pin(11, Pin.OUT)
button_b_led = Pin(12, Pin.OUT)
button_c_led = Pin(13, Pin.OUT)
button_d_led = Pin(14, Pin.OUT)
button_e_led = Pin(15, Pin.OUT)

if qt_btns_present:
    btn_press_counter = 0
else:
    btn_press_counter = None

# the built-in LED of the Raspberry Pi Pico W
conn_led = Pin(7, Pin.OUT) 
bi_led = Pin("LED", Pin.OUT)
# and the activity LED
activity_led = Pin(6, Pin.OUT)
activity_led.value(0)
activity_led_state = 0

gc.collect()

#
#  blink_activity_led
#
def blink_activity_led(nr_times, activ_led=False, blink_slow=True):
    global activity_led_state
    curr_state = activity_led_state
    if nr_times is None:
        nr_times = 1
        
    if blink_slow:
        delay = 50  # was 0.5
    else:
        delay = 10  # was 0.1
    
    if curr_state == 1:
        if activ_led:
            activity_led.value(0)  # first switch the led off
        conn_led.value(0)
        bi_led.off()
        time.sleep_ms(delay)
    
    for _ in range(nr_times):
        if activ_led:
            activity_led.value(1)
        conn_led.value(1)
        bi_led.on()
        time.sleep_ms(delay)
        if activ_led:
            activity_led.value(0)
        conn_led.value(0)
        bi_led.off()
        time.sleep_ms(delay)
        
    if curr_state == 1:  # if the led originally was on, switch it back on
        if activ_led:
            activity_led.value(1)
        conn_led.value(1)
        bi_led.on()   

def display_image(nr, filename):
    retval = True
    try:
        if nr != 99:
            print("Opening image file: {:3d} \'{:s}\'".format(nr, filename))
        # Open the JPEG file
        j.open_file(filename)
        gc.collect()
        # Decode the JPEG
        j.decode() # (0, 0, JPEG_SCALE_FULL)
        # Display the result
        disp.update()
    except OSError as e:
        print(f"Could not open file: error {e} occurred")
        retval = False
    return retval
    
def disp_text(txt):
    disp.set_pen(1)
    disp.clear()
    disp.set_pen(0)
    disp.text(txt, 10, HEIGHT//2-1, scale=3)
    disp.update()
    gc.collect()

# Check for button presses on the Gamepad QT
def ck_qt_btns():
    global selected_group, activity_led_state
    
    if not qt_btns_present:
        return
    
    BUTTON_SELECT = const(0)
    BUTTON_B = const(1)
    BUTTON_Y = const(2)
    BUTTON_A = const(5)
    BUTTON_X = const(6)
    BUTTON_START = const(16)
    
    TAG = "ck_qt_btns(): "
    s_btn = "Button "
    s_pre = " pressed"
    btns = ["X", "Y", "A", "B"]
    btn_names = ["BUTTON_X", "BUTTON_Y", "BUTTON_A", "BUTTON_B"]

    gc.collect()
    time.sleep(0.2)
    # Get the button presses, if any...
    buttons = seesaw.digital_read_bulk() # was: (button_mask)
    time.sleep(0.5)
    # print("\n"+TAG+f"buttons = {buttons}")
    res = -1

    for _ in range(len(btns)):
        if _ == 0:
            bz = 1 << BUTTON_X
            if not (buttons & bz):
                btnX = True
                res = _
            else:
                btnX = False
        elif _ == 1:
            bz = 1 << BUTTON_Y
            if not (buttons & bz):
                btnY = True
                res = _
            else:
                btnY = False
        elif _ == 2:
            bz = 1 << BUTTON_A
            if not (buttons & bz):
                btnA = True
                res = _
            else:
                btnA = False
        elif _ == 3:
            bz = 1 << BUTTON_B
            if not (buttons & bz):
                btnB = True
                res = _
            else:
                btnB = False
        else:
            btnX = btnY = btnA = btnB = False
            res = -1

    if res >= 0 and res <= 3:
        blink_activity_led(2, activ_led=True, blink_slow=False)
        print(s_btn+btns[res]+s_pre)
        # print(TAG+f"res = {res}")

        if btnX or btnA:     
            selected_group += 1  # goto next group
            if selected_group > nr_groups -1:
                selected_group = 0  # Goto first group
            print(TAG+f"Group nr inreased. New selected group = {selected_group}")
            blink_activity_led(nr_groups)  # show the choosen group by blinking the activity led
        if btnY or btnB:
            selected_group -= 1 # goto previous group
            if selected_group < 0:
                selected_group = nr_groups -1 # goto last group
            print(TAG+f"Group nr decreased. New selected group = {selected_group}")
            blink_activity_led(nr_groups)  # show the choosen group by blinking the activity led
        res = -1
        
        activity_led.value(0)
        conn_led.value(0)
        activity_led_state = 0
        
        
def setup():
    global hold_vsys_en_pin, activity_led_state, on_battery

    
    print("\nImage gallery SD modified example for Pimoroni Inky Frame")
    if qt_btns_present :
        print("Gamepad QT buttons A or X <<< Group index >>> buttons B or Y")
    print("Press button A...E to display an image")

    gc.collect()  # Claw back some RAM!
    # set up and enable vsys hold so we don't go to sleep
    HOLD_VSYS_EN_PIN = 2

    hold_vsys_en_pin = Pin(HOLD_VSYS_EN_PIN, Pin.OUT)
    hold_vsys_en_pin.value(True)
    
    # set up the ADC that's connected to the system input voltage
    vsys = ADC(29)

    # on a Pico W we need to pull GP25 high to be able to read vsys
    # this is because this pin is shared with the wireless module's SPI interface
    spi_output = (Pin(25, Pin.OUT))
    spi_output.value(True)

    # how we convert the reading into a voltage
    conversion_factor = 3 * 3.3 / 65535
         
    # convert the raw ADC read into a voltage, and then a percentage
    voltage = vsys.read_u16() * conversion_factor
    percentage = 100 * ((voltage - empty_battery) / (full_battery - empty_battery))
    if percentage > 100:
        percentage = 100.00

    # monitoring vbus tells us if Inky is being USB powered
    # it seems to not always identify USB power correctly when run through Thonny?
    vbus = Pin('WL_GPIO2', Pin.IN)
    
    # add text
    if vbus.value():
        on_battery = False
        print('USB power!')
    else:
        on_battery = True
        blink_activity_led(5, True, False) # blink the CONN LED, the RPi PICOW LED and the ACTIVITY LED. Blink fast
        print('Battery power!')
        print('{:.2f}'.format(voltage) + "v")
        print('{:.0f}%'.format(percentage))
        
    # setup
    activity_led.value(1)
    activity_led_state = 1

# Check for button presses on the Inkey Frame
def ck_btns():
    global button_a, button_b, button_c, button_d, button_e
    button_a_led.off()
    button_b_led.off()
    button_c_led.off()
    button_d_led.off()
    button_e_led.off()

    # read the shift register
    # we can tell which button has been pressed by checking if a specific bit is 0 or 1
    result = sr.read()  # result usually = 128. When button_a .. button_e is pressed result = 129..133
    
    # print(f"result= {result}")
    button_a = sr[7]
    button_b = sr[6]
    button_c = sr[5]
    button_d = sr[4]
    button_e = sr[3]

    # light up the activity LED when Inky is awake
    activity_led.value(1)
    activity_led_state = 1
    idx = -1 # force key not found if no button was pressed

    if button_a == 1:
        button_a_led.on()
        # IMAGE_A
        return 0
    if button_b == 1:
        button_b_led.on()
        # IMAGE_B
        return 1
    if button_c == 1:
        button_c_led.on()
        # IMAGE_C
        return 2
    if button_d == 1:
        button_d_led.on()
        # IMAGE_D
        return 3
    if button_e == 1:
        button_e_led.on()
        # IMAGE_E
        return 4
    return idx

def main():
    global red_int_flag, blu_int_flag, sd, selected_group
    TAG= "main(): "
    setup()
    gc.collect()
    files_shown_dict = {}
    curr_img = ""
    msg1_shown = False
    msg2_shown = False
    msg3_shown = False
    disp_fail = False
    print(TAG+f"Current selected group = {selected_group}")
    #loop_nr = 0
    while True:
        try:
            #loop_nr += 1
            #if loop_nr >= 100:
            #    loop_nr = 0
            #print(TAG+f"loop nr: {loop_nr}")
            ck_qt_btns()  # Check button presses from the Gamepad QT
            gc.collect()
            idx = ck_btns() # Check button presses on the Inky Frame

            if idx in [0, 1, 2, 3 ,4]:  # handle only if a button was pressed
                if not msg1_shown:
                    print(TAG+f"index of keypress is: {idx}")
                    msg1_shown = True
                idx2 = ((selected_group) * 5) + idx
                if not msg2_shown and not disp_fail:
                    print(TAG+f"Going to show image whos index is: {idx2}")
                    msg2_shown = True
                if idx2 in img_dict.keys():
                    fn = '/sd/images/'+img_dict[idx2]+'.jpg'
                    if fn == curr_img:
                        if not msg3_shown:
                            print(f"Image \'{curr_img}\' is being displayed")
                            msg3_shown = True
                        idx = -1
                    else:
                        msg_shown = False
                        curr_img = fn
                        disp_fail = display_image(idx2+1, fn)
                        # go to sleep if on battery power
                        activity_led.value(0)
                        conn_led.value(0)
                        activity_led_state = 0
                        if on_battery:
                            # go to sleep until someone pushes a button
                            # (this will only work if code is saved as main.py and on battery power)
                            hold_vsys_en_pin.init(Pin.IN)
                        if not idx2 in files_shown_dict.keys():
                            files_shown_dict[idx2] = curr_img
                        if len(files_shown_dict) == 5:
                            print(f"Group {grp_idx} images shown:")
                            for k,v in files_shown_dict.items():
                                print("{:3d} {:s}".format(k+1, v))
                            grp_idx += 1
                            if (grp_idx+1) * 5 > len(img_dict):
                                grp_idx = 0
                                print(f"All images have been displayed.\nContinuing with the first group.")
                            else:
                                print(f"All images of group {grp_idx} have been displayed.\nContinuing with the next group.")
                            files_shown_dict = {}  # we start a new series
        except KeyboardInterrupt:
            print("KeyboardInterrupt. Exiting...")
            try:
                #uos.umount('/sd')
                activity_led.value(0)
                conn_led.value(0)
                break
            except OSError as e:
                print(f"Error: {e}")
                raise
    sys.exit()

    # machine.reset()
                        
if __name__ == '__main__':
    main()

