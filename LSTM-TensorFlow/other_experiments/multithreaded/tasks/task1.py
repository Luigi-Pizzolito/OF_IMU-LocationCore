import time
import pyautogui

def type_and_press_enter(text):
    # Give some time to switch to the desired window
    # time.sleep(2)
    
    # Type the given text
    pyautogui.typewrite(text)
    
    # Press Enter
    pyautogui.press('enter')
    
def send_commands(printer):
    time.sleep(1)
    print("Task 1: Sending commands over API...")
    printer.send_gcode('G91')
    for i in range(3):
        # printer.send_gcode('G1 X-100 F6000')
        # printer.send_gcode('M400')
        # printer.send_gcode('G1 X+100 F6000')
        # printer.send_gcode('M400')
        type_and_press_enter('G1 X-100 F6000')
        time.sleep(1)
        type_and_press_enter('G1 X+100 F6000')
        time.sleep(1)
    printer.send_gcode('G90')
    time.sleep(1)  # Simulate task duration
    print("Task 1: Finished sending commands.")