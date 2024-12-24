import serial
import time
import keyboard

ser = serial.Serial('COM6', 9600, timeout=1) # Replace COM6 with COM linked with Arduino
time.sleep(2)

def BASE_CW():
    ser.write(b'+')
    time.sleep(0.1)
def BASE_CCW():
    ser.write(b'-')
    time.sleep(0.1)
def JOINT_1_UP():
    ser.write(b'w')
    time.sleep(0.05)

def JOINT_1_DOWN():
    ser.write(b's')
    time.sleep(0.05)

def JOINT_2_UP():
    ser.write(b'a')
    time.sleep(0.05)

def JOINT_2_DOWN():
    ser.write(b'd')
    time.sleep(0.05)

def GRIPPER(): 
    ser.write(b'o')
    time.sleep(0.5)

def PARK():
    ser.write(b'0')
    time.sleep(0.05)

def main():
    print("Press 'w' , 's', 'a', 'd', 'o' & '0' . Press 'q' to quit.")
    try:
        while True:
            if keyboard.is_pressed('+'):  # Check if '+' key is pressed
                BASE_CW()
            if keyboard.is_pressed('-'):  # Check if '-' key is pressed
                BASE_CCW()
            if keyboard.is_pressed('w'):  # Check if 'w' key is pressed
                JOINT_1_UP()
            if keyboard.is_pressed('s'):  # Check if 's' key is pressed
                JOINT_1_DOWN()
            if keyboard.is_pressed('a'):  # Check if 'a' key is pressed
                JOINT_2_UP()
            if keyboard.is_pressed('d'):  # Check if 'd' key is pressed
                JOINT_2_DOWN()
            if keyboard.is_pressed('o'):  # Check if 'o' key is pressed
                GRIPPER()
            if keyboard.is_pressed('0'):  # Check if '0' key is pressed
                PARK()
            elif keyboard.is_pressed('q'):  # Exit if 'q' key is pressed
                print("Exiting...")
                break
    except KeyboardInterrupt:
        print("Program interrupted.")
    finally:
        ser.close()  # Close the serial connection when exiting

if __name__ == "__main__":
    main()