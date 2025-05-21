import threading
import time
import signal
import sys
from tasks.task1 import send_commands
from tasks.task2 import receive_input
from tasks.task3 import run_ssh_command, stop_ssh_command

import serial
import serial.tools.list_ports

def find_serial_port():
    ports = serial.tools.list_ports.comports()
    for port in ports:
        if '/dev/ttyACM' in port.device:
            return port.device
    return None

import moonrakerpy as moonpy
import paramiko

def copy_files_over_ssh(hostname, port, username, password, local_path, remote_path):
    try:
        # Create an SSH client
        ssh_client = paramiko.SSHClient()
        ssh_client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        ssh_client.connect(hostname, port=port, username=username, password=password)

        # Create an SFTP session
        sftp = ssh_client.open_sftp()

        # Copy the file
        sftp.get(remote_path, local_path)
        print(f"File {remote_path} copied to {local_path}")

        # Close the SFTP session and SSH client
        sftp.close()
        ssh_client.close()
    except Exception as e:
        print(f"Error: {e}")

# Flag to control the execution of Task 2
stop_task2 = threading.Event()

def signal_handler(sig, frame):
    print("Stopping Task 3...")
    stop_task2.set()  # Signal Task 2 to stop
    stop_ssh_command()  # Send Ctrl+C to Task 3
    sys.exit(0)

def main():
    # Setup
    print("Connecting to printer")
    global printer
    printer = moonpy.MoonrakerPrinter('http://192.168.8.112')
    print('Moonraker connected')
    
    print("Press enter to connect to device")
    input()
    print("Connecting to device")
    port = None
    while port is None:
        port = find_serial_port()
        if port is None:
            print("No /dev/ttyACM* port found. Retrying in 5 seconds...")
            time.sleep(5)

    ser = serial.Serial(port, 115200, timeout=1)
    print(f"Connected to {port}")
    
    flag = True
    while flag:
        line = ser.readline().decode('utf-8').strip()
        if line != "":
            print(line)
            if "Starting Tasks" in line:
                print("Ensure printer is clear then press enter to connect")
                input()

                # global printer

                # print('Homing Printer')
                # printer.send_gcode('G28')
                # printer.send_gcode('M400')
                # print('Printer Homed')
                # print('Raising hotend')
                # # printer.send_gcode('G91')
                # printer.send_gcode('G1 Z50 F300')
                # # printer.send_gcode('G90')
                # printer.send_gcode('M400')
                # print('Hotend raised, please attach device to printer, then press enter')
                # input()
                # print('Lowering hotend')
                # printer.send_gcode('G90')
                # printer.send_gcode('G1 Z5 X117 Y154 F6000')
                # printer.send_gcode('M400')
                # print('Hotend lowered, press enter to continue')
                
                # printer.send_gcode('G91')
                # G1 X-100 F6000
                # G1 X+100 F6000
                print("Clearing old motion logs")
                print('Connecting via SSH')
                ssh = paramiko.SSHClient()
                ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
                ssh.connect('192.168.8.112', username='luigipizzolito', password='Palmeiras123')
                print('SSH connected')
                stdin, stdout, stderr = ssh.exec_command('rm -fv ~/mylogpy*')
                print(stdout.read().decode())
                print(stderr.read().decode())
                ssh.close()
                print("Old motion logs cleared")
                print("ready to begin, press enter to start")
                input()
                flag = False
                
    print("Starting tasks")
                
    
    
    
    # Register signal handler for Ctrl+C
    signal.signal(signal.SIGINT, signal_handler)

    # Create threads for each task
    task1_thread = threading.Thread(target=send_commands, args=(printer,))
    task2_thread = threading.Thread(target=receive_input, args=(ser,stop_task2,))
    # task3_thread = threading.Thread(target=run_ssh_command)

    # Start the threads
    task1_thread.start()
    task2_thread.start()
    # task3_thread.start()

    # Wait for Task 1 to finish
    task1_thread.join()
    stop_task2.set()  # Stop Task 2 after Task 1 is done
    task2_thread.join()  # Wait for Task 2 to finish

    # Send Ctrl+C to Task 3
    # stop_ssh_command()
    # task3_thread.join()  # Wait for Task 3 to finish
    
    
    
    
    ser.close()
    print("Done logging device data")
    
    print("Raising hotend")
    printer.send_gcode('G90')
    printer.send_gcode('G1 Z50 X117 Y154 F6000')
    printer.send_gcode('M400')
    for _ in range(30):
        time.sleep(1)
        print(".
              ")
    print("Collecting printer logs")
    copy_files_over_ssh('192.168.8.112', 22, 'luigipizzolito', 'Palmeiras123', './log/mylogpy.json.gz', '/home/luigipizzolito/mylogpy.json.gz')
    copy_files_over_ssh('192.168.8.112', 22, 'luigipizzolito', 'Palmeiras123', './log/mylogpy.index.gz', '/home/luigipizzolito/mylogpy.index.gz')
    print("Printer logs collected")
    # print("Clearing old motion logs")
    # print('Connecting via SSH')
    # ssh = paramiko.SSHClient()
    # ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    # ssh.connect('192.168.8.112', username='luigipizzolito', password='Palmeiras123')
    # print('SSH connected')
    # stdin, stdout, stderr = ssh.exec_command('rm -fv ~/mylogpy*')
    # print(stdout.read().decode())
    # print(stderr.read().decode())
    # ssh.close()
    # print("Old motion logs cleared")
    # print("Removed files from printer")
    

if __name__ == "__main__":
    main()