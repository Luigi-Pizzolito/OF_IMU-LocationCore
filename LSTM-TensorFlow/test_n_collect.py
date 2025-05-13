import serial
import csv

import moonrakerpy as moonpy
import paramiko
import threading

import serial.tools.list_ports

def find_serial_port():
    ports = serial.tools.list_ports.comports()
    for port in ports:
        if '/dev/ttyACM' in port.device:
            return port.device
    return None

def read_from_port(ser, output_file, printer):
    with open(output_file, 'w', newline='') as csvfile:
        csvwriter = csv.writer(csvfile)
        started_cap = False
        flag = False
        while True:
            line = ser.readline().decode('utf-8').strip()
            if line != "":
                print(line)
                if "Starting Tasks" in line:
                    print("Ensure printer is clear then press enter to connect")
                    input()

                    # global printer

                    print('Homing Printer')
                    printer.send_gcode('G28')
                    printer.send_gcode('M400')
                    print('Printer Homed')
                    print('Raising hotend')
                    # printer.send_gcode('G91')
                    printer.send_gcode('G1 Z50 F300')
                    # printer.send_gcode('G90')
                    printer.send_gcode('M400')
                    print('Hotend raised, please attach device to printer, then press enter')
                    input()
                    print('Lowering hotend')
                    printer.send_gcode('G90')
                    printer.send_gcode('G1 Z5 X117 Y154 F6000')
                    printer.send_gcode('M400')
                    print('Hotend lowered, press enter to continue')
                    # printer.send_gcode('G91')
                    # G1 X-100 F6000
                    # G1 X+100 F6000




                    # print('Connecting via SSH')
                    # ssh = paramiko.SSHClient()
                    # ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
                    # ssh.connect('192.168.8.112', username='luigipizzolito', password='Palmeiras123')
                    # print('SSH connected')
                    print("please connect over ssh")
                    print("ssh luigipizzolito@192.168.1.112")
                    input()

                    # stdin, stdout, stderr = ssh.exec_command('neofetch')
                    # print(stdout.read().decode())
                    # ssh.close()

                    print("removing old motion log files")
                    print("please run the following command on the printer")
                    print("rm -fv ~/mylogpy*")
                    print("then press enter")
                    input()
                    # stdin, stdout, stderr = ssh.exec_command('rm -fv ')
                    # print(stdout.read().decode())
                    # print(stderr.read().decode())

                    print("starting printer motion logging")
                    print("please run the following command on the printer")
                    print("python ~/klipper/scripts/motan/data_logger.py ~/printer_data/comms/klippy.sock mylogpy")
                    print("then press enter")
                    input()
                    # stdin, stdout, stderr = ssh.exec_command('python ~/klipper/scripts/motan/data_logger.py ~/printer_data/comms/klippy.sock mylogpy')
                    # print(stdout.read().decode())
                    # print(stderr.read().decode())
                    # input()
                    # ssh.close()
    
                    print("ready to begin, press enter to start")
                    input()
                    flag = False

                    def run_printer_tasks(printer):
                        def printer_tasks():
                            global flag
                            printer.send_gcode('G91')
                            for i in range(5):
                                printer.send_gcode('G1 X-100 F3000')
                                printer.send_gcode('M400')
                                printer.send_gcode('G1 X+100 F3000')
                                printer.send_gcode('M400')
                            printer.send_gcode('G90')
                            flag = True
                        printer_thread = threading.Thread(target=printer_tasks)
                        printer_thread.start()
                        return printer_thread
                            
                    ser.write(b'a')
                    run_printer_tasks(printer)
                    
                if flag:
                    csvfile.close()
                    return
                
                if "FLAG" in line:
                    started_cap = True
                    csvwriter.writerow("millis,accel_x,accel_y,accel_z,quat_w,quat_x,quat_y,quat_z,of_x,of_y".split(','))
                    csvfile.flush()
                    continue
                if started_cap:
                    csvwriter.writerow(line.split(','))
                    csvfile.flush()

        

def main():
    
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




    read_from_port(ser, 'log/device.csv', printer)
    
    print("Done logging device data")
    
    print("Raising hotend")
    printer.send_gcode('G90')
    printer.send_gcode('G1 Z50 X117 Y154 F6000')
    printer.send_gcode('M400')
    
    print("Collecting printer logs")

if __name__ == "__main__":
    main()
