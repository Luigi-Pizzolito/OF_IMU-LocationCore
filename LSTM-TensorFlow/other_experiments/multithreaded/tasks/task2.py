import csv

def receive_input(ser, stop_event):
    print("Task 2: Receiving input from serial port...")
    with open('./log/device.csv', 'w', newline='') as csvfile:
        csvwriter = csv.writer(csvfile)
        ser.write(b'a')
        started_cap = False
        print("now motion logging device data")
        while not stop_event.is_set():
            # Simulate receiving input
            
            line = ser.readline().decode('utf-8').strip()
            if line != "":
                # print(line)
                if "FLAG" in line:
                    started_cap = True
                    csvwriter.writerow("millis,accel_x,accel_y,accel_z,quat_w,quat_x,quat_y,quat_z,of_x,of_y".split(','))
                    csvfile.flush()
                    print("Started capturing data")
                    continue
                if started_cap:
                    csvwriter.writerow(line.split(','))
                    csvfile.flush()
        print("Task2: Halting device data capture")
        csvfile.flush()
        csvfile.close()
    print("Task 2: Stopped receiving input.")