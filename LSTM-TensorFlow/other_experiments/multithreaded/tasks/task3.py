import time
import paramiko

ssh_client = None
stdin = None

def run_ssh_command():
    global ssh_client
    global stdin
    print("Task 3: Connecting to another host and running SSH command...")
    ssh_client = paramiko.SSHClient()
    ssh_client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    ssh_client.connect('192.168.8.112', username='luigipizzolito', password='Palmeiras123')
    stdin, stdout, stderr = ssh_client.exec_command('python ~/klipper/scripts/motan/data_logger.py ~/printer_data/comms/klippy.sock mylogpy')
    try:
        for line in iter(stdout.readline, ""):
            print(line, end="")
    except KeyboardInterrupt:
        print("Task 3: Received Ctrl+C, stopping...")
    # finally:
    #     print("Task 3: Sending Ctrl+C")
    #     stdin.write(chr(3))
    #     stdin.flush()
    #     print("Task 3: Closing SSH connection")
    #     time.sleep(1)
    #     ssh_client.close()
    #     time.sleep(2)
    print("Task 3: Finished running SSH command.")

def stop_ssh_command():
    if ssh_client:
        print("Task 3: Sending Ctrl+C")
        stdin.write(chr(3))
        stdin.flush()
        print("Task 3: Closing SSH connection")
        time.sleep(1)
        ssh_client.close()
        time.sleep(2)
        print("finished closing ssh connection")