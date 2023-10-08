import serial
import queue
import json
import threading
import time
import datetime
import rospy2 as rospy

class TriggerReporter(threading.Thread):

    def __init__(self, path):
        super().__init__()
        self.serial = serial.Serial(path, 115200, timeout=0)
        self.buffer = b""
        self.last_kl = 0
        self.running = True
        self.trigger_callback = None

        
    def run(self):
        while self.running:
            reading = True
            while reading:
                byte = self.serial.read(1)

                if byte == b"":
                    reading = False
                if byte == b"\n":
                    reading = False
                    self.buffer += byte
                else:
                    self.buffer += byte

            if self.buffer.endswith(b"\n"):

                try:
                    msg = json.loads(self.buffer.decode('ascii'))
                    if "type" in msg and msg["type"] == "trigger":
                        self.trigger_handler(msg)

                except BaseException as e:
                    rospy.logerr(e)
                    rospy.logerr(f"Invalid msg {self.buffer}")
                self.buffer = b""

            if time.time() - self.last_kl > 1.0:
                self.send_msg({"keep_alive":True})
                self.last_kl = time.time()

    def trigger_handler(self, msg):
        if abs(int(msg["sec"]) - msg["sec"]) > 0:
            raise NotImplementedError("Subsecond timestamps not supported")
        reference = datetime.datetime(2000 + msg["year"], msg["month"], msg["day"], msg["hour"], msg["min"], int(msg["sec"]))
        delta = datetime.timedelta(microseconds=msg["delta_us"])
        timestamp = reference + delta
        if self.trigger_callback is not None:
            self.trigger_callback(msg["trigger_count"], timestamp)
        else:
            rospy.loginfo(f"Trigger {msg['trigger_count']} at {timestamp}")

    def send_msg(self,msg):
        self.serial.write(f"{json.dumps(msg)}\n".encode())

    def reset_count(self):
        self.send_msg({'reporter': 'reset'})
    
    
    def stop(self):
        self.runnning = False
    

if __name__ == "__main__":
    tr = TriggerReporter("/dev/serial/by-id/usb-MicroPython_Board_in_FS_mode_e660d05113943d35-if00")
    tr.start()
