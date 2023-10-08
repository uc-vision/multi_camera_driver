import time
from machine import UART, Pin, WDT, Timer
import json
import sys
import select
import uasyncio as asyncio


class TimeReference(object):
    
    def __init__(self, cpu_us, year, month, day, hour, min, sec):
        self.cpu_us = cpu_us
        self.year  = year
        self.month = month
        self.day   = day
        self.hour  = hour
        self.min   = min
        self.sec   = sec
        
    def msg(self):
        return json.dumps({
            "type": "reference",
            "year": self.year,
            "month": self.month,
            "day": self.day,
            "hour": self.hour,
            "min": self.min,
            "sec": self.sec,
            "cpu_us": self.cpu_us
        })
    
class NMEAClock(object):
    
    def __init__(self):
        self.in_body = False
        self.msg_buffer = None
        self.reference_time = TimeReference(0,0,0,0,0,0,0)
        
        self.gps_uart = UART(1, baudrate=9600, tx=Pin(8), rx=Pin(9))
        self.isr_last_pps = 0
        self.pps = Pin(18, Pin.IN)
        self.pps.irq(self.pps_isr, trigger=Pin.IRQ_FALLING)
        
        
    def pps_isr(self,pin):
        self.isr_last_pps = time.ticks_us()
        
    def consume_byte(self, c):
        if c is None:
            pass
        elif c == b'$':
            self.msg_buffer = c
            self.in_body = True
            
        elif self.in_body:
            if c == b'\r':
                self.in_body = False
                return True
            else:
                self.msg_buffer += c
        return False

    def get_msg(self):
        return self.msg_buffer.decode("ascii").split(',')
    
    async def run(self):
        while True:
            await asyncio.sleep_ms(1)
            c = self.gps_uart.read(1)
            got_message = self.consume_byte(c)
            if got_message:
                msg = self.get_msg()
                if (msg[0] == '$GPRMC' or msg[0] == '$GNRMC') and self.isr_last_pps != 0:
                    utc_year  = int(msg[9][4:])
                    utc_month = int(msg[9][2:4])
                    utc_day   = int(msg[9][0:2])
                    
                    utc_hour = int(msg[1][0:2])
                    utc_min  = int(msg[1][2:4])
                    utc_sec  = float(msg[1][4:])
                    self.reference_time = TimeReference(self.isr_last_pps, utc_year, utc_month, utc_day, utc_hour, utc_min, utc_sec)
                    self.isr_last_pps = 0
                    
                    #print(self.reference_time.msg())
                    #print(self.sync_age)
      
    @property
    def sync_age(self):
        if self.reference_time.cpu_us == 0:
            return -1
        return time.ticks_diff(time.ticks_us(), self.reference_time.cpu_us) / 1e6
         
class TriggerReporter(object):
    """
    The TriggerReporter is used for report trigger utc time when the
    cameras are self triggering.
    """
    
    def __init__(self, clock):
        self.led = Pin('LED', Pin.OUT)
        self.clock = clock
        
        self.trigger_time = None
        self.last_trigger_time = None
        
        self.trigger_count = 0
        
        self.trigger_pin = Pin(16, Pin.IN)
        self.trigger_pin.irq(self.trigger_in_isr, trigger=Pin.IRQ_RISING)
        
        self.trigger_queue = []
        self.last_trigger_time = None
        self.trigger_count = 0
        
    def trigger_in_isr(self, pin):
        self.trigger_queue.append((time.ticks_us(), pin.value()))
        self.led.toggle()
        
    async def run(self):
        while True:
            await asyncio.sleep_ms(1)
            while len(self.trigger_queue) > 0:
                trigger_time, value = self.trigger_queue.pop(0)
                if self.last_trigger_time != None:
                    fps = 1e6 / (trigger_time - self.last_trigger_time)
                else:
                    fps = 0
                    
                self.last_trigger_time = trigger_time
                
                reference_time = self.clock.reference_time
                reference_delta = time.ticks_diff(trigger_time, reference_time.cpu_us)
                
                trigger_msg = {
                    "type": "trigger",
                    "year": reference_time.year,
                    "month": reference_time.month,
                    "day": reference_time.day,
                    "hour": reference_time.hour,
                    "min": reference_time.min,
                    "sec": reference_time.sec,
                    "delta_us": reference_delta,
                    "fps": fps,
                    "value": value,
                    "sync_age": self.clock.sync_age,
                    "trigger_count": self.trigger_count
                }
                print(json.dumps(trigger_msg))
                self.trigger_count += 1
                
    def reset(self):
        self.trigger_queue = []
        self.last_trigger_time = None
        self.trigger_count = 0
        

    
class Trigger(object):

    
    def __init__(self, clock):
        self.led = Pin('LED', Pin.OUT)
        self.clock = clock
        
        self.trigger_time = None
        self.last_trigger_time = None
        self.target_fps = 0
        
        self.trigger_count = 0
        
        self.trigger_pin = Pin(16, Pin.OUT)
        self.trigger_timer = None
        
        self.set_fps(self.target_fps)
        
            
        
    def trigger_out_isr(self, t):
        self.trigger_pin.on()
        self.trigger_time = time.ticks_us()
        
        self.trigger_clear_timer.init(period=10, mode=Timer.ONE_SHOT, callback=self.trigger_out_clear_isr)
        self.led.toggle()
        
    
    def trigger_out_clear_isr(self,t):
        self.trigger_pin.off()
        self.trigger_count += 1
        
    async def run(self):
        while True:
            await asyncio.sleep_ms(1)
            if self.trigger_time != None:
                if self.last_trigger_time != None:
                    fps = 1e6 / (self.trigger_time - self.last_trigger_time)
                else:
                    fps = 0
                    
                self.last_trigger_time = self.trigger_time
                
                reference_time = self.clock.reference_time
                reference_delta = time.ticks_diff(self.trigger_time, reference_time.cpu_us)
                
                trigger_msg = {
                    "type": "trigger",
                    "year": reference_time.year,
                    "month": reference_time.month,
                    "day": reference_time.day,
                    "hour": reference_time.hour,
                    "min": reference_time.min,
                    "sec": reference_time.sec,
                    "delta_us": reference_delta,
                    "fps": fps,
                    "sync_age": self.clock.sync_age,
                    "trigger_count": self.trigger_count
                }
                print(json.dumps(trigger_msg))
                self.trigger_time = None
                
                
    def _disable_trigger(self):
        if self.trigger_timer is not None:
            self.trigger_timer.deinit()
            self.trigger_timer = None
        
    def _enable_trigger(self):
        if self.target_fps > 0:
            self.trigger_timer = Timer()
            self.trigger_timer.init(mode=Timer.PERIODIC, freq=self.target_fps, callback=self.trigger_out_isr)
            self.trigger_clear_timer = Timer()
                
    def set_fps(self, fps):
        self.target_fps = fps
        if self.trigger_timer is not None:
            self._disable_trigger()
            self._enable_trigger()
                
    def start(self):
        self.trigger_count = 0
        self._disable_trigger()
        self._enable_trigger()
        
    def stop(self):
        self._disable_trigger()
        
    def single(self):
        self._disable_trigger()
        self.trigger_out_isr(None)
        
class HostComm(object):

    def __init__(self, wdt, trigger, reporter):
        self.wdt = wdt
        self.trigger = trigger
        self.reporter = reporter
        self.poll = select.poll()
        self.poll.register(sys.stdin, select.POLLIN)

    def get_msg(self):
        if len(self.poll.poll(0)) > 0:
            try:
                return json.loads(sys.stdin.readline().strip())
            except:
                print("invalid msg")
                return None
        return None
    
    async def run(self):
        while True:
            await asyncio.sleep_ms(10)
            msg = self.get_msg()
            if msg != None:
                if self.wdt is not None:
                    self.wdt.feed()
                    
                if 'trigger' in msg and self.trigger is not None:
                    if msg['trigger'] == 'start':
                        self.trigger.start()
                    elif msg['trigger'] == 'stop':
                        self.trigger.stop()
                        
                if 'fps' in msg and self.trigger is not None:
                    self.trigger.set_fps(msg['fps'])
                    
                if 'reporter' in msg and self.reporter is not None:
                    if msg['reporter'] == 'reset':
                        self.reporter.reset()

        
async def test(wdt, trigger):
    await asyncio.sleep(1)
    
    wdt.feed()
    print("Setting fps to 1fps")
    trigger.set_fps(1)
    trigger.start()
    
    await asyncio.sleep(4)
    
    wdt.feed()

    print("Setting fps to 5fps")
    trigger.set_fps(5)


    await asyncio.sleep(4)
    wdt.feed()

    print("Setting fps to 10fps")
    trigger.set_fps(10)
    
    await asyncio.sleep(4)
    wdt.feed()
    
    print("Stopping Trigger")
    trigger.stop()
    
    await asyncio.sleep(2)
    wdt.feed()
    
    print("single trigger")
    trigger.single()
    
    await asyncio.sleep(2)
    wdt.feed()
    
    print("single trigger")
    trigger.single()
    
async def test2(wdt, trigger):
    await asyncio.sleep(1)
    
    #wdt.feed()
    print("Setting fps to 5fps")
    trigger.set_fps(10)
    trigger.start()
    
    await asyncio.sleep(4)
    
    #wdt.feed()

        
def main():
    wdt = None #WDT(timeout=5000)
    clock = NMEAClock()
    #trigger = Trigger(clock)
    trigger_reporter = TriggerReporter(clock)
    host = HostComm(wdt, None, trigger_reporter)
    
    loop = asyncio.get_event_loop()
    loop.create_task(clock.run())
    #loop.create_task(trigger.run())
    loop.create_task(host.run())
    loop.create_task(trigger_reporter.run())
    #loop.create_task(test2(None, trigger))
    loop.run_forever()
    
main()
    

