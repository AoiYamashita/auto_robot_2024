import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import time

r_gains = [0.1,0,0.01]
s_gains = [0.1,0,]

class PID:
    def __init__(self,gain,sleep=1):
        self.sleep = sleep
        self.count = 0
        self.gain = gain.copy()
        self.proportional = 0
        self.integral = 0
        self.output = 0
        self.ref = 0#目標値
        self.time = time.perf_counter_ns()
    def cb(self,msg):
        self.output = 0
        self.count += 1
        if self.count % self.sleep != 0:return
        i = time.perf_counter_ns()
        self.integral += (self.ref+msg.data)*((i-self.time)/1e9)/2
        self.output = float(self.gain[0]*(self.ref-msg.data) + self.gain[1]*self.integral - 1e9*self.gain[2]*((self.ref-msg.data)-self.proportional)/(i-self.time))
        print(self.gain[0]*(self.ref-msg.data))
        self.proportional = self.ref-msg.data
        self.time = i
        self.count = 0
    def set_ref(self,ref):
        self.ref = ref

class UnderCarriage(Node):#三輪独ステ
    def __init__(self):
        super().__init__("UnderCarriage")
        self.speedSub = self.create_subscription(Float32,"Speed",self.SpeedCB,10)
        self.angleSub = self.create_subscription(Float32,"Angle",self.AngleCB,10)

        self.rollers = ["red_0","red_1","red_2"]
        self.steers = ["steer0","steer1","steer2"]
        self.rollersPID = [PID(r_gains),PID(r_gains),PID(r_gains)]
        self.steersPID = [PID(s_gains,10),PID(s_gains,10),PID(s_gains,10)]

        self.rollersSub = []
        self.rollersPub = []
        for i in range(3):
            #self.rollersSub.append(self.create_subscription(Float32,self.rollers[i]+"/get_rpm",self.rollersPID[i].cb,10))
            self.rollersPub.append(self.create_publisher(Float32,self.rollers[i]+"/set_power",10))
        
        self.steersSub = []
        self.steersPub = []
        for i in range(3):
            self.steersSub.append(self.create_subscription(Float32,self.steers[i]+"/get_angle",self.steersPID[i].cb,10))
            self.steersPub.append(self.create_publisher(Float32,self.steers[i]+"/set_power",10))
        
        self.timer = self.create_timer(0.001,self.Timer_cb)
    def SpeedCB(self,msg):
        for i in self.rollersPID:
            i.set_ref(msg.data)
    def AngleCB(self,msg):
        for i in self.steersPID:
            i.set_ref(msg.data)
    def Timer_cb(self):
        for i in range(3):
            msg = Float32()
            msg.data = float(self.rollersPID[i].output)
            self.rollersPub[i].publish(msg)
            msg.data = float(self.steersPID[i].output)
            self.steersPub[i].publish(msg)

def main():
    rclpy.init()
    underC = UnderCarriage()
    rclpy.spin(underC)