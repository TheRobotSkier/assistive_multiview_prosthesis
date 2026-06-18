from dynamixel_sdk import PortHandler, PacketHandler, COMM_SUCCESS
import time

port = PortHandler("/dev/ttyUSB1")
pkt = PacketHandler(2.0)
port.openPort()
port.setBaudRate(57600)
motor_id = 1

# Read current position
pos, res, _ = pkt.read4ByteTxRx(port, motor_id, 132)
print(f"Initial position: {pos} ({pos/4095*360:.1f} deg)")

# Read operating mode
mode, res, _ = pkt.read1ByteTxRx(port, motor_id, 11)
print(f"Operating mode: {mode} (1=Velocity, 3=Position)")

# Read torque enable
te, res, _ = pkt.read1ByteTxRx(port, motor_id, 64)
print(f"Torque enable: {te}")
if te == 0:
    print("Enabling torque...")
    pkt.write1ByteTxRx(port, motor_id, 64, 1)
    time.sleep(0.1)

# Try velocity control
print("Testing VELOCITY: writing goal velocity = 50")
pkt.write4ByteTxRx(port, motor_id, 104, 50)
for i in range(30):
    time.sleep(0.1)
    if i % 5 == 0:
        pos, res, _ = pkt.read4ByteTxRx(port, motor_id, 132)
        print(f"  t={i*0.1:.1f}s  pos={pos} ({pos/4095*360:.1f} deg)")

print("Stopping...")
pkt.write4ByteTxRx(port, motor_id, 104, 0)
time.sleep(0.5)
pos, res, _ = pkt.read4ByteTxRx(port, motor_id, 132)
print(f"Final position: {pos} ({pos/4095*360:.1f} deg)")

port.closePort()
