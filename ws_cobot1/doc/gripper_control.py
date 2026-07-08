import time
from pymodbus.client import ModbusTcpClient

# 1. Connect to the OnRobot Compute Box
COMPUTE_BOX_IP = "192.168.1.1" # Replace with your actual Compute Box IP
client = ModbusTcpClient(host=COMPUTE_BOX_IP, port=502)

if not client.connect():
    print("Failed to connect to the Compute Box.")
    exit()
    
# 2. Activate the Gripper
# Write 0x01 to Register 0 (Action Request) to start initialization
client.write_register(0, 0x01)
time.sleep(2) # Wait for initialization to complete

# 3. Set Width and Force
target_width = 500  # 50.0 mm
target_force = 100  # 10 N
client.write_register(1, target_width)
client.write_register(2, target_force)

# 4. Command the Gripper to Grip
# Write 0x08 (Grip command) to Register 0
client.write_register(0, 0x08)
time.sleep(2) # Wait for grip

# 5. Read Actual Width (if needed)
result = client.read_holding_registers(address=0x06, count=1)
if not result.isError():
    print(f"Current Gripper Width: {result.registers[0] / 10.0} mm")

# Close connection
client.close()
