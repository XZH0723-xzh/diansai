"""K230 UART发送测试 (IO10→MSPM0 PA18)"""
from ybUtils.YbUart import YbUart
from libs.YbProtocol import YbProtocol
import time

uart = YbUart(baudrate=115200)
pto = YbProtocol()

print("UART Test - Sending to MSPM0...")

count = 0
while True:
    # 模拟球位数据: x=284, y=119, w=15, h=15
    data = pto.get_object_detect_data(284, 119, 15, 15, "ball")
    uart.send(data)
    count += 1
    print(f"#{count} Sent:", data)
    time.sleep(0.5)
