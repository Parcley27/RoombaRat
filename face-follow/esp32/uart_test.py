from machine import UART
import time

uart = UART(1, baudrate=115200, tx=26, rx=25)
time.sleep_ms(100)

uart.write(b'TEST')
time.sleep_ms(100)
result = uart.read(4)

if result == b'TEST':
    print("UART OK:", result)
else:
    print("UART FAIL — received:", result)
    print("Try UART(2)...")
    uart2 = UART(2, baudrate=115200, tx=26, rx=25)
    uart2.write(b'TEST')
    time.sleep_ms(100)
    result2 = uart2.read(4)
    if result2 == b'TEST':
        print("UART(2) OK — switch to UART(2) in main.py")
    else:
        print("UART(2) also failed — check wiring")
