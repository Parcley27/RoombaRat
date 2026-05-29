from machine import UART, Pin
import time

BRC_PIN = 4
UART_TX = 17
UART_RX = 16

brc  = Pin(BRC_PIN, Pin.OUT)
uart = UART(2, baudrate=115200, tx=UART_TX, rx=UART_RX)

def _send(data):
    uart.write(bytes(data))

def wake():
    brc.value(1)
    time.sleep_ms(100)
    brc.value(0)
    time.sleep_ms(500)
    brc.value(1)
    time.sleep_ms(100)

def start():
    _send([128])
    time.sleep_ms(200)
    _send([132])
    time.sleep_ms(200)

def drive(velocity, radius=0x8000):
    v = velocity & 0xFFFF
    r = radius   & 0xFFFF
    _send([137, v >> 8, v & 0xFF, r >> 8, r & 0xFF])

def stop():
    drive(0)

wake()
time.sleep_ms(500)
start()

drive(200)
time.sleep_ms(2000)
stop()
