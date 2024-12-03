import spidev

print("ready go")

import time

spi = spidev.SpiDev()

try:
    print(" open the spi")
    spi.open(0,0)
    spi.max_speed_hz = 500000
    print("spi inital successful")

    send_data = [0, 127]
    print(f"zhunbeishuju: {send_data}")
    
    if not all(isinstance(x,int) for x in send_data):
        raise ValueEooro("send_data xxxx!")
    
    received_data = spi.xfer2(send_data)
    print(f"fa song: {send_data}")
    print(f"RECEIVE: {received_data}")

except Exception as e:
    print(f"SPi fall: {e}")

finally:
    spi.close()
    print("SPI close")
