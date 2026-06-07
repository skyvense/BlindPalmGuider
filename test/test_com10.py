import serial
import time

# 尝试多个波特率
baud_rates = [9600, 19200, 38400, 57600, 115200, 230400]

for baud in baud_rates:
    try:
        ser = serial.Serial('COM10', baud, timeout=2)
        # 发送 FE 00 00 07 100 (ASCII)
        msg = bytes([0xFE, 0x00, 0x00, 0x07, 0x31, 0x30, 0x30])  # "100" as ASCII
        ser.write(msg)
        print(f"[{baud}] Sent: FE 00 00 07 100")

        time.sleep(0.5)
        if ser.in_waiting:
            resp = ser.read(ser.in_waiting)
            print(f"[{baud}] Recv: {resp.hex().upper()}")
        else:
            print(f"[{baud}] No response")

        ser.close()
    except Exception as e:
        print(f"[{baud}] Error: {e}")
    time.sleep(0.2)