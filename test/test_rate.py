import serial, time

# 测试不同刷新间隔，找最快成功频率
for delay_ms in [100, 200, 300, 500, 1000]:
    success = 0
    for i in range(10):
        try:
            ser = serial.Serial('COM6', 115200, timeout=5)
            ser.dtr = False
            ser.rts = False
            time.sleep(0.5)
            resp = ser.read(3000)
            ser.close()

            h = resp.hex().upper()
            idx = h.find('534F465453455249414C20524543463A20')
            if idx > 0 and '454D53205374617274' in h[idx:idx+60]:
                success += 1
        except:
            pass
        time.sleep(delay_ms / 1000.0)

    print(f'Delay {delay_ms}ms: {success}/10 success')
    if success == 10:
        print(f'  -> Found fastest: {delay_ms}ms ({1000/delay_ms:.1f} Hz)')
        break