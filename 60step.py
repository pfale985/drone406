print('50')
for i in range(20):  # ~3 seconds at 10Hz
    send_attitude_target(thrust=0.5)
    time.sleep(0.1)
