import time

from edubot_sdk.edubot_sdk import EdubotGCS  # Импортируем класс РТС
from piosdk.piosdk import Pioneer  # Импортируем класс Pioneer
from threading import Thread

e_1 = EdubotGCS(ip="127.0.0.1", mavlink_port=8000)
e_2 = EdubotGCS(ip="127.0.0.1", mavlink_port=8001)
e_3 = EdubotGCS(ip="127.0.0.1", mavlink_port=8002)

p_1 = Pioneer(method=2, pioneer_ip="127.0.0.1", pioneer_mavlink_port=8003, logger=False)
p_2 = Pioneer(method=2, pioneer_ip="127.0.0.1", pioneer_mavlink_port=8005, logger=False)

e_1.go_to_local_point(5, 5)

while True:
    if e_1.point_reached():
        Thread(target=e_2.go_to_local_point, args=[3, 1]).start()
        Thread(target=e_3.go_to_local_point, args=[2.3, 4]).start()

    if e_2.point_reached():
        p_1.arm()
        p_1.takeoff()
        p_1.led_custom(mode=2, timer=10, color1=[127, 12, 41], color2=[55, 171, 42])
        p_1.go_to_local_point(-4, -4.9, 1.5)

    data = p_1.get_piro_sensor_data(blocking=True)
    if data:
        pos = p_1.get_local_position_lps(blocking=True)
        if pos:
            print(data, pos)

    if p_1.point_reached():
        p_1.land()
        p_2.arm()
        p_2.takeoff()
        p_2.go_to_local_point(4.0, 0, 1.5)
        p_2.led_control(r=200, g=124, b=12)

    data = p_2.get_qr_reader_data(blocking=True)
    if data:
        pos = p_2.get_local_position_lps(blocking=True)
        if pos:
            print(data, pos)

    if p_2.point_reached():
        p_2.land()
        p_2.led_control(r=189, g=34, b=11)

    time.sleep(0.01)
