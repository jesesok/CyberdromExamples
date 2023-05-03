import dataclasses

import threading
from sklearn.cluster import KMeans # pip install sklearn
lock = threading.Lock()
import time
from edubot_sdk.edubot_sdk import EdubotGCS  # Импортируем класс РТС
from piosdk.piosdk import Pioneer  # Импортируем класс Pioneer

class Copter():
    echelon=None
    start_location=None
    location=None
    drone=None
    busy=False
    home=False
    stop=False
    all_mission_complieted=False

    def __init__(self, drone:Pioneer,echelon):
        self.drone=drone
        self.echelon=echelon
        drone.arm()
        while True:
            loc=drone.get_local_position_lps()
            if loc != None:
                break
        self.location = Location(loc[0],loc[1],loc[2])
        self.start_location=Location(loc[0],loc[1],loc[2])

    def set_location(self,loc):
        lock.acquire()
        self.location = Location(loc[0], loc[1], loc[2])
        lock.release()
    def set_cur_location(self):
        while True:
            loc = self.drone.get_local_position_lps()
            if loc != None:
                break
        lock.acquire()
        self.location = Location(loc[0], loc[1], loc[2])
        lock.release()



class Location():
    X=None
    Y=None
    Z=None

    def __init__(self, X, Y,Z):
        self.X=X
        self.Y=Y
        self.Z=Z
        
class ZonePatrul():
    number=None
    points=None

    def __init__(self,n):
        self.points=[]
        self.number=n

    def add_points(self,p):
        self.points.append(p)
zp1=ZonePatrul(1)
zp2=ZonePatrul(2)
zpl=[]
zpl.append(zp1)
zpl.append(zp2)

swap=False

for px in range(-450,450,50):
    if swap==False:
        for py in range(-450,450,50):
            if px <= 0:
                zp1.add_points(Location(X=px/100,Y=py/100,Z=1))
            if (px > 0) :
                zp2.add_points(Location(X=px/100,Y=py/100,Z=1))
a=10
points_interest = []

# Классы для хранения настроек подключения
@dataclasses.dataclass
class IpPort:
    ip: str
    port: int


class DroneConnectingData:
    drone0: IpPort = IpPort(ip="127.0.0.1", port=8000)
    drone1: IpPort = IpPort(ip="127.0.0.1", port=8001)
    # drone2: IpPort = IpPort(ip="127.0.0.1", port=8002)
    # drone3: IpPort = IpPort(ip="127.0.0.1", port=8003)


class RobotConnectingData:
    robot0: IpPort = IpPort(ip="127.0.0.1", port=8004)
    robot1: IpPort = IpPort(ip="127.0.0.1", port=8005)
    # robot2: IpPort = IpPort(ip="127.0.0.1", port=8006)
    # robot3: IpPort = IpPort(ip="127.0.0.1", port=8007)


copters=[]
copters.append( Copter(Pioneer(method=2, ip=DroneConnectingData.drone0.ip, mavlink_port=DroneConnectingData.drone0.port), 1.1))
copters.append( Copter(Pioneer(method=2, ip=DroneConnectingData.drone1.ip, mavlink_port=DroneConnectingData.drone1.port), 1.3))
# код программы...

def patrolling(copter,zp):
    drone = copter.drone
    copter.set_cur_location()

    while copter.location.Z < 0.4:
        drone.arm()
        time.sleep(1)
        drone.takeoff()
        time.sleep(3)
        copter.set_cur_location()


    drone.go_to_local_point(x=copter.location.X, y=copter.location.Y, z=copter.echelon, yaw=0)

    while True:
        if drone.point_reached():
            break

    for p in zp.points:

        drone.go_to_local_point(x=p.X, y=p.Y, z=copter.echelon, yaw=0)

        while True:
            cyrrent_temp = drone.get_piro_sensor_data()
            if cyrrent_temp != None:
                if cyrrent_temp >= 30:
                    while True:
                        curr_pos = drone.get_local_position_lps()
                        if curr_pos!=None:
                            break
                    lock.acquire()
                    points_interest.append([curr_pos[0], curr_pos[1]])
                    lock.release()
            if drone.point_reached():
                break

    lock.acquire()
    copter.busy=False
    copter.all_mission_complieted=True
    copter.set_cur_location()
    #copter.echelon=1+iter*0.4
    lock.release()

first_start=True
while True:
	
    iter=0
    completed_count=0

    for copter in copters:
        if ((not copter.busy) and (not copter.all_mission_complieted)):
            lock.acquire()
            copter.busy = True
            lock.release()
            program_execute_thread = threading.Thread(target=patrolling, args=(copter,zpl[iter],))
            program_execute_thread.start()
            if first_start==True:
                time.sleep(5)

        if (copter.all_mission_complieted):
            completed_count+=1
            
        iter+=1
    # if(not complited_all_fire):
    #     if (completed_count==4):
    #         centre = get_center_clasters(data=points_interest, num_clasters=10)
    #         for c in centre:
    #             centre_fire.append([c[0], c[1], 1])
    #         complited_all_fire=True
    # ck_count=0
    # for ck in centre_complieted:
    #     if ck==1:
    #         ck_count+=1

    # if len(centre_fire)==10:
    #     echelon_start=1.0
    #     if change_echelon:
    #         for copter in copters:
    #             lock.acquire()
    #             copter.echelon=echelon_start
    #             lock.release()
    #             echelon_start+=0.4
    #         change_echelon=False
    #     for copter in copters:
    #         iter_cur=None
    #         icur=0
    #         for a in centre_complieted:
    #             if a==0:
    #                 iter_cur=icur
    #                 break
    #             icur+=1
    #         if ((not copter.busy)):
    #             if iter_cur==None:
    #                 break
    #             lock.acquire()
    #             copter.busy = True
    #             centre_complieted[iter_cur] = 1
    #             lock.release()
    #             program_to_start_thread = threading.Thread(target=fire_detected, args=(copter,centre_fire[iter_cur],iter_cur))
    #             program_to_start_thread.start()
    # if (ck_count ==10):
    #     for copter in copters:
    #         if (not copter.busy)and (not copter.stop):
    #             lock.acquire()
    #             copter.stop =True
    #             copter.busy = True
    #             lock.release()
    #             program_to_start_thread = threading.Thread(target=to_start_mission, args=(copter,))
    #             program_to_start_thread.start()
    #             time.sleep(4)

    first_start=False