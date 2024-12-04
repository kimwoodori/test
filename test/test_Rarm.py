from pymycobot.mycobot import MyCobot
import time


mycobot = MyCobot('/dev/ttyACM1', 115200)
mycobot.set_gripper_mode(0)
mycobot.init_eletric_gripper()
mycobot.send_angles([0, 0, 0, 0, 0, 0], 100)
time.sleep(2)
mycobot.set_gripper_state(0, 50)
time.sleep(2)
mycobot.send_angles([89.64, 0.7, 4.3, -5.0, -91.14, -4.57], 40)
time.sleep(2)
mycobot.send_angles([88.85, 28.65, 54.49, -60.99, -89.91, -0.61], 20)
time.sleep(4)
mycobot.set_gripper_state(1, 50)
time.sleep(2)
mycobot.send_angles([88.15, -31.46, 98.87, -47.98, -86.74, -0.79], 100)
time.sleep(2)
mycobot.send_angles([154.95, -15.38, 103.27, -50.88, -79.45, 150], 30)
time.sleep(4)


mycobot.send_angles([154.86, -51.06, 96.59, -23.9, -80.77, 0], 50)
time.sleep(2)
mycobot.send_angles([-90, 0, 0, 0, 90, 0], 100)
time.sleep(3)#제자리준비
mycobot.send_angles([-80.85, -79.1, 63.89, 23.9, 39.55, -6.94], 20)
time.sleep(6)
print("컵 제자리")
mycobot.set_gripper_state(0, 50)
time.sleep(4)
print("그리퍼 열기")

mycobot.send_angles([0, 0, 0, 0, 107, 0], 30)
time.sleep(8)
mycobot.send_angles([-10.1, -57.65, -36.05, 102.26, 107.31, 1.4], 20)
time.sleep(5)
mycobot.set_gripper_state(1, 50)
time.sleep(3)
mycobot.send_angles([-13.97, -14.06, -99.75, 108.1, 107.92, 0.7], 20)
time.sleep(5)
mycobot.send_angles([-4.21, -39.46, -89.91, 126.56, 106.69, 1.49], 10)
time.sleep(2)