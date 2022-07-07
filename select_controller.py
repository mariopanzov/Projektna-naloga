import airsim
import torch
import socket
import serial.tools.list_ports
import numpy
import keyboard

# # TCP python to Unity C#
host = "127.0.0.1"
CUSER_SOCKET = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
CUSER_SOCKET.connect((host, 1234))

YOLO_TARGET_SOCKET = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
YOLO_TARGET_SOCKET.connect((host, 2345))

CAMERA_SOCKET = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
CAMERA_SOCKET.connect((host, 3456))

AI_SOCKET = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
AI_SOCKET.connect((host, 4567))

POV_SOCKET = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
POV_SOCKET.connect((host, 5678))

# # YOLO Model
model = torch.hub.load('ultralytics/yolov5', 'yolov5s')
model.conf = 0.23
model.classes = [0]

# connect to the AirSim
client = airsim.CarClient()
client.confirmConnection()

# car controls
car_controls = airsim.CarControls()

# read from serial port
ports = list(serial.tools.list_ports.comports())
connectedPort = serial.Serial()

list_ports = []

for p in ports:
    list_ports.append(str(p))
    print(str(p))

a = input("select a Port: COM")
for i in range(0,len(list_ports)):
    if list_ports[i].startswith("COM"+str(a)):
        port_a = "COM" + str(a)
        print("--> Port " + port_a + " selected")

connectedPort = serial.Serial(port_a, 9600, timeout=0.05)

POV_CAR = "car"
POV_TAR = "tar"
USER = "u"
STM32 = "s"
AI = "a"
YES = "y"
NO = "n"
CAMERAMODE_ON = "c"
CAMERAMODE_OFF = "v"
AI_MODE_ON = "p"
AI_MODE_OFF = "l"
CHOICE = 0
FIRST_CYCLE = True
TERMINATOR = False
OK = 'OK'
get0123 = False

while True:
    if CHOICE == 0:
        if FIRST_CYCLE == True:
            print("user")
            POV_SOCKET.sendall(POV_CAR.encode("UTF-8"))            
            CUSER_SOCKET.sendall(USER.encode("UTF-8"))
            AI_SOCKET.sendall(AI_MODE_OFF.encode("UTF-8"))
            # client.reset()
            client.enableApiControl(False)
            FIRST_CYCLE = False

    elif CHOICE == 1:
        print("stm32")
        POV_SOCKET.sendall(POV_CAR.encode("UTF-8"))
        CUSER_SOCKET.sendall(STM32.encode("UTF-8"))
        # client.reset()
        client.enableApiControl(True)
        # if(client.isApiControlEnabled()):
        while True:
            if connectedPort.in_waiting:
                IMU_DATA = connectedPort.readline()
                if IMU_DATA.decode('utf').find('PITCH:') != -1:
                    # print(IMU_DATA.decode('utf'))
                    THROTTLE = IMU_DATA.decode('utf').partition('PITCH:')[2]
                    THROTTLE = THROTTLE[:5]
                    STEER = IMU_DATA.decode('utf').partition('ROLL:')[2]
                    STEER = STEER[:5]
                    car_controls.throttle = float(THROTTLE)/90
                    car_controls.steering = float(STEER)/90
                    client.setCarControls(car_controls)
                
                elif IMU_DATA.decode('utf').find('START') != -1:
                    client.enableApiControl(False)
                    TERMINATOR = True
                    break
                
                else:
                    continue

    # else if 2 camera
    elif CHOICE == 2:
        if FIRST_CYCLE == True:
            print("user_camera")
            CUSER_SOCKET.sendall(USER.encode("UTF-8"))
            POV_SOCKET.sendall(POV_CAR.encode("UTF-8"))
            CAMERA_SOCKET.sendall(CAMERAMODE_ON.encode("UTF-8"))
            # client.reset()
            client.enableApiControl(True)
            FIRST_CYCLE = False
        # if(client.isApiControlEnabled()):
        responses = client.simGetImages([airsim.ImageRequest("my_eyes", airsim.ImageType.Scene, False, False)])
        response = responses[0]
        # # get numpy array
        img1d = numpy.frombuffer(response.image_data_uint8, dtype=numpy.uint8)
        # reshape array to 4 channel image array H X W X 4
        # print(response.width, response.height, sep=" ")
        img_rgb = img1d.reshape(response.height, response.width, 3)
        # original image is fliped vertically
        img_rgb = numpy.flipud(img_rgb)
        # Inference
        results = model(img_rgb)  # includes NMS
        # Results
        if len(results.xyxy[0]) == 0:
            car_controls.brake = 1
            car_controls.throttle = 0
            car_controls.steering = 0
                    
        else:
            s = float(results.xyxy[0][0][0] + ((results.xyxy[0][0][2] - results.xyxy[0][0][0])/2)) #center of found obj on x axis
            t = float(results.xyxy[0][0][1]) # top of found obj on y axis
            s = (s - (response.width / 2)) / (response.width / 2) 
            # print(t)
            t *= -1
            t = (t + (response.height / 2)) / (response.height / 2) 

            # if(t>0):
            #     car_controls.is_manual_gear = False
            #     car_controls.throttle = t

            # else:
            #     car_controls.is_manual_gear = True
            #     car_controls.manual_gear = -1
            #     car_controls.throttle = t * -1

            car_controls.throttle = t
            car_controls.steering = s
            # results.show()  # or .show()

        client.setCarControls(car_controls)

    # else if 3 algoritem
    elif CHOICE == 3:
        if FIRST_CYCLE == True:
            print("algorithm")
            # client.reset()
            client.enableApiControl(False)
            POV_SOCKET.sendall(POV_TAR.encode("UTF-8"))
            CAMERA_SOCKET.sendall(CAMERAMODE_OFF.encode("UTF-8"))
            CUSER_SOCKET.sendall(AI.encode("UTF-8"))
            AI_SOCKET.sendall(AI_MODE_ON.encode("UTF-8"))
            FIRST_CYCLE = False

        responses = client.simGetImages([airsim.ImageRequest("main_back", airsim.ImageType.Scene, False, False)])
        response = responses[0]
        # # get numpy array
        img1d = numpy.frombuffer(response.image_data_uint8, dtype=numpy.uint8)
        # reshape array to 4 channel image array H X W X 4
        # print(response.width, response.height, sep=" ")
        img_rgb = img1d.reshape(response.height, response.width, 3)
        # original image is fliped vertically
        img_rgb = numpy.flipud(img_rgb)
        # Inference
        results = model(img_rgb)  # includes NMS
        # Results
        if len(results.xyxy[0]) == 0:
            YOLO_TARGET_SOCKET.sendall(NO.encode("UTF-8"))
            AI_SOCKET.sendall(NO.encode("UTF-8"))

        else:
            YOLO_TARGET_SOCKET.sendall(YES.encode("UTF-8"))
            AI_SOCKET.sendall(YES.encode("UTF-8"))

            # results.show()  # or .show()


    # while True:
    if TERMINATOR == False: # and get0123 == False
        if connectedPort.in_waiting:
            START_QUEUE = connectedPort.readline()

            if START_QUEUE.decode('utf').find('START') != -1:
                # print(START_QUEUE.decode('utf'))
                connectedPort.write(OK.encode('utf'))
                # get0123 = True
                CHOICE+=1
                CHOICE%=4
                FIRST_CYCLE = True
        # else:
        #     break
    elif TERMINATOR == True:
        connectedPort.write(OK.encode('utf'))
        # get0123 = True
        CHOICE+=1
        CHOICE%=4
        FIRST_CYCLE = True
        TERMINATOR = False
        
        # if get0123 == True:
        #      if connectedPort.in_waiting:
        #         CHOICE_BYTES = connectedPort.readline()

        #         if CHOICE_BYTES.decode('utf').find('MODE: ') != -1:
 
        #             print(CHOICE_BYTES.decode('utf') + "<<<")
        #             CHOICE = CHOICE_BYTES.decode('utf').partition('MODE: ')[2]
        #             CHOICE = CHOICE[:1]
        #             CHOICE = int(CHOICE)
        #             # CHOICE = int.from_bytes(CHOICE_BYTES, 'big')
        #             print(CHOICE)
        #             get0123 = False
        #             FIRST_CYCLE = True
        #             break
    if keyboard.is_pressed('q'):
        break

connectedPort.close()
CUSER_SOCKET.shutdown(2)
YOLO_TARGET_SOCKET.shutdown(2)
CAMERA_SOCKET.shutdown(2)
AI_SOCKET.shutdown(2)
POV_SOCKET.shutdown(2)


YOLO_TARGET_SOCKET.close()
CUSER_SOCKET.close()
CAMERA_SOCKET.close()
AI_SOCKET.close()
POV_SOCKET.close()
