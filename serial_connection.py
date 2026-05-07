import serial
import time

arduino = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
time.sleep(2)
arduino.flush()

def read_sensors():
    if arduino.in_waiting > 0:
        line = ""

        try:
            line = arduino.readline().decode('utf-8').strip()

            if not line:
                return None
            #parse comma sep data
            data = line.split(",")

            if len(data) != 7:
                print("Bad packet length: ", line)
                return None 

            return{
                   "T": int(data[0]), #encoder + millis() = speed 
                   "FL": int(data[1]), #front left 
                   "R": int(data[2]),  #right
                   "FR": int(data[3]), #front right
                   "L": int(data[4]), #left
                   "E": int(data[5]), #encoder 
                   "H": float(data[6]) #heading 
                }
            
        except ValueError:
            print("Bad data: ", line) 
        except UnicodeDecodeError:
            print("Bad serial decode")
    return None

def close_connection():
    arduino.close()
