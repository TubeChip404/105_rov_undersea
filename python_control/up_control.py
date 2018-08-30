from socket import *
import time
import serial
from threading import Thread

#Configure
raspi_addr='192.168.0.112'
raspi_port=6666

power_addr='192.168.0.7'
power_port=20108

joypad_com='COM6'
joypad_band=115200

network_timeout=1
serial_timeout=1

joypad_data_frame_length=24
network_send_delay=0.05

thruster_horiz_left_front_max_min_joypad_max_min=[368,184,0,1024]
thruster_horiz_right_front_max_min_joypad_max_min=[368,184,0,1024]
thruster_horiz_left_behind_max_min_joypad_max_min=[368,184,0,1024]
thruster_horiz_right_behind_max_min_joypad_max_min=[368,184,0,1024]
thruster_verti_left_middle_max_min_joypad_max_min=[368,184,0,1024]
thruster_verti_right_middle_max_min_joypad_max_min=[368,184,0,1024]

arm_front_servo_max_min_joypad_max_min=[368,184,0,1024]
arm_behind_servo_max_min_joypad_max_min=[368,184,0,1024]
camera_servo_max_min_joypad_max_min=[368,184,0,1024]
#End

raspi_sk=socket(AF_INET,SOCK_STREAM)
raspi_sk.connect((raspi_addr,raspi_port))

power_sk=socket(AF_INET,SOCK_STREAM)
power_sk.connect((power_addr,power_port))

joypad_dev=serial.Serial(joypad_com,joypad_band,timeout=serial_timeout)

#global values

joypad_analog_value=[512,512,512,512,512,512,512,512]
joypad_digital_value=[0,0,0]

main_temp=0
power_temp=0

power_data_frame=''
#end
def read_joypad_data_to_global_value():
    global joypad_dev,joypad_analog_value,joypad_digital_value,joypad_data_frame_length
    while 1:
        try:
            joypad_data=joypad_dev.read(joypad_data_frame_length*2)
            for pos in range(0,joypad_data_frame_length):
                if ord(joypad_data[pos])==0xaa:
                    chksum=0
                    for offset in range(0,joypad_data_frame_length-1):
                        chksum+=ord(joypad_data[pos+offset])
                    if chksum%256==ord(joypad_data[pos+23]):
                        for i in range(0,8):
                            joypad_analog_value[i]=ord(joypad_data[pos+i*2+1])*256+ord(joypad_data[pos+i*2+2])
                        joypad_digital_value[0]=ord(joypad_data[pos+17])
                        joypad_digital_value[1]=ord(joypad_data[pos+18])
                        joypad_digital_value[2]=ord(joypad_data[pos+19])
        except:
            print 'joypad read error occur.'
            joypad_dev.close()
            try:
                joypad_dev=serial.Serial(joypad_com,joypad_band,timeout=serial_timeout)
            except:
                time.sleep(0.5)
                continue
            continue
def map_value(module,invalue):
    invalue=float(invalue)
    source_differ=invalue-module[2]
    source_scale=source_differ/(module[3]-module[2])
    if source_scale<0 :
        source_scale=0
    if source_scale>1 :
        source_scale=1
    target_value=module[0]+(module[1]-module[0])*source_scale
    return round(target_value)

def send_data_to_power_controller():
    global power_sk,joypad_analog_value,power_temp,power_data_frame
    while 1 :
        try:
            power_data_frame='\xaa'
            rl_differ=abs(512-joypad_analog_value[2])
            real_x1=(joypad_analog_value[0]-rl_differ)*0.5+512
            real_x2=(joypad_analog_value[0]+rl_differ)*0.5
            real_y1=(joypad_analog_value[1]-rl_differ)*0.5+512
            real_y2=(joypad_analog_value[1]+rl_differ)*0.5
            thruster_horiz_left_front=map_value(thruster_horiz_left_front_max_min_joypad_max_min,real_y1)
            thruster_horiz_right_front=map_value(thruster_horiz_right_front_max_min_joypad_max_min,real_x1)
            thruster_horiz_left_behind=map_value(thruster_horiz_left_behind_max_min_joypad_max_min,real_x2)
            thruster_horiz_right_behind=map_value(thruster_horiz_right_behind_max_min_joypad_max_min,real_y2)
            thruster_verti_left_middle=map_value(thruster_verti_left_middle_max_min_joypad_max_min,joypad_analog_value[3])
            thruster_verti_right_middle=map_value(thruster_verti_right_middle_max_min_joypad_max_min,joypad_analog_value[4])
            arm_front_servo=map_value(arm_front_servo_max_min_joypad_max_min,joypad_analog_value[4])
            arm_behind_servo=map_value(arm_behind_servo_max_min_joypad_max_min,joypad_analog_value[5])
            camera_servo=map_value(camera_servo_max_min_joypad_max_min,joypad_analog_value[6])
            power_data_frame+=chr(int(thruster_horiz_left_front/256))#make package started
            power_data_frame+=chr(int(thruster_horiz_left_front%256))
            power_data_frame+=chr(int(thruster_horiz_right_front/256))
            power_data_frame+=chr(int(thruster_horiz_right_front%256))
            power_data_frame+=chr(int(thruster_horiz_left_behind/256))
            power_data_frame+=chr(int(thruster_horiz_left_behind%256))
            power_data_frame+=chr(int(thruster_horiz_right_behind/256))
            power_data_frame+=chr(int(thruster_horiz_right_behind%256))
            
            power_data_frame+=chr(int(thruster_verti_left_middle/256))
            power_data_frame+=chr(int(thruster_verti_left_middle%256))
            power_data_frame+=chr(int(thruster_verti_right_middle/256))
            power_data_frame+=chr(int(thruster_verti_right_middle%256))
            
            power_data_frame+=chr(int(arm_front_servo/256))
            power_data_frame+=chr(int(arm_front_servo%256))
            power_data_frame+=chr(int(arm_behind_servo/256))
            power_data_frame+=chr(int(arm_behind_servo%256))
            power_data_frame+=chr(int(camera_servo/256))
            power_data_frame+=chr(int(camera_servo%256))
            
            power_data_frame+='\x00'*(32-len(power_data_frame))+chr(joypad_digital_value[0])+chr(joypad_digital_value[1])
            power_data_frame+='\x00'*(47-len(power_data_frame)) 
            chksum=0
            for i in range(0,len(power_data_frame)):
                chksum+=ord(power_data_frame[i])
            power_data_frame+=chr(chksum%256)                   #make package finished
            
            power_sk.send(power_data_frame*5)
            time.sleep(network_send_delay)
            power_temp=ord(power_sk.recv(1))
        except:
            power_sk.close()
            print 'network error occur.'
            try:
                power_sk.connect((power_addr,power_port))
            except:
                power_sk.close()
                time.sleep(0.5)
                continue
            continue

def send_data_to_raspi():
    global power_data_frame,raspi_sk,main_temp
    while 1 :
        try :
            raspi_sk.send(power_data_frame*30)
            main_temp=ord(raspi_sk.recv(1))
        except :
            continue
    
t_read_joypad=Thread(target=read_joypad_data_to_global_value)
t_send_data_to_power=Thread(target=send_data_to_power_controller)
t_send_data_to_raspi=Thread(target=send_data_to_raspi)

t_read_joypad.setDaemon(True)
t_send_data_to_power.setDaemon(True)
t_send_data_to_raspi.setDaemon(True)

t_read_joypad.start()
t_send_data_to_power.start()
t_send_data_to_raspi.start()

while 1 :
    print ('power temp=%d C \t main temp=%d C'%(power_temp,main_temp))
    time.sleep(1)
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
