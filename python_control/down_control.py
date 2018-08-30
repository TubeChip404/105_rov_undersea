from socket import *
import time
import serial
from threading import Thread
import os

#Configure Here

host_port=6666
host_addr=''
host_max_listen=5

serial_number='/dev/ttyUSB0'
serial_bandrate=115200
serial_timeout=5

network_delay=0.01

#End
allow_reuse_address = 1

host_sk= socket(AF_INET,SOCK_STREAM)
host_sk.setsockopt(SOL_SOCKET,SO_REUSEADDR,1)
host_sk.bind((host_addr,host_port))

client_sk= socket(AF_INET,SOCK_STREAM)
host_sk.listen(host_max_listen)

serial_send_data=''
main_temp=0
def recv_data_from_client():
    global serial_send_data,host_sk,network_delay,client_sk
    while 1 :
        print 'waiting for connection'
        client_sk, client_addr = host_sk.accept()
        print 'connected from :',client_addr
        while 1 :
            try:
                recv_data=client_sk.recv(1024)
                if len(recv_data)<48 :
                    continue
                for pos in range(0,len(recv_data)-48):
                    if ord(recv_data[pos])==0xaa :
                        chksum=0;
                        for offset in range(0,47):
                            chksum+=ord(recv_data[pos+offset])
                        if chksum%256==ord(recv_data[pos+47]):
                            serial_send_data=''
                            for i in range(0,48):
                                #print ord(recv_data[pos+i]),
                                serial_send_data+=recv_data[pos+i]
                        else:
                            break
                client_sk.send(chr(main_temp%256))
                time.sleep(network_delay)
            except:
                client_sk.close()
                break


def serial_send_data_to_board():
    global serial_send_data,serial_bandrate,serial_number,serial_timeout,main_temp
    dev=serial.Serial(serial_number,serial_bandrate,timeout=serial_timeout)
    while 1 :
        dev.write(serial_send_data)
        main_temp=ord(dev.read(1))

def switch_camera_record():
    global serial_send_data
    last_state=0
    os.system("killall nc")
    os.system("killall raspivid")
    os.system("killall nc")
    os.system("killall raspivid")
    os.system("/opt/vc/bin/raspivid -t 0 -w 1280 -h 720 -ih -fps 30  -mm matrix -vs -awb incandescent -o - | nc -k -l 8090 &")
    while 1 :
        if len(serial_send_data)<40 :
            continue
        if ord(serial_send_data[34])!=last_state :
            last_state=ord(serial_send_data[34])
            if ord(serial_send_data[34])==1 :
                os.system("killall nc")
                os.system("killall raspivid")
                os.system("killall nc")
                os.system("killall raspivid")
                filename=time.strftime("%Y-%m-%d %H-%M-%S",time.localtime())
                cmd="/opt/vc/bin/raspivid -t 0 -w 1280 -h 720 -ih -fps 30  -mm matrix -vs -awb incandescent -o - |tee \"/home/pi/"+filename+".mp4\" | nc -k -l 8090 &"
                print cmd
                os.system(cmd)
            else :
                os.system("killall nc")
                os.system("killall raspivid")
                os.system("killall nc")
                os.system("killall raspivid")
                os.system("/opt/vc/bin/raspivid -t 0 -w 1280 -h 720 -ih -fps 30  -mm matrix -vs -awb incandescent -o - | nc -k -l 8090 &")

    
t_serial=Thread(target=serial_send_data_to_board)
t_network=Thread(target=recv_data_from_client)
t_camera=Thread(target=switch_camera_record)

t_serial.setDaemon(True)
t_network.setDaemon(True)
t_camera.setDaemon(True)

t_serial.start()
t_network.start()
t_camera.start()

while 1 :
    try:
        print ('main temp=%d'%(main_temp))
        time.sleep(1)
    except:
        client_sk.close()
        host_sk.close()
        exit()