from socket import *


raspi_sk=socket(AF_INET,SOCK_STREAM)
raspi_sk.connect(('192.168.0.112',6666))
while 1 :
    cmd_str=raw_input(">")
    data='\xaa'+'\x00'*33
    if cmd_str=='1':
        data+=chr(1)
    else :
        data+=chr(0)
    data+='\x00'*12
    chksum=0
    for i in range(0,47):
        chksum+=ord(data[i])
    data+=chr(chksum%256)
    raspi_sk.send(data*30)