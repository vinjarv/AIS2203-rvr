
'''
Sudo:   
        - Make crash control     
'''

import cv2 as cv
import socket
import time

#def sendSensordata():
    # Implementer data-overføring fra rPi her Helene
    #tmp = 0
    
#def recvCommands():
    # Implementer kommando- overføring her Helene
    #tmp = 0
    

class Videostream:
    RX_BUFFER_SIZE = 4
    TX_BUFFER_SIZE = 65000
       
    def __init__(self, host_ip, port):
        self._host_ip = host_ip
        self._port = port
        self.__sock = None
        
    def initServer(self):
        self.__sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.__sock.bind((self._host_ip, self._port))
        
    def run(self, cap):
        print("run init")
        img_size, addr = self.__recvImgSize()
        print("addr: ", addr)
        ret, img_frame = cap.read()
        print("ret: ", ret)
        img_frame_resized = cv.resize(img_frame, img_size, interpolation=cv.INTER_AREA)
        suc, img_enc = cv.imencode(".jpg", img_frame_resized)
        if suc:
            img_bytes = img_enc.tobytes()
        img_packets, n_Packets = self.__splitInPackets(img_bytes)
        self.__sendHeader(n_Packets, img_bytes, addr)
        self.__sendPackets(img_packets, addr)
            
    def __recvImgSize(self):
        rx_buffer, addr = self.__sock.recvfrom(self.RX_BUFFER_SIZE)
        img_height = (rx_buffer[0]<<8)+(rx_buffer[1]&0xff)
        img_width = (rx_buffer[2]<<8)+(rx_buffer[3]&0xff)
        img_dim = (img_width, img_height)
        return img_dim, addr
        
    def __splitInPackets(self, byteString):
        num_packets = len(byteString)//self.TX_BUFFER_SIZE
        if(len(byteString)%self.TX_BUFFER_SIZE):
            num_packets += 1
        img_byte_array = [byteString[i:i+self.TX_BUFFER_SIZE] for i in range(0, len(byteString), self.TX_BUFFER_SIZE)]
        #Numerate:
        for i in range(len(img_byte_array)):
            img_byte_array[i] += (i+1).to_bytes(2, 'big')
        
        return img_byte_array, num_packets
    
    #Num_packets, Packet_size, Num_bytes, Num_bytes in last
    def __sendHeader(self, nPackets, byteString, addr):
        tx_header_buffer = nPackets.to_bytes(2, 'big') + \
            (self.TX_BUFFER_SIZE+2).to_bytes(2, 'big') + \
                (len(byteString)+2*nPackets).to_bytes(4, 'big') + \
                    (len(byteString)%self.TX_BUFFER_SIZE+2).to_bytes(2, 'big')
        self.__sock.sendto(tx_header_buffer, addr)
        
    def __sendPackets(self, byte_array, addr):
        t0 = time.time()
        for i in range(len(byte_array)):
            self.__sock.sendto(byte_array[i], addr)
            time.sleep(0.001)
        print("PFS: \t", 1/(time.time()-t0))

    
                          
if __name__ == '__main__':  
    vidStream = Videostream("0.0.0.0", 8888)
    vidStream.initServer()
    # Implement switch case or something so vidcap only starts when rx_buf != 0 (timeout maybe 5sek)
    cap = cv.VideoCapture(0, cv.CAP_ANY)
    if cap:
        print("cap true")
    while True:
        vidStream.run(cap)
        
    cap.release()
        
    

    
    

    













