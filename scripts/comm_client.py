from socket import socket, AF_INET, SOCK_DGRAM
import time
import json

def udp_client(net_info):
    sock_client = socket(AF_INET, SOCK_DGRAM)
    buf_size = 1024
    
    while True:
        msg = input('Please input the message >>:').strip()
        if not msg:
            continue
        print(type(msg))
        print(msg)
        msg_dict = {'msg': msg}
        print(type(msg_dict))
        print(msg_dict)
        msg_json = json.dumps(msg_dict)
        print(type(msg_json))
        print(msg_json)
        sock_client.sendto(msg_json.encode('utf-8'), net_info)
        print('Send message({}) to'.format(msg_dict), net_info)
        # time.sleep(0.5)
        
        response, addr = sock_client.recvfrom(buf_size)
        print(type(response))
        print(response)
        response_json = response.decode('utf-8')
        print(type(response_json))
        print(response_json)
        response_dict = json.loads(response_json)
        print(type(response_dict))
        print(response_dict)
        print('Got response({}) from'.format(response_dict), addr)

if __name__ == '__main__':
    net_addr = "127.0.0.1"
    net_port = 8080
    udp_client((net_addr, net_port))
    
    print("Udp client test done!")
