from socket import socket, AF_INET, SOCK_DGRAM
import time
import json

def udp_server(net_info):
    sock_server = socket(AF_INET, SOCK_DGRAM)
    sock_server.bind(net_info)
    buf_size = 1024
    
    while True:
        msg, addr = sock_server.recvfrom(buf_size)
        print(type(msg))
        print(msg)
        msg_json = msg.decode('utf-8')
        print(type(msg_json))
        print(msg_json)
        msg_dict = json.loads(msg_json)
        print(type(msg_dict))
        print(msg_dict)
        print('Got message({}) from'.format(msg_dict), addr)
        # time.sleep(0.5)
        
        response = time.ctime()
        # print(type(response))
        # print(response)
        response_dict = {'response': response}
        # print(type(response_dict))
        # print(response_dict)
        response_json = json.dumps(response_dict)
        # print(type(response_json))
        # print(response_json)
        sock_server.sendto(response_json.encode('utf-8'), addr)
        print("Send response({}) to".format(response_dict), addr)

if __name__ == '__main__':
    net_addr = "127.0.0.1"
    net_port = 8080
    udp_server((net_addr, net_port))
    
    print("Udp server test done!")
