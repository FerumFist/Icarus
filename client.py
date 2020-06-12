import socket
import sys
import time

def main():
    soc = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    host = "127.0.0.1"
    port = 8888

    try:
        soc.connect((host, port))
    except:
        print("Connection error")
        sys.exit()

    print("Enter 'quit' to exit")
    message = input(" -> ")

    while message != 'quit':
        print(time.time())
        soc.sendall(message.encode("utf8"))
        if soc.recv(5120).decode("utf8") == "-":
            recv = soc.recv(4096)
            print(recv.decode("utf-8"))
            print(time.time())


        message = input(" -> ")

    soc.send(b'--quit--')

if __name__ == "__main__":
    main()