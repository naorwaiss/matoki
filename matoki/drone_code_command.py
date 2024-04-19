import socket

def main():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.sendto(''.encode(), ("127.0.0.1", 5005))
    while True:
        length = sock.recv(3).decode()
        message = sock.recv(3+int(length)).decode()
        print(message)



if __name__ == "__main__":
    main()