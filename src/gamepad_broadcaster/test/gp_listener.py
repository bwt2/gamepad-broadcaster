import socket

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(("127.0.0.1", 5005))

print("Listening on UDP 5005")
while True:
    data, addr = sock.recvfrom(1024)
    print(f"Received: {data.decode()}")