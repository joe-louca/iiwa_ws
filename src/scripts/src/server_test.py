import socket

HOST = '192.168.0.115'                  
PORT = 50017       # port

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.bind((HOST, PORT))
s.listen(1)
conn, addr = s.accept()
print ('Connected by', addr)
conn.send("Hello Client".encode())
conn.send("How are you".encode())
conn.close()
print("Connection", addr, "Closed")
