import socket 

# Create a socket object
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
print('socket successfully created')

# Define port on which to connect
ip = '192.168.0.16' # surface '192.168.0.115' # dell-ubu
port = 4000

# Connect to the server on dell-ubu
s.connect((ip, port))
print('socket connected to %s' %(ip))

# Receive data from the server
print(s.recv(1024))

# Close the connection
s.close
