import socket             

hostname = socket.gethostname()
local_ip = socket.gethostbyname(hostname)
print(hostname)
print(local_ip)

# next create a socket object  
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
print ("Socket successfully created") 
  
# set host ip and port number
host_ip = '192.168.0.115'#82.45.2.139'
port = 4000                
  
# bind socket to ip-port & listen to requests  
s.bind((host_ip, port))    
print ("socket bound to %s" %(port))  
  
# put the socket into listening mode, max queue of 5
s.listen(5)   
print ("socket is listening")            
  
# a forever loop until we interrupt it or an error occurs  
while True:  
    c, addr = s.accept()      # Establish connection with client.  
    print ('Got connection from', addr) 
    
    print(c.recv(1024))
    
    c.send('Thank you for connecting')  # send a thank you message to the client. 
    
    
    # Close the connection with the client  
    c.close()  
