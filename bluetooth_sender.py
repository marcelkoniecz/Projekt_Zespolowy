import bluetooth

bd_addr = "00:21:07:34:E0:63"
port = 1
sock=bluetooth.BluetoothSocket(bluetooth.RFCOMM)
sock.connect((bd_addr, port))

print("Write input")

while True:
    data = input()
    if data == "x":
        break
    sock.send(data.strip())

sock.close()
