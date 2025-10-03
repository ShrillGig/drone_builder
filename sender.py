import serial, time


symbols = [('1', '1'), ('2', '2'),  ('3', '3')]

port = "COM3"
uart = serial.Serial(port, 9600, timeout=2)
time.sleep(2)
uart.reset_input_buffer()

size = len(symbols)
uart.write(f"{size}".encode())
resp = uart.readline().decode().strip()
print(f"Accepted list size: {resp}")

for symbol in symbols:

    uart.write(f"#{symbol[0]};".encode())
    uart.write(f"${symbol[1]};".encode())

for symbol in symbols:
    resp = uart.readline().decode().strip()
    print(f"Accepted: {resp}")
    resp = uart.readline().decode().strip()
    print(f"Accepted: {resp}")

print("Please enter p to run a program")
if input() == 'p':
    uart.write('p'.encode())
    resp = uart.readline().decode().strip()
    print(f"Accepted to run code: {resp}")

uart.close()
