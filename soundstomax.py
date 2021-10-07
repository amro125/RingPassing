import pythonosc
from pythonosc import udp_client
import pickle


# This is a sample Python script.

# Press Shift+F10 to execute it or replace it with your code.
# Press Double Shift to search everywhere for classes, files, tool windows, actions, and settings.
import socket
import pickle

def print_hi(name):
    # Use a breakpoint in the code line below to debug your script.
    print(f'Hi, {name}')  # Press Ctrl+F8 to toggle the breakpoint.


# Press the green button in the gutter to run the script.
if __name__ == '__main__':
    IP = "192.168.1.73"
    PORT_TO_MAX = 7980
    message = [2, 2]

    client = udp_client.SimpleUDPClient(IP, PORT_TO_MAX)
    client.send_message('arms', message)
    print("Done!")

    # UDP_IP = "192.168.1.73"
    # UDP_PORT = 7980
    # a = str.encode("HELLO")
    # # MESSAGE = pickle.dumps(a)
    # sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    # sock.sendto(a, (UDP_IP, UDP_PORT))

# See PyCharm help at https://www.jetbrains.com/help/pycharm/
