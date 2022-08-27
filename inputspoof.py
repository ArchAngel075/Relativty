#https://www.aliexpress.com/item/32977929644.html
import socket
import numpy as np
from pynput.keyboard import Key, KeyCode, Listener

HOST = "127.0.0.1"  # The server's hostname or IP address
PORT = 50000  # The port used by the server

def on_press(key):
    print('{0} pressed'.format(
        key))
    if key == KeyCode.from_char('z'):
        print("send input packet", True);
        s.sendall(bytearray("{I010010011}".encode()))
    

def on_release(key):
    print('{0} release'.format(
        key))
    if(key == KeyCode.from_char('z')):
        s.sendall(bytearray("{I000000000}".encode()))
    if key == Key.esc:
        # Stop listener
        return False

print("char char {0}".format(KeyCode.from_char('z')))
with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    print("attempt connection")
    s.settimeout(None)
    while True:
        try:
            s.connect((HOST, PORT))
            break;
        except ConnectionRefusedError as e:
            print("refused. Try again");
    print("connected, listen for keys")
    with Listener(
            on_press=on_press,
            on_release=on_release) as listener:
        listener.join()