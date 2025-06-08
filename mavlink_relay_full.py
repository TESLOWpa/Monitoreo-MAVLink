import socket
import select
import threading
import serial
from pymavlink import mavutil

def read_mavlink_and_broadcast(master, clients_tcp, lock_tcp, udp_out):
    while True:
        try:
            msg = master.recv_msg()
            if msg:
                data = msg.get_msgbuf()

                # TCP
                with lock_tcp:
                    for client in clients_tcp.copy():
                        try:
                            client.sendall(data)
                        except Exception as e:
                            print(f"[TCP] Error enviando a cliente: {e}")
                            clients_tcp.remove(client)

                # UDP
                if udp_out:
                    try:
                        udp_out.sendto(data, udp_out.dest)
                    except Exception as e:
                        print(f"[UDP] Error enviando: {e}")

        except Exception as e:
            print(f"[MAVLINK] Error leyendo: {e}")

def start_tcp_server(host, port, clients, lock):
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.bind((host, port))
    server.listen(5)
    server.setblocking(False)
    print(f"[TCP] Escuchando en {host}:{port}")

    inputs = [server]
    while True:
        readable, _, _ = select.select(inputs, [], [], 1.0)
        for s in readable:
            if s is server:
                client_socket, addr = server.accept()
                print(f"[TCP] Cliente conectado: {addr}")
                client_socket.setblocking(False)
                with lock:
                    clients.append(client_socket)

class UDPSender:
    def __init__(self, host, port):
        self.dest = (host, port)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def sendto(self, data, dest):
        self.sock.sendto(data, dest)

def main():
    # Parámetros fijos
    mavport = "/dev/ttyACM0"
    mavbaud = 115200  # Baudrate para MAVLink

    tcpaddr = "0.0.0.0"  # Escucha en todas las interfaces
    tcpport = 2001

    udpaddr = "10.207.0.112"
    udpport = 5760

    print("[INFO] Conectando a MAVLink en /dev/ttyACM0 @115200...")
    master = mavutil.mavlink_connection(mavport, baud=mavbaud)
    master.wait_heartbeat()
    print("[INFO] HEARTBEAT recibido")

    clients_tcp = []
    lock_tcp = threading.Lock()

    # Inicia servidor TCP
    threading.Thread(target=start_tcp_server, args=(tcpaddr, tcpport, clients_tcp, lock_tcp), daemon=True).start()

    # Inicializa salida UDP
    udp_out = UDPSender(udpaddr, udpport)
    print(f"[UDP] Enviando datos a {udpaddr}:{udpport}")

    # Bucle principal
    read_mavlink_and_broadcast(master, clients_tcp, lock_tcp, udp_out)

if __name__ == "__main__":
    main()

