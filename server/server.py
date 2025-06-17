import socket
import cv2
import numpy as np
import struct
import threading
from ball_processing import BallProcessing


class ClientHandler:
    def __init__(self, conn, addr):
        self.conn = conn
        self.addr = addr
        self.bp = BallProcessing()
        self.running = True
        self.thread = threading.Thread(target=self.run)
        self.thread.daemon = True
        self.thread.start()

    def process_frame(self, frame):
        mask, processed_frame = self.bp.process(frame)
        params = self.bp.get_current_data()

        command = 'STOP'
        if params['obj_x'] > 0 and params['obj_y'] > 0 and params['obj_r'] > 5:
            if params['obj_x'] > 400:
                command = 'RIGHT'
            elif params['obj_x'] < 240:
                command = 'LEFT'

        # Опционально: показывать только для одного клиента или выключить
        # window_name = f"Ball Tracking {self.addr}"
        # cv2.imshow(window_name, processed_frame)
        # cv2.waitKey(1)

        return command

    def run(self):
        print(f"[NEW CONNECTION] {self.addr} connected.")
        try:
            while self.running:
                size_data = self.conn.recv(4)
                if len(size_data) != 4:
                    break

                frame_size = struct.unpack('>L', size_data)[0]

                frame_data = b''
                while len(frame_data) < frame_size:
                    chunk = self.conn.recv(min(4096, frame_size - len(frame_data)))
                    if not chunk:
                        break
                    frame_data += chunk

                if len(frame_data) != frame_size:
                    break

                frame = cv2.imdecode(np.frombuffer(frame_data, dtype=np.uint8), cv2.IMREAD_COLOR)
                if frame is None:
                    continue

                command = self.process_frame(frame)
                self.conn.sendall(command.encode())

        except Exception as e:
            print(f"[ERROR] Connection with {self.addr} closed: {e}")
        finally:
            self.conn.close()
            print(f"[CONNECTION CLOSED] {self.addr}")
            self.running = False


class BallTrackingServer:
    def __init__(self):
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server_socket.bind(('192.168.50.211', 1489))
        self.server_socket.listen(5)  # Максимум 5 ожидающих подключений
        print("[SERVER STARTED] Waiting for clients...")

    def accept_connections(self):
        while True:
            conn, addr = self.server_socket.accept()
            print(f"[ACCEPTED] Connection from {addr}")
            ClientHandler(conn, addr)

    def run(self):
        try:
            self.accept_connections()
        except KeyboardInterrupt:
            print("Shutting down server...")
        finally:
            self.server_socket.close()
            cv2.destroyAllWindows()


if __name__ == '__main__':
    server = BallTrackingServer()
    server.run()
