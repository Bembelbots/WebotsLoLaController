import socket
import struct
import cv2
import numpy as np
import queue
from enum import Enum
from threading import Thread


class CamIndex(Enum):
    TOP = 0
    BOTTOM = 1



class ImageServer:
    def __init__(self, addr, img_w, img_h, camera):
        self.img_h = img_h
        self.img_w = img_w
        self.img_bytes = img_w*img_h*2 # image size in bytes
        self.camera = camera           # camera
        self.running = True

        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind(addr)
        sock.listen()
        sock.settimeout(0.01)
        sock.setsockopt(socket.SOL_TCP, socket.TCP_NODELAY, 1)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, (16+self.img_bytes)*4)
        self.sock = sock

        self.queue = queue.Queue(maxsize=3)
        self.thread = Thread(target=self.run, daemon=True)
        self.thread.start()



    def send(self, tick, image):
        self.queue.put((tick, image))



    def stop(self):
        self.running = False
        self.thread.join()



    def bgra2yuv(self, buf):
        bgra = np.frombuffer(buf, dtype=np.uint8).reshape(self.img_h, self.img_w, 4)
        bgr = cv2.cvtColor(bgra, cv2.COLOR_BGRA2BGR)
        y, u, v = cv2.split(cv2.cvtColor(bgr, cv2.COLOR_BGR2YUV))

        # Downsample u horizontally
        u = cv2.resize(u, (self.img_w//2, self.img_h), interpolation=cv2.INTER_LINEAR)

        # Downsample v horizontally
        v = cv2.resize(v, (self.img_w//2, self.img_h), interpolation=cv2.INTER_LINEAR)

        # Interleave u and v:
        uv = np.zeros_like(y)
        uv[:, 0::2] = u
        uv[:, 1::2] = v

        # Merge y and uv channels
        yuv422 = cv2.merge((y, uv))
        return yuv422



    def run(self):
        conn = None
        while self.running:
            try:
                tick, img = self.queue.get(timeout=0.1)
                self.queue.task_done()
            except queue.Empty:
                continue

            if conn:
                img = self.bgra2yuv(img)
                header = struct.pack("8sHBBHH", b'wbimage\x00', tick, self.camera.value, 2, self.img_w, self.img_h)

                try:
                    conn.send(header)
                    conn.send(img)
                except ConnectionError:
                    conn.close()
                    conn = None
            else:
                try:
                    (conn, addr) = self.sock.accept()
                except:
                    conn = None
                    continue
