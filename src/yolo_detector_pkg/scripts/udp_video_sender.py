#!/usr/bin/env python3
"""飞车侧视频发送端 —— 把一份 JPEG 帧分块用裸 UDP 发给车。

跨机链路一律裸 UDP、不碰 DDS(两机同名话题不做区分,合并 DDS 域会串台,理由同
activity_control_pkg/src/xmachine_bridge.cpp)。车侧收端见 car 侧
follower_pkg/src/flycar_video_bridge.cpp,收齐一帧后 publish 成
/flycar/camera/image/compressed,平板从车 localhost 走 FastDDS 订阅显示。

一张 JPEG 几十 KB,远超 UDP 单包安全大小,所以按 ~1400B 分块,每块带
帧id+块序号+总块数,车侧按帧 id 重组;丢块就丢这一帧(视频丢帧无所谓,不重传)。

包格式(小端,两机都是 ARM 小端;改一处必须车侧同步改):
  magic:u16 = 0xFC08
  frame_id:u16      整帧序号,重组用,回绕无妨
  chunk_idx:u16     本块序号 0..total-1
  chunk_total:u16   本帧总块数
  之后是 payload: JPEG 的第 chunk_idx 段
头共 8 字节。
"""

import socket
import struct

MAGIC_VIDEO = 0xFC08
HEADER_FMT = "<HHHH"  # magic, frame_id, chunk_idx, chunk_total (小端)
HEADER_SIZE = struct.calcsize(HEADER_FMT)  # 8


class UdpVideoSender:
    """把 JPEG 帧分块 UDP 发到 (car_ip, car_port)。线程无关,直接在推理回调里调 send()。"""

    def __init__(self, car_ip, car_port, chunk_payload=1400, logger=None):
        self.addr = (car_ip, int(car_port))
        self.chunk_payload = int(chunk_payload)
        self._log = logger or (lambda _m: None)
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        # 发送缓冲大到能一次塞完一帧的所有块,避免突发丢
        try:
            self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 1 << 20)
        except OSError:
            pass
        self._frame_id = 0
        self._log(f"UdpVideoSender -> {self.addr}, chunk={self.chunk_payload}B")

    def send(self, jpeg_bytes):
        """发一帧 JPEG。UTF 无关,jpeg_bytes 是 bytes。发送失败静默丢(下一帧继续)。"""
        if not jpeg_bytes:
            return
        n = len(jpeg_bytes)
        total = (n + self.chunk_payload - 1) // self.chunk_payload
        if total > 0xFFFF:
            # 帧大到 6.5 万块几乎不可能,真遇到就丢弃避免 chunk_total 溢出
            self._log(f"帧过大丢弃: {n} 字节需要 {total} 块")
            return
        fid = self._frame_id & 0xFFFF
        self._frame_id += 1
        for idx in range(total):
            start = idx * self.chunk_payload
            payload = jpeg_bytes[start:start + self.chunk_payload]
            header = struct.pack(HEADER_FMT, MAGIC_VIDEO, fid, idx, total)
            try:
                self._sock.sendto(header + payload, self.addr)
            except OSError:
                # 缓冲满/瞬时不可达: 丢这块,整帧作废,不阻塞推理
                break

    def close(self):
        try:
            self._sock.close()
        except OSError:
            pass
