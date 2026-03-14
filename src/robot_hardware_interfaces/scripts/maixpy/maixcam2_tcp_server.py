#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import configparser
import os
import socket
from typing import Any, Dict, List, Optional, Tuple

from maix import app, camera, time
from maix.err import Err # type: ignore


CMD_SET_REPORT = 0xF8
CMD_APP_LIST = 0xF9
CMD_START_APP = 0xFA
CMD_EXIT_APP = 0xFB
CMD_CUR_APP_INFO = 0xFC
CMD_APP_INFO = 0xFD

APP_CMD_SNAP = 0x01

HEADER = b"\xAA\xCA\xAC\xBB"
VERSION = 0x01
FLAG_IS_RESP = 0x80
FLAG_RESP_OK = 0x40
FLAG_IS_REPORT = 0x20


class MaixCam2ProtocolServer:
    def __init__(self, host: str = "0.0.0.0", port: int = 5555, buff_size: int = 4096):
        self.host = host
        self.port = port
        self.buff_size = buff_size
        self.server: Optional[socket.socket] = None
        self.conn: Optional[socket.socket] = None
        self.addr: Optional[Tuple[str, int]] = None
        self.rx_buffer = bytearray()

        self.should_exit = False
        self.pending_switch_app: Optional[Tuple[str, str]] = None
        self.last_launched_app_id: Optional[str] = None
        self.last_launched_exec_name: Optional[str] = None

        self.cam: Any = None

    def hex_bytes(self, data: bytes) -> str:
        return " ".join(f"{b:02X}" for b in data)

    def crc16_ibm(self, data: bytes) -> int:
        crc = 0x0000
        for b in data:
            crc ^= b
            for _ in range(8):
                if crc & 0x01:
                    crc = (crc >> 1) ^ 0xA001
                else:
                    crc >>= 1
        return crc & 0xFFFF

    def encode_frame(self, flags: int, cmd: int, body: bytes) -> bytes:
        data_len = 1 + 1 + len(body) + 2
        frame_wo_crc = bytearray()
        frame_wo_crc += HEADER
        frame_wo_crc += int(data_len).to_bytes(4, "little")
        frame_wo_crc += bytes([flags & 0xFF, cmd & 0xFF])
        frame_wo_crc += body

        crc = self.crc16_ibm(bytes(frame_wo_crc))
        frame_wo_crc += int(crc).to_bytes(2, "little")
        return bytes(frame_wo_crc)

    def encode_resp_ok(self, cmd: int, body: bytes = b"") -> bytes:
        flags = FLAG_IS_RESP | FLAG_RESP_OK | VERSION
        return self.encode_frame(flags, cmd, body)

    def encode_resp_err(self, cmd: int, err_code: int, err_text: bytes) -> bytes:
        flags = FLAG_IS_RESP | VERSION
        body = bytes([err_code & 0xFF]) + err_text
        return self.encode_frame(flags, cmd, body)

    def decode_one_frame(self) -> Optional[Tuple[int, int, bytes]]:
        if len(self.rx_buffer) < 8:
            return None

        if self.rx_buffer[0:4] != HEADER:
            idx = self.rx_buffer.find(HEADER)
            if idx < 0:
                self.rx_buffer.clear()
                return None
            del self.rx_buffer[:idx]
            if len(self.rx_buffer) < 8:
                return None

        data_len = int.from_bytes(self.rx_buffer[4:8], "little")
        full_len = 8 + data_len
        if data_len < 4:
            del self.rx_buffer[:4]
            return None
        if len(self.rx_buffer) < full_len:
            return None

        frame = bytes(self.rx_buffer[:full_len])
        del self.rx_buffer[:full_len]

        recv_crc = int.from_bytes(frame[-2:], "little")
        calc_crc = self.crc16_ibm(frame[:-2])
        if recv_crc != calc_crc:
            print(f"crc mismatch recv=0x{recv_crc:04X} calc=0x{calc_crc:04X}")
            return None

        flags = frame[8]
        cmd = frame[9]
        body = frame[10:-2]
        return flags, cmd, body

    def load_apps(self) -> Dict[str, Dict[str, str]]:
        info_path = "/maixapp/apps/app.info"
        conf = configparser.ConfigParser()
        conf.read(info_path)

        apps: Dict[str, Dict[str, str]] = {}
        for section in list(conf.keys()):
            if section in ["basic", "DEFAULT"]:
                continue
            apps[section] = dict(conf[section])
        return apps

    def split_cstrs(self, data: bytes) -> List[str]:
        if not data:
            return []
        parts = data.split(b"\x00")
        out: List[str] = []
        for p in parts:
            if len(p) > 0:
                out.append(p.decode("utf-8", errors="ignore"))
        return out

    def encode_app_list_body(self, apps: Dict[str, Dict[str, str]]) -> bytes:
        keys = list(apps.keys())
        body = bytes([len(keys) & 0xFF])
        for app_id in keys:
            body += app_id.encode() + b"\x00"
        return body

    def encode_app_info_body(self, app_idx: int, app_id: str, apps: Dict[str, Dict[str, str]]) -> bytes:
        info = apps.get(app_id, {})
        name = info.get("name", app_id)
        desc = info.get("desc", "")
        body = bytes([app_idx & 0xFF])
        body += app_id.encode() + b"\x00"
        body += name.encode() + b"\x00"
        body += desc.encode() + b"\x00"
        return body

    def parse_start_app_body(self, body: bytes) -> Tuple[Optional[str], str, str]:
        if len(body) < 1:
            return None, "start_app body empty", ""

        apps = self.load_apps()
        app_ids = list(apps.keys())

        idx = body[0]
        params = self.split_cstrs(body[1:])

        target_app_id: Optional[str] = None
        app_func = ""

        if idx != 0xFF:
            if idx >= len(app_ids):
                return None, "invalid app idx", ""
            target_app_id = app_ids[idx]
            if len(params) == 0:
                app_func = ""
            elif len(params) == 1:
                app_func = params[0]
            else:
                return None, "too many params for idx mode", ""
        else:
            if len(params) == 0:
                return None, "app_id required when idx=0xFF", ""
            if len(params) > 2:
                return None, "too many params for app_id mode", ""
            target_app_id = params[0]
            if target_app_id not in apps:
                return None, "app_id not found", ""
            if len(params) == 2:
                app_func = params[1]

        return target_app_id, "", app_func

    def init_camera(self):
        if self.cam is None:
            self.cam = camera.Camera(640, 480)

    def handle_snap(self) -> bytes:
        self.init_camera()
        img = self.cam.read()
        save_dir = "/root/photos"
        if not os.path.exists(save_dir):
            os.makedirs(save_dir)
        ts = time.time_ms()
        save_path = f"{save_dir}/snap_{ts}.jpg"
        img.save(save_path)
        return save_path.encode("utf-8")

    def send(self, data: bytes):
        if self.conn is None:
            return
        print("[tx]", self.hex_bytes(data))
        self.conn.sendall(data)

    def close_client(self):
        if self.conn is not None:
            try:
                self.conn.close()
            except Exception:
                pass
            self.conn = None
            self.addr = None

    def close(self):
        self.close_client()
        if self.server is not None:
            try:
                self.server.close()
            except Exception:
                pass
            self.server = None

    def pid_file_path(self, app_id: str) -> str:
        return f"/tmp/{app_id}.pid"

    def try_stop_by_pid_file(self, app_id: str) -> bool:
        pid_file = self.pid_file_path(app_id)
        if not os.path.exists(pid_file):
            return False
        try:
            with open(pid_file, "r") as f:
                pid_str = f.read().strip()
            if not pid_str:
                return False
            pid = int(pid_str)
            ret = os.system(f"kill -TERM {pid}")
            print(f"exit_app stop by pid ret={ret}, pid={pid}")
            if ret == 0:
                return True
        except Exception as e:
            print(f"exit_app stop by pid failed: {e}")
        return False

    def stop_app_by_id(self, app_id: str) -> bool:
        apps = self.load_apps()
        if app_id not in apps:
            print(f"exit_app: app_id not found: {app_id}")
            return False

        exec_name = apps[app_id].get("exec", "")
        if not exec_name:
            print(f"exit_app: app_id {app_id} has empty exec")
            return False

        if self.try_stop_by_pid_file(app_id):
            if self.last_launched_app_id == app_id:
                self.last_launched_app_id = None
                self.last_launched_exec_name = None
            return True

        full_path = f"/maixapp/apps/{app_id}/{exec_name}"
        cmds = [
            f"pkill -f '{full_path}'",
            f"pkill -f '/maixapp/apps/{app_id}/'",
            f"pkill -f './{exec_name}'",
            f"pkill -x '{exec_name}'",
        ]
        for cmd in cmds:
            ret = os.system(cmd)
            print(f"exit_app stop process ret={ret}, cmd={cmd}")
            if ret == 0:
                if self.last_launched_app_id == app_id:
                    self.last_launched_app_id = None
                    self.last_launched_exec_name = None
                return True
        return False

    def process_req(self, cmd: int, req_body: bytes) -> Optional[bytes]:

        if cmd == APP_CMD_SNAP:
            snap_path = self.handle_snap()
            return self.encode_resp_ok(cmd, snap_path)

        if cmd == CMD_SET_REPORT:
            return self.encode_resp_err(cmd, int(Err.ERR_NOT_IMPL), b"set_report not impl")

        if cmd == CMD_APP_LIST:
            apps = self.load_apps()
            return self.encode_resp_ok(cmd, self.encode_app_list_body(apps))

        if cmd == CMD_CUR_APP_INFO:
            apps = self.load_apps()
            curr_id = app.app_id()
            app_ids = list(apps.keys())
            if curr_id in app_ids:
                idx = app_ids.index(curr_id)
            else:
                idx = 0xFF
            body = self.encode_app_info_body(idx, curr_id, apps)
            return self.encode_resp_ok(cmd, body)

        if cmd == CMD_APP_INFO:
            if len(req_body) < 1:
                return self.encode_resp_err(cmd, int(Err.ERR_ARGS), b"invalid app_info body")

            apps = self.load_apps()
            app_ids = list(apps.keys())
            idx = req_body[0]

            if idx != 0xFF:
                if idx >= len(app_ids):
                    return self.encode_resp_err(cmd, int(Err.ERR_ARGS), b"invalid app idx")
                app_id = app_ids[idx]
                resp = self.encode_app_info_body(idx, app_id, apps)
                return self.encode_resp_ok(cmd, resp)

            app_id = req_body[1:].decode("utf-8", errors="ignore").rstrip("\x00")
            if app_id not in apps:
                return self.encode_resp_err(cmd, int(Err.ERR_ARGS), b"app_id not found")
            resp = self.encode_app_info_body(app_ids.index(app_id), app_id, apps)
            return self.encode_resp_ok(cmd, resp)

        if cmd == CMD_START_APP:
            target_app_id, err_text, app_func = self.parse_start_app_body(req_body)
            if target_app_id is None:
                return self.encode_resp_err(cmd, int(Err.ERR_ARGS), err_text.encode("utf-8"))

            self.pending_switch_app = (target_app_id, app_func)
            return self.encode_resp_ok(cmd, b"")

        if cmd == CMD_EXIT_APP:
            app_id = req_body.decode("utf-8", errors="ignore").rstrip("\x00").strip()
            if not app_id:
                return self.encode_resp_err(cmd, int(Err.ERR_ARGS), b"exit_app requires app_id body")

            stopped = self.stop_app_by_id(app_id)
            if not stopped:
                return self.encode_resp_err(cmd, int(Err.ERR_ARGS), b"exit_app target not running or not found")
            return self.encode_resp_ok(cmd, b"stopped")

        return self.encode_resp_err(cmd, int(Err.ERR_NOT_IMPL), b"cmd not support")

    def apply_pending_actions(self):
        if self.pending_switch_app is None:
            return

        app_id, app_func = self.pending_switch_app
        self.pending_switch_app = None

        apps = self.load_apps()
        if app_id not in apps:
            print(f"switch_app aborted: app_id not found in app.info: {app_id}")
            return

        exec_name = apps[app_id].get("exec", "")
        exec_path = f"/maixapp/apps/{app_id}/{exec_name}" if exec_name else ""
        print(f"switch_app request: app_id={app_id}, app_func={app_func}, exec={exec_path}")
        try:
            curr = app.app_id()
            print(f"switch_app before current_app={curr}")
        except Exception:
            pass

        try:
            if len(app_func) > 0:
                try:
                    app.switch_app(app_id, app_func) # type: ignore
                except TypeError:
                    app.switch_app(app_id)
            else:
                app.switch_app(app_id)

            switched_ok = False
            try:
                time.sleep_ms(200)
                curr2 = app.app_id()
                print(f"switch_app after current_app={curr2}, need_exit={app.need_exit()}")
                switched_ok = (curr2 == app_id)
            except Exception:
                pass

            if switched_ok:
                return

            if app.need_exit():
                try:
                    app.set_exit_flag(False)
                    print("switch_app fallback: clear exit flag")
                except Exception:
                    pass

            app_dir = f"/maixapp/apps/{app_id}"
            if not exec_name:
                print("switch_app fallback failed: app.info has empty exec")
                return

            if not os.path.isdir(app_dir):
                print(f"switch_app fallback failed: app dir not exists: {app_dir}")
                return

            cmd = (
                f"cd {app_dir} && "
                f"chmod +x ./{exec_name} >/dev/null 2>&1; "
                f"nohup ./{exec_name} >/tmp/{app_id}.log 2>&1 & echo $! > {self.pid_file_path(app_id)}"
            )
            ret = os.system(cmd)
            print(f"switch_app fallback launch ret={ret}, cmd={cmd}")
            if ret == 0:
                self.last_launched_app_id = app_id
                self.last_launched_exec_name = exec_name
        except Exception as e:
            print("switch_app failed:", e)

    def serve_client(self):
        assert self.conn is not None
        self.conn.settimeout(0.2)
        self.rx_buffer.clear()

        while not app.need_exit() and not self.should_exit:
            try:
                data = self.conn.recv(self.buff_size)
                if not data:
                    print("client disconnected")
                    break

                print("[rx]", self.hex_bytes(data))

                self.rx_buffer.extend(data)
                while True:
                    frame = self.decode_one_frame()
                    if frame is None:
                        break

                    flags, cmd, req_body = frame

                    is_resp = (flags & FLAG_IS_RESP) != 0
                    is_report = (flags & FLAG_IS_REPORT) != 0

                    if (not is_resp) and (not is_report):
                        tx = self.process_req(cmd, req_body)
                        if tx:
                            self.send(tx)
                            self.apply_pending_actions()
                    elif is_report:
                        self.send(self.encode_resp_ok(cmd, b"1"))
                    else:
                        pass

            except socket.timeout:
                continue
            except Exception as e:
                print("serve client error:", e)
                break

        self.close_client()

    def run(self):
        self.server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server.bind((self.host, self.port))
        self.server.listen(1)
        self.server.settimeout(0.5)

        print(f"[maixcam2] protocol tcp server listen at {self.host}:{self.port}")
        print("[maixcam2] wait host connect ...")

        while not app.need_exit() and not self.should_exit:
            try:
                conn, addr = self.server.accept()
                self.conn = conn
                self.addr = addr
                print(f"[maixcam2] client connected: {addr}")
                self.serve_client()
            except socket.timeout:
                continue
            except Exception as e:
                print("accept error:", e)

        self.close()
        print("[maixcam2] server exit")


def main():
    server = MaixCam2ProtocolServer(host="0.0.0.0", port=5555, buff_size=4096)
    server.run()


if __name__ == "__main__":
    main()
