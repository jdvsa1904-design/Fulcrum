import io
import socket
import contextlib
import traceback

try:
    from PySide6 import QtCore
except ImportError:
    from PySide2 import QtCore

PORT = 7003
_srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
_srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
_srv.bind(("127.0.0.1", PORT))
_srv.listen(4)
_srv.setblocking(False)
_ns = {"__name__": "fulcrum_bridge"}


def _serve():
    try:
        conn, _ = _srv.accept()
    except BlockingIOError:
        return
    conn.settimeout(10)
    data = b""
    try:
        while not data.endswith(b"\n\x04\n"):
            chunk = conn.recv(65536)
            if not chunk:
                break
            data += chunk
        code = data[:-3].decode("utf-8") if data.endswith(b"\n\x04\n") else data.decode("utf-8")
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            try:
                exec(code, _ns)
            except Exception:
                traceback.print_exc(file=buf)
        conn.sendall(buf.getvalue().encode("utf-8"))
    finally:
        conn.close()


_timer = QtCore.QTimer()
_timer.timeout.connect(_serve)
_timer.start(50)
print("Fulcrum bridge listening on 127.0.0.1:%d" % PORT)
