import socket
import sys

PORTS = {"max": 7003, "maya": 7002}


def run(app, code, timeout=60):
    s = socket.create_connection(("127.0.0.1", PORTS[app]), 5)
    s.settimeout(timeout)
    if app == "maya":
        s.sendall(code.encode("utf-8"))
    else:
        s.sendall(code.encode("utf-8") + b"\n\x04\n")
    out = b""
    while True:
        try:
            chunk = s.recv(65536)
        except socket.timeout:
            break
        if not chunk:
            break
        out += chunk
        if app == "maya":
            break
    s.close()
    return out.decode("utf-8", "replace")


if __name__ == "__main__":
    print(run(sys.argv[1], sys.stdin.read()))
