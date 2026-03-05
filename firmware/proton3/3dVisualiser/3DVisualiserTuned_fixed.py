# live_visualizer.py
# Reads CSV from Serial:
# t_ms,roll,pitch,yaw,gx,gy,gz,ax,ay,az,tempC,pressPa,alt_m,vz_mps,uR,uP
#
# Shows a rotating 3D model (OBJ/STL). Falls back to a cube if no model is provided.
#
# Install:
#   pip install pyserial numpy matplotlib
#
# Run:
#   python live_visualizer.py
#
# Set PORT below (Windows: "COM8", Linux: "/dev/ttyACM0", macOS: "/dev/tty.usbmodemXXXX")

import sys
import time
import math
import threading
from pathlib import Path
from collections import deque

import numpy as np
import serial

import matplotlib
# Robust backend selection (fixes many "white window" cases)
try:
    matplotlib.use("TkAgg")
except Exception:
    pass

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection


# ----------------------------
# Config
# ----------------------------
PORT = "COM8"          # <- change
BAUD = 115200
SER_TIMEOUT = 0.2

# --- 3D model ---
# Put an .obj or .stl file next to this script and set the filename here.
# Recommended: export your rocket as a TRIANGULATED mesh.
MODEL_FILE = ""  # e.g. "tracer.obj" or "rocket.stl". Leave empty to auto-pick first .stl/.obj in this folder.

# Scale and orientation tweaks (because CAD coordinate systems vary)
MODEL_SCALE = 1.0             # multiply all vertices by this

# If your CAD rocket points in the wrong axis, remap axes here.
# Example: (x,y,z) -> (x,z,y) would be AXIS_MAP=(0,2,1)
AXIS_MAP = (0, 1, 2)

# Optionally flip axes (multiply by -1). Example: (-x,y,z) => AXIS_SIGN=(-1,1,1)
AXIS_SIGN = (1, 1, 1)

# How to use the angles:
# "state"  -> use roll/pitch/yaw directly from MCU
# "pid"    -> visualize (roll+uR, pitch+uP) to see controller action
ANGLE_MODE = "state"   # "state" or "pid"

# If your roll/pitch/yaw are radians (they are in your Arduino), keep as False.
# If you ever send degrees, set True.
ANGLES_ARE_DEG = False

# Plot update limiting
MAX_FPS = 60


# ----------------------------
# Small helpers
# ----------------------------
def clamp(x, a, b):
    return a if x < a else (b if x > b else x)

def cube_vertices(size=1.2):
    s = size / 2.0
    return np.array([
        [-s, -s, -s],
        [ s, -s, -s],
        [ s,  s, -s],
        [-s,  s, -s],
        [-s, -s,  s],
        [ s, -s,  s],
        [ s,  s,  s],
        [-s,  s,  s],
    ], dtype=float)

CUBE_FACES = [
    [0, 1, 2, 3],
    [4, 5, 6, 7],
    [0, 1, 5, 4],
    [2, 3, 7, 6],
    [1, 2, 6, 5],
    [0, 3, 7, 4],
]

def R_x(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1,0,0],[0,c,-s],[0,s,c]], dtype=float)

def R_y(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c,0,s],[0,1,0],[-s,0,c]], dtype=float)

def R_z(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c,-s,0],[s,c,0],[0,0,1]], dtype=float)

def rotate_vertices(V, roll, pitch, yaw):
    # ZYX: yaw -> pitch -> roll
    R = R_z(yaw) @ R_y(pitch) @ R_x(roll)
    return (R @ V.T).T


# ----------------------------
# Mesh loading (OBJ / STL)
# ----------------------------
def _center_and_scale(V: np.ndarray, scale: float) -> np.ndarray:
    V = V.astype(float)
    # center at origin
    c = 0.5 * (V.min(axis=0) + V.max(axis=0))
    V = V - c
    # uniform scale to roughly fit into [-1,1]
    span = (V.max(axis=0) - V.min(axis=0)).max()
    if span > 1e-9:
        V = V / span
    V = V * (2.0 * scale)  # span -> ~2
    return V

def load_obj(path: str):
    verts = []
    faces = []
    with open(path, "r", encoding="utf-8", errors="ignore") as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            if line.startswith("v "):
                parts = line.split()
                if len(parts) >= 4:
                    verts.append([float(parts[1]), float(parts[2]), float(parts[3])])
            elif line.startswith("f "):
                parts = line.split()[1:]
                idx = []
                for p in parts:
                    # f v, f v/vt, f v//vn, f v/vt/vn
                    v = p.split("/")[0]
                    if not v:
                        continue
                    vi = int(v)
                    # OBJ indices are 1-based, negative allowed
                    if vi < 0:
                        vi = len(verts) + 1 + vi
                    idx.append(vi - 1)
                # triangulate polygon fan
                if len(idx) >= 3:
                    for k in range(1, len(idx) - 1):
                        faces.append([idx[0], idx[k], idx[k + 1]])
    if not verts or not faces:
        raise ValueError("OBJ has no vertices/faces (export as mesh, triangulated).")
    return np.array(verts, dtype=float), np.array(faces, dtype=np.int32)

def _is_binary_stl(buf: bytes) -> bool:
    # Heuristic: binary STL has 80-byte header, then uint32 triangle count, then 50 bytes per tri
    if len(buf) < 84:
        return False
    tri_count = int.from_bytes(buf[80:84], "little", signed=False)
    expected = 84 + tri_count * 50
    return expected == len(buf)

def load_stl(path: str):
    with open(path, "rb") as f:
        buf = f.read()

    if _is_binary_stl(buf):
        tri_count = int.from_bytes(buf[80:84], "little", signed=False)
        verts = []
        faces = []
        off = 84
        for _ in range(tri_count):
            # 12 bytes normal, 36 bytes vertices, 2 bytes attr
            off += 12
            v0 = np.frombuffer(buf, dtype=np.float32, count=3, offset=off); off += 12
            v1 = np.frombuffer(buf, dtype=np.float32, count=3, offset=off); off += 12
            v2 = np.frombuffer(buf, dtype=np.float32, count=3, offset=off); off += 12
            off += 2
            base = len(verts)
            verts.extend([v0.tolist(), v1.tolist(), v2.tolist()])
            faces.append([base, base + 1, base + 2])
        return np.array(verts, dtype=float), np.array(faces, dtype=np.int32)

    # ASCII STL
    text = buf.decode("utf-8", errors="ignore").splitlines()
    verts = []
    faces = []
    cur = []
    for line in text:
        line = line.strip()
        if line.lower().startswith("vertex "):
            parts = line.split()
            if len(parts) >= 4:
                cur.append([float(parts[1]), float(parts[2]), float(parts[3])])
        if line.lower().startswith("endloop"):
            if len(cur) == 3:
                base = len(verts)
                verts.extend(cur)
                faces.append([base, base + 1, base + 2])
            cur = []
    if not verts or not faces:
        raise ValueError("STL has no triangles.")
    return np.array(verts, dtype=float), np.array(faces, dtype=np.int32)

def resolve_model_path() -> Path:
    """Resolve MODEL_FILE relative to the script directory.
    If MODEL_FILE is empty, auto-pick the first .stl/.obj found next to this script.
    """
    script_dir = Path(__file__).resolve().parent
    if MODEL_FILE and MODEL_FILE.strip():
        p = Path(MODEL_FILE.strip())
        if not p.is_absolute():
            p = (script_dir / p).resolve()
        return p

    # auto-pick
    for ext in (".stl", ".obj", ".STL", ".OBJ"):
        hits = sorted(script_dir.glob(f"*{ext}"))
        if hits:
            return hits[0].resolve()
    return Path()  # empty => not found


def load_mesh_or_cube():
    """Returns (V0, faces_idx, kind, info) where faces_idx is Nx3 (triangles)."""
    try:
        p = resolve_model_path()
        if not p or not str(p):
            raise FileNotFoundError(
                "No model found. Set MODEL_FILE or place an .stl/.obj next to this script."
            )
        if not p.exists():
            raise FileNotFoundError(f"Model file not found: {p}")

        ext = p.suffix.lower()
        if ext == ".obj":
            V, F = load_obj(str(p))
            kind = "obj"
        elif ext == ".stl":
            V, F = load_stl(str(p))
            kind = "stl"
        elif ext in (".glb", ".gltf"):
            # Optional: only works if trimesh is installed.
            try:
                import trimesh  # type: ignore
            except Exception:
                raise ValueError("GLB/GLTF needs 'trimesh'. Install: pip install trimesh")
            m = trimesh.load(str(p), force="mesh")
            if hasattr(m, "geometry") and isinstance(m.geometry, dict) and m.geometry:
                m = trimesh.util.concatenate(tuple(m.geometry.values()))
            if not hasattr(m, "vertices") or not hasattr(m, "faces"):
                raise ValueError("GLB did not load as a mesh.")
            V = np.asarray(m.vertices, dtype=float)
            F = np.asarray(m.faces, dtype=np.int32)
            kind = "glb"
        else:
            raise ValueError(f"Unsupported model extension: {ext} (use .obj or .stl)")

        # axis map + sign
        V = V[:, list(AXIS_MAP)]
        V = V * np.array(AXIS_SIGN, dtype=float)
        V = _center_and_scale(V, MODEL_SCALE)

        info = f"{kind.upper()} | {p.name} | {len(V)} verts | {len(F)} tris"
        print(f"[MODEL] Loaded {info} from {p}")
        return V, F, kind, info

    except Exception as e:
        reason = str(e)
        print(f"[MODEL] Using cube fallback. Reason: {reason}")
        V = cube_vertices(size=1.2)
        tris = []
        for f in CUBE_FACES:
            tris.append([f[0], f[1], f[2]])
            tris.append([f[0], f[2], f[3]])
        info = f"CUBE fallback | {reason}"
        return V, np.array(tris, dtype=np.int32), "cube", info
# ----------------------------
# Serial reader (thread)
# ----------------------------
latest = {
    "ok": False,
    "t_ms": 0,
    "roll": 0.0, "pitch": 0.0, "yaw": 0.0,
    "gx": 0.0, "gy": 0.0, "gz": 0.0,
    "ax": 0.0, "ay": 0.0, "az": 1.0,
    "tempC": 0.0, "pressPa": 0.0, "alt_m": 0.0, "vz_mps": 0.0,
    "uR": 0.0, "uP": 0.0,
    "last_line": ""
}
lock = threading.Lock()

def parse_csv_line(line: str):
    # returns dict or None
    parts = line.strip().split(",")
    if len(parts) != 16:
        return None

    try:
        t_ms   = int(parts[0])
        roll   = float(parts[1])
        pitch  = float(parts[2])
        yaw    = float(parts[3])
        gx     = float(parts[4])
        gy     = float(parts[5])
        gz     = float(parts[6])
        ax     = float(parts[7])
        ay     = float(parts[8])
        az     = float(parts[9])
        tempC  = float(parts[10])
        press  = float(parts[11])
        alt_m  = float(parts[12])
        vz_mps = float(parts[13])
        uR     = float(parts[14])
        uP     = float(parts[15])
    except ValueError:
        return None

    if ANGLES_ARE_DEG:
        roll  = math.radians(roll)
        pitch = math.radians(pitch)
        yaw   = math.radians(yaw)

    return {
        "t_ms": t_ms,
        "roll": roll, "pitch": pitch, "yaw": yaw,
        "gx": gx, "gy": gy, "gz": gz,
        "ax": ax, "ay": ay, "az": az,
        "tempC": tempC, "pressPa": press, "alt_m": alt_m, "vz_mps": vz_mps,
        "uR": uR, "uP": uP,
    }

def serial_worker():
    try:
        ser = serial.Serial(PORT, BAUD, timeout=SER_TIMEOUT)
    except Exception as e:
        print(f"[SERIAL] Failed to open {PORT} @ {BAUD}: {e}")
        print("Fix: close Arduino Serial Monitor/Plotter, check port name, driver, permissions.")
        return

    print(f"[SERIAL] Opened {PORT} @ {BAUD}")
    ser.reset_input_buffer()

    # Skip header lines until we see a valid CSV frame
    while True:
        try:
            raw = ser.readline()
        except Exception as e:
            print(f"[SERIAL] Read error: {e}")
            break

        if not raw:
            continue

        line = raw.decode(errors="ignore").strip()
        if not line:
            continue

        data = parse_csv_line(line)
        if data is None:
            # ignore non-CSV (header / debug)
            continue

        with lock:
            latest.update(data)
            latest["ok"] = True
            latest["last_line"] = line

def start_serial_thread():
    th = threading.Thread(target=serial_worker, daemon=True)
    th.start()
    return th


# ----------------------------
# Main visualizer
# ----------------------------
def main():
    start_serial_thread()

    # 3D setup
    plt.ion()
    fig = plt.figure("IMU 3D Visualizer")
    ax3 = fig.add_subplot(111, projection="3d")
    ax3.set_box_aspect((1, 1, 1))
    ax3.set_xlim(-1, 1)
    ax3.set_ylim(-1, 1)
    ax3.set_zlim(-1, 1)
    ax3.set_xlabel("X")
    ax3.set_ylabel("Y")
    ax3.set_zlabel("Z")

    V0, F0, model_kind, model_info = load_mesh_or_cube()

    poly = Poly3DCollection([], alpha=0.35, linewidths=0.2)
    ax3.add_collection3d(poly)

    last_draw = 0.0
    last_t_ms = None

    # If no serial data, show a small hint
    txt = ax3.text2D(0.02, 0.02, "", transform=ax3.transAxes)
    model_txt = ax3.text2D(0.02, 0.95, model_info, transform=ax3.transAxes)

    plt.show(block=False)

    while True:
        # Keep UI responsive
        plt.pause(0.001)

        now = time.time()
        if now - last_draw < (1.0 / MAX_FPS):
            continue
        last_draw = now

        with lock:
            ok = latest["ok"]
            t_ms = latest["t_ms"]
            roll = latest["roll"]
            pitch = latest["pitch"]
            yaw = latest["yaw"]
            uR = latest["uR"]
            uP = latest["uP"]
            alt_m = latest["alt_m"]
            vz_mps = latest["vz_mps"]
            last_line = latest["last_line"]

        if not ok:
            txt.set_text(f"{model_info}\nNo data yet. Check PORT={PORT}, close Serial Monitor.\nWaiting...")
            continue

        # Choose visualization angles
        if ANGLE_MODE == "pid":
            use_roll = roll + uR
            use_pitch = pitch + uP
            use_yaw = yaw
        else:
            use_roll = roll
            use_pitch = pitch
            use_yaw = yaw

        # Rotate model
        V = rotate_vertices(V0, roll=use_roll, pitch=use_pitch, yaw=use_yaw)
        tris = V[F0]  # (n,3,3)
        poly.set_verts(tris)

        # Titles / debug
        ax3.set_title(
            f"roll={math.degrees(roll):6.1f}°  pitch={math.degrees(pitch):6.1f}°  yaw={math.degrees(yaw):6.1f}°   "
            f"alt={alt_m:7.2f} m  vz={vz_mps:6.2f} m/s   mode={ANGLE_MODE}   model={model_kind}"
        )
        txt.set_text(f"t_ms={t_ms}  last_frame_ok\n")

        # Force draw
        fig.canvas.draw_idle()

        # detect stalled stream
        if last_t_ms is not None and t_ms == last_t_ms:
            # same frame repeated (serial stalled) -> show hint
            txt.set_text(f"Stream stalled? Close Serial Monitor. PORT={PORT}\nLast line: {last_line[:120]}")
        last_t_ms = t_ms


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nBye.")
        sys.exit(0)
