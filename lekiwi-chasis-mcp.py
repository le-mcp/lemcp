import sys
import os
current_dir = os.path.dirname(os.path.abspath(__file__))
target_dir = os.path.join(current_dir, '../lerobot_alohamini/lerobot/common/robot_devices/motors')
sys.path.append(os.path.abspath(target_dir))

from mcp.server.fastmcp import FastMCP
from feetech import FeetechMotorsBus
from lerobot.common.robot_devices.motors.configs import FeetechMotorsBusConfig
import math, numpy as np

mcp = FastMCP("so100_base")

WHEEL_ORDER = ["left_wheel", "back_wheel", "right_wheel"]
BUS = None

def init_bus():
    global BUS
    if BUS: return
    cfg = FeetechMotorsBusConfig(
        port="/dev/ttyACM0",
        motors={
            "left_wheel":  (8,  "sts3215"),
            "back_wheel":  (9,  "sts3215"),
            "right_wheel": (10, "sts3215"),
        },
    )
    BUS = FeetechMotorsBus(cfg)
    BUS.connect()
    print('connected!')
    BUS.write("Lock", 0)
    BUS.write("Mode", [1, 1, 1], WHEEL_ORDER)
    BUS.write("Lock", 1)

def degps_to_raw(dps: float) -> int:
    steps = 4096 / 360
    raw = int(abs(dps) * steps)
    raw = min(raw, 0x7FFF)
    return raw | 0x8000 if dps < 0 else raw

def body_to_wheel_raw(vx, vy, omega_deg, r=0.05, base=0.125):
    omega = math.radians(omega_deg)
    v = np.array([vx, vy, omega])
    angles = np.radians([300, 180, 60])
    M = np.array([[math.cos(a), math.sin(a), base] for a in angles])
    wheel_lin = M @ v            # m/s
    wheel_dps = wheel_lin / r * (180 / math.pi)
    return {n: degps_to_raw(d) for n, d in zip(WHEEL_ORDER, wheel_dps)}

def send_raw(cmds: dict):
    raw_vals = [cmds[n] for n in WHEEL_ORDER]
    BUS.write("Goal_Speed", raw_vals, WHEEL_ORDER)

@mcp.tool()
def move_robot(direction: str, speed: float = 0.2) -> str:
    """
    direction ∈ {forward, backward, left, right, rotate_left, rotate_right}
    speed     线速度 (m/s) 或角速度 (deg/s) 的标量
    """
    init_bus()
    vx = vy = omega = 0.0
    if direction == "forward":   vy =  speed
    elif direction == "backward":vy = -speed
    elif direction == "left":    vx =  speed
    elif direction == "right":   vx = -speed
    elif direction == "rotate_left":  omega =  speed
    elif direction == "rotate_right": omega = -speed
    else:
        return "Invalid direction"
    send_raw(body_to_wheel_raw(vx, vy, omega))
    print('move robot!')
    return f"OK – {direction} @ {speed}"

@mcp.tool()
def stop_robot() -> str:
    """立即停车"""
    init_bus()
    send_raw({n: 0 for n in WHEEL_ORDER})
    print('stop robot!')
    return "Stopped."

if __name__ == "__main__":
    mcp.run()            # python server.py   或   mcp run server.py
