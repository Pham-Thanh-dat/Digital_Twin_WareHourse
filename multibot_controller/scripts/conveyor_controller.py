#!/usr/bin/env python3
"""Dieu khien CA 4 bang chuyen trong warehouse_layout2.world bang 1 tien
trinh Python DUY NHAT, dung lich trinh TUYET DOI (absolute deadline) thay
vi 'sleep N roi sleep N' lap lai nhu ban bash cu.

TAI SAO BAN BASH CU BI LECH PHA (khong dong bo):
Ban cu dung 16 vong lap bash nen doc lap, moi vong chi biet "sleep X giay
roi goi lenh". Nhung ban than lenh goi (subprocess ign service) ton them
~100-300ms MOI LAN, va thoi gian nay KHONG duoc tru vao X -> moi chu ky
thuc te dai hon du dinh mot chut -> sai so CONG DON dan qua nhieu vong,
cang chay lau cang lech nhieu giua cac hop/bang chuyen voi nhau.

CACH SUA: dung 1 tien trinh Python duy nhat, luu "thoi diem tuyet doi can
reset tiep theo" cho tung hop (theo dong ho he thong), moi vong lap CHI
sleep dung khoang con lai toi thoi diem do (KHONG CONG DON sai so), dam
bao moi hop luon dung nhip bat ke lenh goi mat bao lau.
"""

import heapq
import subprocess
import time

WORLD = "warehouse_layout2"
SPEED = 0.4
PACKAGE_Z = 0.895
SAFETY_MARGIN = 1.5   # giay, tru bot de reset SOM hon truoc khi cham puli cuoi

# ==== Dinh nghia 4 bang chuyen: axis 'x' hoac 'y' la truc hop truot theo ====
BELTS = [
    # (ten, truc truot, gia_tri_truc_co_dinh, diem_dau_tren_truc_truot, chieu_dai, so_hop)
    {"name": "inbound_belt",  "axis": "x", "fixed": 9.00,  "start": 0.00,  "length": 23.6, "n": 8},
    {"name": "outbound_belt", "axis": "x", "fixed": 9.00,  "start": 27.60, "length": 23.6, "n": 8},
    {"name": "west_belt",     "axis": "y", "fixed": -3.00, "start": 0.00,  "length": 18.0, "n": 8},
    {"name": "east_belt",     "axis": "y", "fixed": 54.20, "start": 0.00,  "length": 18.0, "n": 8},
]


def run(cmd):
    subprocess.run(cmd, shell=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


def set_velocity(model, axis, speed):
    if axis == "x":
        lin = f"linear: {{x: {speed}, y: 0.0, z: 0.0}}"
    else:
        lin = f"linear: {{x: 0.0, y: {speed}, z: 0.0}}"
    cmd = (
        f"ign topic -t /model/{model}/cmd_vel -m ign_msgs.Twist "
        f"-p \"{lin}\" --num 1"
    )
    run(cmd)


def set_pose(model, x, y):
    cmd = (
        f"ign service -s /world/{WORLD}/set_pose "
        f"--reqtype ign_msgs.Pose --reptype ign_msgs.Boolean --timeout 500 "
        f"--req 'name: \"{model}\" position: {{x: {x}, y: {y}, z: {PACKAGE_Z}}}'"
    )
    run(cmd)


def coords_for(belt, position_on_axis):
    """Tra ve (x, y) that su dua tren truc truot va vi tri doc truc do."""
    if belt["axis"] == "x":
        return position_on_axis, belt["fixed"]
    else:
        return belt["fixed"], position_on_axis


def reset_package(belt, i, position_on_axis):
    model = f"{belt['name']}_pkg{i}"
    x, y = coords_for(belt, position_on_axis)
    set_pose(model, x, y)
    set_velocity(model, belt["axis"], SPEED)


def main():
    print("=== Dat van toc ban dau cho tat ca hop (goi lap lai cho chac) ===")
    for belt in BELTS:
        for i in range(belt["n"]):
            model = f"{belt['name']}_pkg{i}"
            for _ in range(3):
                set_velocity(model, belt["axis"], SPEED)
                time.sleep(0.05)

    print("=== Dong bo vi tri chinh xac ban dau cho tat ca hop ===")
    events = []  # heap cua (thoi_diem_tuyet_doi, belt_index, package_index)
    now = time.monotonic()

    for bi, belt in enumerate(BELTS):
        spacing = belt["length"] / belt["n"]
        full_cycle = max(1.0, belt["length"] / SPEED - SAFETY_MARGIN)
        for i in range(belt["n"]):
            offset = spacing * (i + 0.5)
            pos = belt["start"] + offset
            reset_package(belt, i, pos)

            first_interval = max(0.5, (belt["length"] - offset) / SPEED - SAFETY_MARGIN)
            deadline = now + first_interval
            heapq.heappush(events, (deadline, bi, i, full_cycle))

    print(f"=== Bat dau vong lap dong bo (lich trinh tuyet doi, khong con lech pha) ===")
    print("Nhan Ctrl+C de dung.")

    try:
        while True:
            deadline, bi, i, full_cycle = heapq.heappop(events)
            wait = deadline - time.monotonic()
            if wait > 0:
                time.sleep(wait)

            belt = BELTS[bi]
            reset_package(belt, i, belt["start"])

            # Lich lai su kien TIEP THEO dua tren deadline TUYET DOI cu, khong
            # phai "now + full_cycle" (tranh cong don sai so tu thoi gian xu ly)
            next_deadline = deadline + full_cycle
            heapq.heappush(events, (next_deadline, bi, i, full_cycle))
    except KeyboardInterrupt:
        print("\nDa dung.")


if __name__ == "__main__":
    main()
