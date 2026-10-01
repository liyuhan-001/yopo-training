import csv
import math
import sys

if len(sys.argv) != 5:
    print("用法：python3 analyze_odom.py CSV文件 目标X 目标Y 目标Z")
    sys.exit(1)


csv_file = sys.argv[1]

target = (
    float(sys.argv[2]),
    float(sys.argv[3]),
    float(sys.argv[4]),
)


with open(csv_file, newline="", encoding="utf-8-sig") as f:
    reader = csv.DictReader(f)
    fields = reader.fieldnames or []
    def find_field(suffix):
        for name in fields:
            if name.endswith(suffix):
                return name
        raise RuntimeError("找不到字段：" + suffix)
    x_key = find_field("pose.position.x")
    y_key = find_field("pose.position.y")
    z_key = find_field("pose.position.z")
    points = []


    for row in reader:
        try:
            point = (
                float(row[x_key]),
                float(row[y_key]),
                float(row[z_key]),
            )
        except (ValueError, TypeError, KeyError):
            continue

        if all(math.isfinite(value) for value in point):
            points.append(point)

if len(points) < 2:
    raise RuntimeError("有效里程计数据不足，无法计算")
path_length = 0.0
for first, second in zip(points, points[1:]):
    path_length += math.sqrt(
        (second[0] - first[0]) ** 2
        + (second[1] - first[1]) ** 2
        + (second[2] - first[2]) ** 2
    )

start = points[0]
end = points[-1]

straight_distance = math.sqrt(
    (end[0] - start[0]) ** 2
    + (end[1] - start[1]) ** 2
    + (end[2] - start[2]) ** 2
)
target_error = math.sqrt(
    (end[0] - target[0]) ** 2
    + (end[1] - target[1]) ** 2
    + (end[2] - target[2]) ** 2
)
print(f"有效采样点数：{len(points)}")
print(f"起点：({start[0]:.3f}, {start[1]:.3f}, {start[2]:.3f})")
print(f"实际终点：({end[0]:.3f}, {end[1]:.3f}, {end[2]:.3f})")
print(f"目标点：({target[0]:.3f}, {target[1]:.3f}, {target[2]:.3f})")
print(f"路径长度：{path_length:.3f} m")
print(f"起终点直线距离：{straight_distance:.3f} m")
print(f"终点到目标误差：{target_error:.3f} m")

