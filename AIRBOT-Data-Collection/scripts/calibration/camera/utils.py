import math


def equivalent_focal_length_to_fov(f_eq: int, direction: str = "v"):
    """
    通过等效焦距计算 FOV
    :param f_eq: 等效焦距 (单位: mm)
    :param direction: 'horizontal', 'vertical', 或 'diagonal'
    :return: 视场角 (角度单位)
    """
    # 全画幅传感器的标准尺寸 (mm)
    sensor_dims = {
        "h": 36,
        "v": 24,
        "d": 43.267,  # sqrt(36^2 + 24^2)
    }

    if direction not in sensor_dims:
        raise ValueError(
            "Invalid direction. Use 'horizontal', 'vertical', or 'diagonal'."
        )

    L = sensor_dims[direction]
    fov_rad = 2 * math.atan(L / (2 * f_eq))

    return math.degrees(fov_rad)


if __name__ == "__main__":
    vfovs = {}
    for f in {26, 35}:
        vfovs[f] = equivalent_focal_length_to_fov(f, "v")
    print(vfovs)
