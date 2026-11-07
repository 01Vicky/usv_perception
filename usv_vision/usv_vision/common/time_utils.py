def ros_time_to_dict(stamp) -> dict:
    return {
        "sec": int(stamp.sec),
        "nanosec": int(stamp.nanosec),
    }


def ros_delta_ms(start_stamp, end_stamp) -> float:
    start_ns = int(start_stamp.sec) * 1_000_000_000 + int(start_stamp.nanosec)
    end_ns = int(end_stamp.sec) * 1_000_000_000 + int(end_stamp.nanosec)
    return round((end_ns - start_ns) / 1_000_000.0, 3)
