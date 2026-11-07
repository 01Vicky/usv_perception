from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy


def image_qos(reliability: str = "best_effort", depth: int = 1) -> QoSProfile:
    selected = ReliabilityPolicy.BEST_EFFORT
    if str(reliability).strip().lower() == "reliable":
        selected = ReliabilityPolicy.RELIABLE
    return QoSProfile(
        reliability=selected,
        history=HistoryPolicy.KEEP_LAST,
        depth=max(1, int(depth)),
    )
