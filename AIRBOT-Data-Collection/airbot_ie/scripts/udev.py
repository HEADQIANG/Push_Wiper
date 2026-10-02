import subprocess
import shutil
import os
import pyudev


def get_camera_usb_mapping():
    """
    Get a mapping of camera devices to their USB paths.
    This function uses udevadm to query the system for all video devices
    and retrieves their USB path information.
    Returns:
        dict: A dictionary mapping video device paths to their USB paths.
    """
    # Check if udevadm is available
    if not shutil.which("udevadm"):
        raise OSError("udevadm is not available on this system.")
    mapping = {}
    video_devices = [d for d in os.listdir("/dev") if d.startswith("video")]
    for device in video_devices:
        dev_path = f"/dev/{device}"
        try:
            cmd = ["udevadm", "info", "--query=all", "--name=" + dev_path]
            output = subprocess.check_output(cmd, universal_newlines=True)
            for line in output.split("\n"):
                if "ID_PATH=" in line:
                    usb_path = line.strip().split("=")[1]
                    mapping[dev_path] = usb_path
                    break
        except subprocess.CalledProcessError:
            continue
    return mapping


def find_usb_devices_by_vid_pid(vid: str, pid: str) -> list:
    """Find USB devices by vendor ID and product ID.
    Args:
        vid (str): Vendor ID.
        pid (str): Product ID.
    Returns:
        list: List of USB device paths.
    """
    context = pyudev.Context()
    devices = []
    for device in context.list_devices(subsystem="usb"):
        vendor_id = device.get("ID_VENDOR_ID")
        product_id = device.get("ID_MODEL_ID")
        if vendor_id and product_id:
            device_id = f"{vendor_id}:{product_id}"
            if device_id in {vid, pid}:
                busnum = device.get("BUSNUM")
                devnum = device.get("DEVNUM")
                if busnum and devnum:
                    devices.append(f"{busnum}/{devnum}")
    return devices


if __name__ == "__main__":
    from pprint import pprint

    pprint(get_camera_usb_mapping())
    devices = find_usb_devices_by_vid_pid("0483:0000", "1d50:606f")
    print(f"Detected USB devices: {devices}")
