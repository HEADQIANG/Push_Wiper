import subprocess
import platform
from typing import Dict, Any, Optional
import re
import os


class SystemInfo:
    @classmethod
    def get_product(cls, with_sudo: bool = False) -> Dict[str, Any]:
        if with_sudo:
            return cls._get_product_dmidecode()
        else:
            try:
                return {
                    "product_name": subprocess.check_output(
                        "cat /sys/devices/virtual/dmi/id/product_name",
                        shell=True,
                        text=True,
                    ).strip(),
                }
            except subprocess.CalledProcessError:
                # 如果在容器中无法访问DMI信息，返回有限的信息
                return {"product_name": "Unknown (container environment)"}

    @staticmethod
    def _get_product_dmidecode():
        try:
            result = subprocess.check_output(
                "sudo dmidecode -t system", shell=True, text=True
            )
            info = {}
            for line in result.splitlines():
                if "Manufacturer:" in line:
                    info["manufacturer"] = line.strip().split(":", 1)[1].strip()
                elif "Product Name:" in line:
                    info["product_name"] = line.strip().split(":", 1)[1].strip()
                elif "Version:" in line:
                    info["version"] = line.strip().split(":", 1)[1].strip()
                elif "Serial Number:" in line:
                    info["serial_number"] = line.strip().split(":", 1)[1].strip()
                elif "UUID:" in line:
                    info["uuid"] = line.strip().split(":", 1)[1].strip()
            return info
        except Exception as e:
            print(f"Error accessing dmidecode: {e}")
            return {"product_name": "Unknown (container environment)"}

    @staticmethod
    def get_cpu():
        info = {
            "model_name": "",
            "cores_physical": "0",
            "cores_logical": "0",
        }

        try:
            with open("/proc/cpuinfo") as f:
                cpuinfo = f.read()

            for line in cpuinfo.splitlines():
                if line.startswith("model name"):
                    info["model_name"] = line.split(":", 1)[1].strip()
                elif line.startswith("siblings"):
                    info["cores_logical"] = line.split(":", 1)[1].strip()
                elif line.startswith("cpu cores"):
                    info["cores_physical"] = line.split(":", 1)[1].strip()

            # 如果未能获取物理核心数，尝试使用nproc命令
            if info["cores_physical"] == "0":
                try:
                    info["cores_physical"] = subprocess.check_output(
                        "nproc", shell=True, text=True
                    ).strip()
                except subprocess.CalledProcessError:
                    pass
        except Exception as e:
            print(f"Error reading CPU info: {e}")

        return info

    @staticmethod
    def get_memory():
        mem_info = {
            "mem_total": "Unknown",
            "mem_free": "Unknown",
            "swap_total": "0 kB",
            "swap_free": "0 kB",
        }

        try:
            with open("/proc/meminfo") as f:
                meminfo = f.read()

            for line in meminfo.splitlines():
                if "MemTotal" in line:
                    mem_info["mem_total"] = line.split(":", 1)[1].strip()
                elif "MemFree" in line:
                    mem_info["mem_free"] = line.split(":", 1)[1].strip()
                elif "SwapTotal" in line:
                    mem_info["swap_total"] = line.split(":", 1)[1].strip()
                elif "SwapFree" in line:
                    mem_info["swap_free"] = line.split(":", 1)[1].strip()
        except Exception as e:
            print(f"Error reading memory info: {e}")

        return mem_info

    @staticmethod
    def get_gpu():
        """
        读取lspci命令输出并解析VGA设备信息为结构化字典，适应容器环境

        Returns:
            dict: 包含VGA设备信息的字典，键为设备编号，值为设备属性字典
        """
        try:
            # 首先检查lspci命令是否可用
            subprocess.check_call("which lspci > /dev/null 2>&1", shell=True)

            output = subprocess.check_output(
                "lspci | grep -i 'vga\\|3d\\|display'", shell=True, text=True
            )
            if not output.strip():
                return {
                    "info": "No GPU detected or information not available in container"
                }

            lines = output.strip().split("\n")

            devices = {}
            for i, line in enumerate(lines, 1):
                # 正则表达式匹配PCI地址、设备类型、厂商、型号和修订版本
                match = re.match(
                    r"([0-9a-f:.]+)\s+([^:]+):\s+([^[]+)\[([^]]+)\](?:\s+\(rev\s+([0-9a-f]+)\))?",
                    line,
                )

                if match:
                    pci_address, device_type, vendor, model, revision = match.groups()

                    # 清理字符串
                    pci_address = pci_address.strip()
                    device_type = device_type.strip()
                    vendor = vendor.strip()
                    model = model.strip()
                    revision = revision.strip() if revision else ""

                    devices[f"device{i}"] = {
                        "pci_address": pci_address,
                        "device_type": device_type,
                        "vendor": vendor,
                        "model": model,
                        "revision": revision,
                    }
                else:
                    # 尝试另一种格式的匹配
                    match = re.match(
                        r"([0-9a-f:.]+)\s+([^:]+):\s+([^(]+)(?:\s+\(rev\s+([0-9a-f]+)\))?",
                        line,
                    )
                    if match:
                        pci_address, device_type, vendor_model, revision = (
                            match.groups()
                        )

                        # 清理字符串
                        pci_address = pci_address.strip()
                        device_type = device_type.strip()
                        vendor_model = vendor_model.strip()
                        revision = revision.strip() if revision else ""

                        # 尝试分离厂商和型号
                        parts = vendor_model.split(" ", 1)
                        vendor = parts[0]
                        model = parts[1] if len(parts) > 1 else ""

                        devices[f"device{i}"] = {
                            "pci_address": pci_address,
                            "device_type": device_type,
                            "vendor": vendor,
                            "model": model,
                            "revision": revision,
                        }

            return devices

        except subprocess.CalledProcessError:
            # 尝试使用nvidia-smi获取NVIDIA GPU信息
            try:
                subprocess.check_call("which nvidia-smi > /dev/null 2>&1", shell=True)
                output = subprocess.check_output(
                    "nvidia-smi --query-gpu=name,driver_version,memory.total --format=csv,noheader",
                    shell=True,
                    text=True,
                )

                devices = {}
                for i, line in enumerate(output.strip().split("\n"), 1):
                    parts = line.split(", ")
                    if len(parts) >= 3:
                        devices[f"device{i}"] = {
                            "vendor": "NVIDIA",
                            "model": parts[0].strip(),
                            "driver_version": parts[1].strip(),
                            "memory": parts[2].strip(),
                        }
                return devices
            except subprocess.CalledProcessError:
                return {
                    "info": "GPU information not available in container environment"
                }
        except Exception as e:
            print(f"Error getting GPU info: {e}")
            return {"info": f"Error retrieving GPU information: {str(e)}"}

    @staticmethod
    def is_in_docker() -> bool:
        """检测是否在Docker容器中运行"""
        return (
            os.path.exists("/.dockerenv")
            or os.path.isfile("/proc/self/cgroup")
            and any("docker" in line for line in open("/proc/self/cgroup"))
        )

    @staticmethod
    def get_platform() -> Dict[str, Any]:
        platform_info = platform.uname()._asdict()

        # 添加容器信息
        if SystemInfo.is_in_docker():
            platform_info["environment"] = "Docker container"

            # 尝试获取容器ID
            try:
                with open("/proc/self/cgroup") as f:
                    for line in f:
                        if "docker" in line:
                            container_id = line.split("/")[-1].strip()
                            platform_info["container_id"] = container_id
                            break
            except Exception:
                pass

        return platform_info

    @classmethod
    def all_info(cls, with_sudo: bool = False) -> Dict[str, Any]:
        info = {
            "product": cls.get_product(with_sudo),
            "cpu": cls.get_cpu(),
            "memory": cls.get_memory(),
            "gpu": cls.get_gpu(),
            "platform": cls.get_platform(),
        }

        # 添加容器环境标记
        if cls.is_in_docker():
            info["container_environment"] = True

        return info


if __name__ == "__main__":
    from pprint import pprint

    system_info = SystemInfo.all_info()
    pprint(system_info)
