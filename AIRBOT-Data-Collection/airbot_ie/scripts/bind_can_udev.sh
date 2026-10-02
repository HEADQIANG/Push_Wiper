#!/bin/bash

CAN_RULE_PATH=/etc/udev/rules.d/91-usb-can-airbot.rules
SLCAN_RULE_PATH=/etc/udev/rules.d/91-usb-slcan-airbot.rules

TARGET_NAMES=()

# 解析命令行参数
while [[ "$#" -gt 0 ]]; do
    case $1 in
        rm)
            if [ -z "$2" ] || [ "$2" = "slcan" ]; then
                rm -f "$SLCAN_RULE_PATH"
                echo "Removed $SLCAN_RULE_PATH"
            fi
            if [ -z "$2" ] || [ "$2" = "can" ]; then
                rm -f "$CAN_RULE_PATH"
                echo "Removed $CAN_RULE_PATH"
            fi
            exit 0
            ;;
        --target)
            shift
            while [[ "$#" -gt 0 && ! "$1" =~ ^-- ]]; do
                TARGET_NAMES+=("$1")
                shift
            done
            ;;
        *)
            echo "Unknown argument: $1"
            exit 1
            ;;
    esac
done

# 检查 root 权限
if [ $EUID -ne 0 ]; then
    echo "This script must be run as root. Use sudo to run the script."
    exit 1
fi

echo "Detecting connected USB2CAN devices..."

# 提取符合条件的 lsusb 信息
USB_IDS=$(lsusb | grep -E "ID (0483:0000|1d50:606f)" | awk '{print $2, $4}' | sed 's/://')

# 使用 dmesg 获取插入时间并排序
DEVICE_LIST=()
while read -r BUS DEV; do
    USB_PATH="/dev/bus/usb/$BUS/$DEV"
    SYS_PATH=$(udevadm info --query=path --name="$USB_PATH" 2>/dev/null)
    if [ -z "$SYS_PATH" ]; then continue; fi

    DEV_INFO=$(udevadm info --query=property --path="$SYS_PATH")
    SERIAL=$(echo "$DEV_INFO" | grep 'ID_SERIAL_SHORT=' | cut -d= -f2)
    if [ -z "$SERIAL" ]; then continue; fi

    # 找到 dmesg 中最后一次插入记录的时间
    LAST_TIME=$(dmesg | grep "$SERIAL" | tail -n1 | grep -oP '^\[\s*\K[0-9.]+')
    if [ -z "$LAST_TIME" ]; then LAST_TIME=0; fi

    DEVICE_LIST+=("$LAST_TIME|$USB_PATH|$SERIAL")
done <<< "$USB_IDS"

# 根据时间排序
IFS=$'\n' SORTED_DEVICES=($(printf "%s\n" "${DEVICE_LIST[@]}" | sort -n))

if [ ${#SORTED_DEVICES[@]} -eq 0 ]; then
    echo "No USB2CAN devices detected."
    exit 1
fi

if [ ${#TARGET_NAMES[@]} -ne ${#SORTED_DEVICES[@]} ]; then
    echo "Error: Number of --target names does not match number of detected devices"
    exit 1
fi

# 创建 udev 规则和服务
for idx in "${!SORTED_DEVICES[@]}"; do
    entry="${SORTED_DEVICES[$idx]}"
    INSERT_TIME="${entry%%|*}"
    rest="${entry#*|}"
    USB_PATH="${rest%%|*}"
    SERIAL="${rest##*|}"
    CAN_NAME="${TARGET_NAMES[$idx]}"

    DEV_INFO=$(udevadm info --query=property --path=$(udevadm info --query=path --name=$USB_PATH))
    VENDOR_ID=$(echo "$DEV_INFO" | grep 'ID_VENDOR_ID=' | cut -d= -f2)
    PRODUCT_ID=$(echo "$DEV_INFO" | grep 'ID_MODEL_ID=' | cut -d= -f2)

    if [[ "$VENDOR_ID" == "0483" && "$PRODUCT_ID" == "0000" ]]; then
        # SLCAN 设备处理
        cat >>"$SLCAN_RULE_PATH" <<EOL
ACTION=="add", SUBSYSTEM=="tty", ATTRS{idVendor}=="$VENDOR_ID", ATTRS{idProduct}=="$PRODUCT_ID", ATTRS{serial}=="$SERIAL", SYMLINK+="$CAN_NAME", GROUP="dialout", MODE="0777", TAG+="systemd", ENV{SYSTEMD_WANTS}="slcan_$CAN_NAME@.service"
EOL

        cat >/etc/systemd/system/slcan_$CAN_NAME@.service <<EOL
[Unit]
Description=SocketCAN device $CAN_NAME
After=dev-$CAN_NAME.device
BindsTo=dev-$CAN_NAME.device

[Service]
ExecStart=/usr/local/bin/slcan_add_$CAN_NAME.sh
Type=forking
EOL

        cat >/usr/local/bin/slcan_add_$CAN_NAME.sh <<EOL
#!/bin/bash
/usr/bin/slcand -o -c -f -s8 -S 3000000 /dev/$CAN_NAME $CAN_NAME
sleep 1
/usr/sbin/ip link set up $CAN_NAME
/usr/sbin/ip link set $CAN_NAME txqueuelen 1000
EOL

        chmod +x /usr/local/bin/slcan_add_$CAN_NAME.sh
        chmod +x /etc/systemd/system/slcan_$CAN_NAME@.service
        echo "Created slcan udev rule and service for $CAN_NAME"
    else
        # CAN 设备处理
        cat >>"$CAN_RULE_PATH" <<EOL
ACTION=="add", SUBSYSTEM=="net", ATTRS{idVendor}=="$VENDOR_ID", ATTRS{idProduct}=="$PRODUCT_ID", ATTRS{serial}=="$SERIAL", NAME="$CAN_NAME", RUN+="/sbin/ip link set $CAN_NAME up type can bitrate 1000000", RUN+="/sbin/ip link set $CAN_NAME txqueuelen 1000"
EOL
        chmod +x "$CAN_RULE_PATH"
        echo "Created CAN udev rule for $CAN_NAME"
    fi
done

# Reload udev rules
udevadm control --reload-rules
udevadm trigger
echo "Udev rules reloaded successfully. Reconnect your USB2CAN devices."
