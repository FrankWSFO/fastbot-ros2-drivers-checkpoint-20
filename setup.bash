#!/bin/bash

set -e

# Setting up lidar driver

#!/bin/bash
set -e

RULE_FILE="/etc/udev/rules.d/lslidar.rules"

list_ports() { ls /dev/ttyUSB* /dev/ttyACM* 2>/dev/null | sort || true; }

echo "Unplug the lidar from the Pi, then press Enter."
read -r
BEFORE=$(list_ports)

echo "Now plug the lidar in (waiting up to 20 seconds)..."
DEV=""
for _ in $(seq 1 40); do
  sleep 0.5
  NEW=$(comm -13 <(echo "$BEFORE") <(list_ports))
  if [ -n "$NEW" ]; then
    DEV=$(echo "$NEW" | head -n1)
    break
  fi
done

if [ -z "$DEV" ]; then
  echo "No new serial device appeared."
  exit 1
fi
sleep 1  # let udev finish populating properties

VID=$(udevadm info -q property -n "$DEV" | sed -n 's/^ID_VENDOR_ID=//p')
PID=$(udevadm info -q property -n "$DEV" | sed -n 's/^ID_MODEL_ID=//p')
if [ -z "$VID" ] || [ -z "$PID" ]; then
  echo "Could not read IDs from $DEV"
  exit 1
fi
echo "Detected $DEV with $VID:$PID"

RULE="SUBSYSTEM==\"tty\", ATTRS{idVendor}==\"$VID\", ATTRS{idProduct}==\"$PID\", SYMLINK+=\"lslidar\""

if [ -f "$RULE_FILE" ] && grep -qxF "$RULE" "$RULE_FILE"; then
  echo "Rule already present."
else
  echo "$RULE" | sudo tee "$RULE_FILE" >/dev/null
  sudo udevadm control --reload-rules
  sudo udevadm trigger
  echo "Wrote $RULE_FILE"
fi

# Wait up to 5 seconds for udev to create the symlink
for _ in $(seq 1 10); do
  [ -L /dev/lslidar ] && break
  sleep 0.5
done

if [ ! -L /dev/lslidar ]; then
  echo "ERROR: /dev/lslidar was not created." >&2
  echo "Unplug and replug the lidar, then rerun this script before continuing." >&2
  exit 1
fi

ls -l /dev/lslidar

sudo udevadm control --reload-rules
sudo udevadm trigger



# Setting up camera driver:

PKGS="ros-humble-v4l2-camera ros-humble-image-transport-plugins v4l-utils"

MISSING=""
for p in $PKGS; do
  dpkg -s "$p" >/dev/null 2>&1 || MISSING="$MISSING $p"
done

if [ -n "$MISSING" ]; then
  echo "Installing:$MISSING"
  sudo apt-get install -y $MISSING
else
  echo "All dependencies already installed."
fi

SRC="$HOME/ros2_ws/src/fastbot-ros2-drivers-checkpoint-20/updated_firmware_config.txt"   # your updated version, kept in your repo
DEST="/boot/firmware/config.txt"
BACKUP="/boot/firmware/config.txt.bak"

# Do nothing if it's already up to date
if cmp -s "$SRC" "$DEST"; then
  echo "config.txt already up to date."
  exit 0
fi

# Keep the first backup only, so reruns don't overwrite the original
[ -f "$BACKUP" ] || sudo cp "$DEST" "$BACKUP"

sudo cp "$SRC" "$DEST"
sync
echo "config.txt updated. Reboot for changes to take effect."