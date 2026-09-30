#!/bin/sh
# Installs mavrelay as a systemd service on Debian/Ubuntu. Run from the relay directory:
#   sudo sh install-server.sh
# Creates /etc/mavrelay/mavrelay.ini with fresh keys on the first run and prints them.
set -eu

PORT="${PORT:-14650}"

command -v python3 >/dev/null || { echo "python3 is required (apt install python3)"; exit 1; }
id mavrelay >/dev/null 2>&1 || useradd --system --no-create-home --shell /usr/sbin/nologin mavrelay

install -d -m 755 /opt/mavrelay
install -m 644 mavrelay.py /opt/mavrelay/mavrelay.py
install -m 644 mavrelay.service /etc/systemd/system/mavrelay.service
install -d -m 750 -g mavrelay /etc/mavrelay

if [ ! -f /etc/mavrelay/mavrelay.ini ]; then
    VKEY=$(python3 /opt/mavrelay/mavrelay.py genkey)
    GKEY=$(python3 /opt/mavrelay/mavrelay.py genkey)
    cat > /etc/mavrelay/mavrelay.ini <<EOF
[server]
listen = 0.0.0.0:${PORT}
vehicle_key = ${VKEY}
gcs_key = ${GKEY}
#tcp_listen = 127.0.0.1:5760
#tcp_allow = 127.0.0.1/32, ::1/128
session_timeout = 120
log_level = info
# photos from the aircraft: in /var/lib/mavrelay/snapshots, for 7 days
#snapshot_days = 7
EOF
    chown root:mavrelay /etc/mavrelay/mavrelay.ini
    chmod 640 /etc/mavrelay/mavrelay.ini
    echo "Created /etc/mavrelay/mavrelay.ini"
    echo "  vehicle key (ESP32 firmware):  ${VKEY}"
    echo "  GCS key (laptop agent):        ${GKEY}"
fi

if command -v ufw >/dev/null && ufw status | grep -q "Status: active"; then
    ufw allow "${PORT}/udp"
fi

systemctl daemon-reload
systemctl enable --now mavrelay
systemctl restart mavrelay
sleep 1
systemctl --no-pager status mavrelay | head -n 5
echo "Logs: journalctl -u mavrelay -f"
