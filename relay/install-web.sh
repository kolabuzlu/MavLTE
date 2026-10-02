#!/bin/sh
# Puts MavLTE's web page (mavweb.py: the Aircraft card for a phone) on the relay server, with Caddy in front of
# it for HTTPS. Run from the relay directory, after install-server.sh, with the name the page is reached at:
#   sudo sh install-web.sh 203-0-113-10.sslip.io    (your server's address with dashes: no domain needed)
#   sudo sh install-web.sh uav.example.com           (a name of your own, pointed at this server first)
# Caddy takes TCP ports 80 and 443 and gets the certificate from Let's Encrypt by itself. The first run asks for
# the page's password (without a terminal, it makes one up and prints it); to change it later:
#   sudo python3 /opt/mavrelay/mavweb.py password
set -eu

NAME="${1:-}"
if [ -z "$NAME" ]; then
    IP=$(hostname -I 2>/dev/null | awk '{print $1}')
    echo "usage: sudo sh install-web.sh <the page's name>"
    echo "  without a domain of your own:  sudo sh install-web.sh $(echo "${IP:-203.0.113.10}" | tr . -).sslip.io"
    exit 1
fi
[ -f /etc/mavrelay/mavrelay.ini ] || { echo "Install the relay first: sudo sh install-server.sh"; exit 1; }

install -m 644 mavrelay.py aircraft_card.py mavweb.py mavlte.png /opt/mavrelay/
install -d -m 755 /opt/mavrelay/web
install -m 644 web/index.html web/app.css web/app.js web/manifest.webmanifest /opt/mavrelay/web/
install -m 644 mavweb.service /etc/systemd/system/mavweb.service

if ! grep -q '^password *= *pbkdf2_sha256' /etc/mavrelay/mavrelay.ini; then
    if [ -t 0 ]; then
        python3 /opt/mavrelay/mavweb.py password --config /etc/mavrelay/mavrelay.ini
    else
        python3 /opt/mavrelay/mavweb.py password --random --config /etc/mavrelay/mavrelay.ini
    fi
fi

if ! command -v caddy >/dev/null; then
    apt-get update -qq
    DEBIAN_FRONTEND=noninteractive apt-get install -y -qq caddy
fi
CADDYFILE=/etc/caddy/Caddyfile
if [ -f "$CADDYFILE" ] && ! grep -q '127.0.0.1:8090' "$CADDYFILE" && ! grep -q '/usr/share/caddy' "$CADDYFILE"; then
    echo "$CADDYFILE has sites of its own: add these lines to it, then run  systemctl reload caddy"
    printf '\n%s {\n    reverse_proxy 127.0.0.1:8090\n}\n\n' "$NAME"
else  # Caddy's own placeholder, or ours
    cat > "$CADDYFILE" <<EOF
# MavLTE's web page (install-web.sh): HTTPS for ${NAME}, then mavweb on 127.0.0.1:8090
${NAME} {
    reverse_proxy 127.0.0.1:8090
}
EOF
fi

if command -v ufw >/dev/null && ufw status | grep -q "Status: active"; then
    ufw allow 80/tcp
    ufw allow 443/tcp
fi

systemctl daemon-reload
systemctl restart mavrelay  # the mavrelay.py just installed: the aircraft reconnects within a second
systemctl enable --now mavweb
systemctl restart mavweb
systemctl reload caddy 2>/dev/null || systemctl restart caddy
sleep 2
systemctl --no-pager status mavweb | head -n 5
echo "The page: https://${NAME}"
echo "Logs: journalctl -u mavweb -f   (HTTPS and its certificate: journalctl -u caddy)"
