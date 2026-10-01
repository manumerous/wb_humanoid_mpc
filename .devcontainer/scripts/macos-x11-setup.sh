#!/usr/bin/env bash
#
# initializeCommand for .devcontainer/mac-xquartz/devcontainer.json.
#
# Docker Desktop on macOS runs containers inside a Linux VM, so there is no
# shared /tmp/.X11-unix socket to bind-mount like on native Linux. Instead we
# point the container at XQuartz's X server over TCP: enable TCP listening,
# authorize the host's LAN IP, and write that IP into an env file the
# container picks up as DISPLAY=<host-ip>:0.
set -euo pipefail

XQUARTZ_APP="/Applications/Utilities/XQuartz.app"
XHOST_BIN="/opt/X11/bin/xhost"
ENV_FILE="$(cd "$(dirname "${BASH_SOURCE[0]}")/../mac-xquartz" && pwd)/x11.env"

if [ ! -d "${XQUARTZ_APP}" ]; then
  echo "XQuartz is not installed. Install it with:" >&2
  echo "  brew install --cask xquartz" >&2
  echo "then log out/in (or reboot) once before retrying." >&2
  exit 1
fi

# nolisten_tcp defaults to true (TCP disabled); flip it so the container can
# reach the X server over the network. Restart XQuartz if this changed while
# it was already running, since XQuartz only reads this setting at launch.
# XQuartz >= 2.8 reads org.xquartz.X11; older releases used org.macosforge.xquartz.X11.
# The running processes are X11.bin/Xquartz, so match on the app bundle path.
xquartz_running() { pgrep -f "XQuartz.app" >/dev/null 2>&1; }

# enable_iglx makes XQuartz advertise GLX visuals to remote clients; Mesa needs
# them even when rendering in software (LIBGL_ALWAYS_SOFTWARE in the container).
CURRENT_NOLISTEN="$(defaults read org.xquartz.X11 nolisten_tcp 2>/dev/null || echo "1")"
CURRENT_IGLX="$(defaults read org.xquartz.X11 enable_iglx 2>/dev/null || echo "0")"
if [ "${CURRENT_NOLISTEN}" != "0" ] || [ "${CURRENT_IGLX}" != "1" ]; then
  for DOMAIN in org.xquartz.X11 org.macosforge.xquartz.X11; do
    defaults write "${DOMAIN}" nolisten_tcp -bool false
    defaults write "${DOMAIN}" enable_iglx -bool true
  done
  if xquartz_running; then
    osascript -e 'quit app "XQuartz"' >/dev/null 2>&1 || true
    for _ in $(seq 1 20); do
      xquartz_running || break
      sleep 0.5
    done
  fi
fi

if ! xquartz_running; then
  open -a XQuartz
fi

# Wait for the X server to come up (xhost needs a live display to talk to).
for _ in $(seq 1 20); do
  if DISPLAY=:0 "${XHOST_BIN}" >/dev/null 2>&1; then
    break
  fi
  sleep 0.5
done

HOST_IP=""
for IFACE in en0 en1 en2; do
  IP="$(ipconfig getifaddr "${IFACE}" 2>/dev/null || true)"
  if [ -n "${IP}" ]; then
    HOST_IP="${IP}"
    break
  fi
done

if [ -z "${HOST_IP}" ]; then
  echo "Could not determine the host's LAN IP (checked en0, en1, en2)." >&2
  echo "Connect to Wi-Fi/Ethernet and retry, or edit ${ENV_FILE} by hand." >&2
  exit 1
fi

# Local dev convenience: allow any client to connect to this display rather
# than chasing the source IP Docker's VM presents to the X server (it is not
# always the LAN IP above). Fine for a local, trusted dev machine.
DISPLAY=:0 "${XHOST_BIN}" + >/dev/null

cat > "${ENV_FILE}" <<EOF
DISPLAY=${HOST_IP}:0
EOF

# The workspace root is a named volume (so build/install/log stay off the slow
# macOS bind mount). Docker creates it, and the src/ mountpoint for the nested
# bind mount, as root, which leaves colcon unable to write build/, install/
# and log/. Hand both directories to the container user before it starts.
WS_UID="${USER_ID:-$(id -u)}"
WS_GID="${GROUP_ID:-$(id -g)}"
docker run --rm -v wb_humanoid_mpc_ws:/ws busybox \
  sh -c "mkdir -p /ws/src && chown ${WS_UID}:${WS_GID} /ws /ws/src"

echo "XQuartz ready. Container DISPLAY will be ${HOST_IP}:0."
