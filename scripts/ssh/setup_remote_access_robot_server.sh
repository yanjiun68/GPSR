#!/usr/bin/env bash
set -u
set -o pipefail

TARGET_HOSTNAME="putrabot-nuc"
SUMMARY_NAME="REMOTE_ACCESS_SETUP.md"
TAILSCALE_UP_LOG="/tmp/tailscale-up.log"

log() {
  printf '\n==> %s\n' "$*"
}

warn() {
  printf '\nWARNING: %s\n' "$*" >&2
}

run_or_warn() {
  "$@" || warn "Command failed but setup will continue: $*"
}

detect_user() {
  if [ -n "${SUDO_USER:-}" ] && [ "${SUDO_USER:-}" != "root" ] && id "$SUDO_USER" >/dev/null 2>&1; then
    printf '%s\n' "$SUDO_USER"
    return
  fi

  if [ -n "${PKEXEC_UID:-}" ]; then
    local pk_user
    pk_user="$(getent passwd "$PKEXEC_UID" | cut -d: -f1 || true)"
    if [ -n "$pk_user" ] && [ "$pk_user" != "root" ]; then
      printf '%s\n' "$pk_user"
      return
    fi
  fi

  local uid1000
  uid1000="$(getent passwd 1000 | cut -d: -f1 || true)"
  if [ -n "$uid1000" ] && [ "$uid1000" != "root" ]; then
    printf '%s\n' "$uid1000"
    return
  fi

  local first_user
  first_user="$(awk -F: '$3 >= 1000 && $3 < 65534 && $1 != "nobody" { print $1; exit }' /etc/passwd)"
  if [ -n "$first_user" ]; then
    printf '%s\n' "$first_user"
    return
  fi

  printf 'root\n'
}

set_sshd_option() {
  local key="$1"
  local value="$2"
  local file="/etc/ssh/sshd_config"

  if grep -Eiq "^[[:space:]#]*${key}[[:space:]]+" "$file"; then
    sed -i -E "s|^[[:space:]#]*${key}[[:space:]]+.*|${key} ${value}|I" "$file"
  else
    printf '\n%s %s\n' "$key" "$value" >> "$file"
  fi
}

print_inspection() {
  log "System inspection"
  hostnamectl || true
  lsb_release -a || true
  whoami || true
  id || true
  hostname -I || true
  ip route || true
  systemctl status ssh --no-pager || true
  timedatectl || true
}

if [ "$(id -u)" -ne 0 ]; then
  echo "This setup script must run as root." >&2
  exit 1
fi

SETUP_USER="$(detect_user)"
SETUP_HOME="$(getent passwd "$SETUP_USER" | cut -d: -f6)"
if [ -z "$SETUP_HOME" ]; then
  SETUP_HOME="/home/$SETUP_USER"
fi

print_inspection

log "Installing required packages"
apt-get update
DEBIAN_FRONTEND=noninteractive apt-get install -y \
  openssh-server \
  curl \
  wget \
  git \
  ufw \
  tmux \
  htop \
  net-tools \
  avahi-daemon \
  chrony \
  ca-certificates \
  gnupg \
  lsb-release

log "Setting hostname to ${TARGET_HOSTNAME}"
if [ "$(hostnamectl --static 2>/dev/null || hostname)" != "$TARGET_HOSTNAME" ]; then
  hostnamectl set-hostname "$TARGET_HOSTNAME"
else
  echo "Hostname already set to $TARGET_HOSTNAME"
fi

log "Enabling mDNS via avahi-daemon"
systemctl enable --now avahi-daemon

log "Configuring OpenSSH safely"
systemctl enable ssh
set_sshd_option "PermitRootLogin" "no"
set_sshd_option "PubkeyAuthentication" "yes"
set_sshd_option "PasswordAuthentication" "yes"
sshd -t
systemctl restart ssh
systemctl is-active ssh

log "Installing Tailscale with the official Linux install script"
if ! command -v tailscale >/dev/null 2>&1; then
  curl -fsSL https://tailscale.com/install.sh | sh
else
  echo "Tailscale is already installed: $(tailscale version | head -n 1)"
fi

log "Enabling and starting tailscaled"
systemctl enable --now tailscaled

log "Bringing up Tailscale SSH"
: > "$TAILSCALE_UP_LOG"
TAILSCALE_LOGIN_REQUIRED="unknown"
if tailscale status >/dev/null 2>&1 && tailscale ip -4 >/dev/null 2>&1; then
  echo "Tailscale is already authenticated; applying hostname and SSH settings."
  if timeout 60 tailscale up --ssh --hostname "$TARGET_HOSTNAME" 2>&1 | tee "$TAILSCALE_UP_LOG"; then
    TAILSCALE_LOGIN_REQUIRED="no"
  else
    TAILSCALE_LOGIN_REQUIRED="maybe"
  fi
elif [ -n "${TAILSCALE_AUTHKEY:-}" ]; then
  echo "TAILSCALE_AUTHKEY is present; authenticating non-interactively."
  if timeout 120 tailscale up --authkey "$TAILSCALE_AUTHKEY" --ssh --hostname "$TARGET_HOSTNAME" 2>&1 | tee "$TAILSCALE_UP_LOG"; then
    TAILSCALE_LOGIN_REQUIRED="no"
  else
    TAILSCALE_LOGIN_REQUIRED="maybe"
  fi
else
  echo "No TAILSCALE_AUTHKEY found; starting browser-based Tailscale login."
  timeout 120 tailscale up --ssh --hostname "$TARGET_HOSTNAME" 2>&1 | tee "$TAILSCALE_UP_LOG"
  ts_up_status=$?
  if tailscale ip -4 >/dev/null 2>&1; then
    TAILSCALE_LOGIN_REQUIRED="no"
  else
    TAILSCALE_LOGIN_REQUIRED="yes"
    echo "Tailscale login is required. Open the login URL printed above, or rerun after setting TAILSCALE_AUTHKEY."
    if [ "$ts_up_status" -eq 124 ]; then
      echo "The Tailscale login command timed out while waiting for browser authentication."
    fi
  fi
fi

log "Configuring UFW safely"
ufw allow OpenSSH
if command -v tailscale >/dev/null 2>&1 && systemctl is-active --quiet tailscaled; then
  ufw allow in on tailscale0
fi
ufw --force enable
ufw status numbered

log "Disabling sleep, suspend, hibernate, and hybrid sleep"
systemctl mask sleep.target suspend.target hibernate.target hybrid-sleep.target

log "Attempting desktop idle and lock settings for ${SETUP_USER}"
if command -v gsettings >/dev/null 2>&1 && pgrep -u "$SETUP_USER" gnome-session >/dev/null 2>&1; then
  runuser -u "$SETUP_USER" -- gsettings set org.gnome.desktop.session idle-delay 0 || warn "Could not set GNOME idle-delay"
  runuser -u "$SETUP_USER" -- gsettings set org.gnome.settings-daemon.plugins.power sleep-inactive-ac-type 'nothing' || warn "Could not set GNOME AC sleep policy"
  runuser -u "$SETUP_USER" -- gsettings set org.gnome.settings-daemon.plugins.power sleep-inactive-battery-type 'nothing' || warn "Could not set GNOME battery sleep policy"
  runuser -u "$SETUP_USER" -- gsettings set org.gnome.desktop.screensaver lock-enabled false || warn "Could not set GNOME screensaver lock policy"
else
  warn "No graphical GNOME session detected for ${SETUP_USER}; skipped gsettings power/lock changes."
fi

log "Enabling chrony time sync"
systemctl enable --now chrony
chronyc tracking || warn "chronyc tracking is not available yet"

log "Collecting final state"
LAN_IPS="$(hostname -I 2>/dev/null | xargs || true)"
LAN_IP_FIRST="$(printf '%s\n' "$LAN_IPS" | awk '{ print $1 }')"
TAILSCALE_IP="$(tailscale ip -4 2>/dev/null | head -n 1 || true)"
SSH_STATUS="$(systemctl is-active ssh 2>/dev/null || true)"
TAILSCALE_STATUS="$(systemctl is-active tailscaled 2>/dev/null || true)"
UFW_STATUS="$(ufw status 2>/dev/null || true)"
SLEEP_MASKED="yes"
for target in sleep.target suspend.target hibernate.target hybrid-sleep.target; do
  if [ "$(systemctl is-enabled "$target" 2>/dev/null || true)" != "masked" ]; then
    SLEEP_MASKED="no"
  fi
done

if [ -z "$TAILSCALE_IP" ] && [ "$TAILSCALE_LOGIN_REQUIRED" != "yes" ]; then
  TAILSCALE_LOGIN_REQUIRED="maybe"
fi

log "Writing summary file"
SUMMARY_PATH="${SETUP_HOME}/${SUMMARY_NAME}"
cat > "$SUMMARY_PATH" <<EOF_SUMMARY
# Remote Access Setup

Setup time: $(date -Is)

Hostname: $(hostnamectl --static 2>/dev/null || hostname)
Detected user: ${SETUP_USER}
LAN IP addresses: ${LAN_IPS:-unknown}
Tailscale IP: ${TAILSCALE_IP:-not connected}
SSH status: ${SSH_STATUS:-unknown}

## UFW Status

\`\`\`
${UFW_STATUS}
\`\`\`

## Connect From Main PC

\`\`\`bash
ssh ${SETUP_USER}@${TARGET_HOSTNAME}.local
ssh ${SETUP_USER}@${LAN_IP_FIRST:-LAN_IP}
ssh ${SETUP_USER}@${TAILSCALE_IP:-TAILSCALE_IP}
\`\`\`

## Check Services

\`\`\`bash
systemctl status ssh --no-pager
systemctl status tailscaled --no-pager
tailscale status
sudo ufw status
\`\`\`

## Notes

- Reboot is recommended after hostname, sleep, and network service changes.
- SSH password login remains enabled. Do not disable it until SSH key login is confirmed working.
- Tailscale is intended for remote SSH outside LAN. Public router port forwarding is not required.
EOF_SUMMARY
chown "$SETUP_USER:$SETUP_USER" "$SUMMARY_PATH" || true
chmod 0644 "$SUMMARY_PATH"

log "Tailscale status"
tailscale status || true
tailscale ip -4 || true

log "Final checklist"
printf 'SSH active: %s\n' "${SSH_STATUS:-unknown}"
printf 'Tailscale active: %s\n' "${TAILSCALE_STATUS:-unknown}"
printf 'LAN IP: %s\n' "${LAN_IPS:-unknown}"
printf 'Tailscale IP: %s\n' "${TAILSCALE_IP:-not connected}"
printf 'Hostname: %s\n' "$(hostnamectl --static 2>/dev/null || hostname)"
printf 'Firewall status:\n%s\n' "$UFW_STATUS"
printf 'Sleep masked: %s\n' "$SLEEP_MASKED"
printf 'Tailscale browser login required: %s\n' "$TAILSCALE_LOGIN_REQUIRED"
printf 'Reboot recommended: yes\n'
printf 'Summary file: %s\n' "$SUMMARY_PATH"
