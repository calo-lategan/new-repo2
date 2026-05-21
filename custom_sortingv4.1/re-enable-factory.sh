#!/usr/bin/env bash
# Re-enable Hiwonder's factory app (start_app_node.service). The v4.1
# launcher stops + disables this service every run, so this is the
# opt-in escape hatch when you want to go back to the factory behavior.
#
# After running this, reboot or run `sudo systemctl start
# start_app_node.service` to bring the factory bringup up.
#
# Note: running v4.1's launcher again after this will re-disable the
# service. To keep the factory app, do not run launch_v4.1.sh.

set -e
SERVICE="${SERVICE:-start_app_node.service}"
echo "[re-enable] unmasking ${SERVICE} (in case it was masked)"
sudo systemctl unmask "$SERVICE" 2>/dev/null || true
echo "[re-enable] enabling ${SERVICE} (will autostart on boot)"
sudo systemctl enable "$SERVICE"
echo "[re-enable] starting ${SERVICE} now"
sudo systemctl start "$SERVICE"
echo "[re-enable] state:"
systemctl is-enabled "$SERVICE" || true
systemctl is-active  "$SERVICE" || true
echo
echo "Done. v4.1's launcher will re-disable this on its next run."
