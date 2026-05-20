#!/usr/bin/env bash
# v4.1 image-view chain: rqt_image_view -> image_view -> browser.
#
# Auto-opens a desktop image viewer for the annotated camera feed.
# Whichever viewer runs, when it exits (closed by user or crashed)
# the browser fallback is launched at the right Jetson IP. If neither
# native viewer is available we go straight to the browser.
#
# Usage:
#   image_view_chain.sh [topic]              default: /custom_sortingv4_1/image_result
#
# Env:
#   IMAGE_VIEW_BROWSER=firefox|chromium|xdg-open  override picked browser
#   IMAGE_VIEW_BROWSER_DISABLE=1                  skip browser fallback
#   IMAGE_VIEW_WEB_SERVER_PORT=8080                web_video_server port
#   IMAGE_VIEW_THROTTLE_MS=100                    web_video_server &th= value

set +u  # ROS setup files reference unset vars
# Qt MIT-SHM is broken inside the Hiwonder Docker container - both
# rqt_image_view and image_view render blank windows without this.
export QT_X11_NO_MITSHM=1

TOPIC="${1:-/custom_sortingv4_1/image_result}"
WEB_PORT="${IMAGE_VIEW_WEB_SERVER_PORT:-8080}"
THROTTLE_MS="${IMAGE_VIEW_THROTTLE_MS:-100}"

log() { printf "\033[1;36m[image-view]\033[0m %s\n" "$*"; }
err() { printf "\033[1;31m[image-view]\033[0m %s\n" "$*" >&2; }

detect_ip() {
    local ip
    # hostname -I prints all non-loopback IPs space-separated; first wins
    ip=$(hostname -I 2>/dev/null | awk '{print $1}')
    if [ -z "$ip" ]; then
        # Fallback: parse `ip route` for the source IP of the default route
        ip=$(ip route get 1.1.1.1 2>/dev/null | awk '/src/ {for(i=1;i<=NF;i++) if($i=="src"){print $(i+1); exit}}')
    fi
    [ -z "$ip" ] && ip="localhost"
    echo "$ip"
}

open_browser() {
    if [ "${IMAGE_VIEW_BROWSER_DISABLE:-0}" = "1" ]; then
        log "browser fallback disabled (IMAGE_VIEW_BROWSER_DISABLE=1)"
        return
    fi
    local ip url cmd
    ip=$(detect_ip)
    # &th=<ms> throttles web_video_server's polling. Without it some
    # browsers + web_video_server 3.x render a blank/stalled stream.
    url="http://${ip}:${WEB_PORT}/stream?topic=${TOPIC}&th=${THROTTLE_MS}"
    log "browser fallback URL: $url"
    # Honor explicit override
    if [ -n "${IMAGE_VIEW_BROWSER:-}" ] && command -v "$IMAGE_VIEW_BROWSER" >/dev/null 2>&1; then
        cmd="$IMAGE_VIEW_BROWSER"
    elif command -v xdg-open    >/dev/null 2>&1; then cmd="xdg-open"
    elif command -v sensible-browser >/dev/null 2>&1; then cmd="sensible-browser"
    elif command -v firefox     >/dev/null 2>&1; then cmd="firefox"
    elif command -v chromium    >/dev/null 2>&1; then cmd="chromium"
    elif command -v chromium-browser >/dev/null 2>&1; then cmd="chromium-browser"
    elif command -v google-chrome >/dev/null 2>&1; then cmd="google-chrome"
    else
        err "no browser binary found - open this URL manually:"
        err "  $url"
        return
    fi
    log "opening browser ($cmd)"
    # Fork it so we don't block the chain shell. Send stdio to /dev/null
    # so it doesn't pollute our [image-view] log.
    setsid "$cmd" "$url" </dev/null >/dev/null 2>&1 &
}

# Quick probe that web_video_server is actually serving the topic. We don't
# block on this - the browser will surface its own error if it isn't.
probe_web_server() {
    local ip; ip=$(detect_ip)
    if command -v curl >/dev/null 2>&1; then
        if curl -fsS -m 2 "http://${ip}:${WEB_PORT}/" >/dev/null 2>&1; then
            log "web_video_server reachable on ${ip}:${WEB_PORT}"
        else
            err "warning: web_video_server not responding on ${ip}:${WEB_PORT} - "
            err "          browser fallback may show 'connection refused'."
            err "          check:  sudo systemctl status start_app_node.service"
        fi
    fi
}

run_rqt() {
    log "launching rqt_image_view $TOPIC"
    rqt_image_view "$TOPIC"
    local rc=$?
    log "rqt_image_view exited (rc=$rc) - opening browser fallback"
    probe_web_server
    open_browser
}

run_image_view() {
    log "rqt_image_view not available - launching ros2 run image_view image_view"
    ros2 run image_view image_view --ros-args -r "image:=$TOPIC"
    local rc=$?
    log "image_view exited (rc=$rc) - opening browser fallback"
    probe_web_server
    open_browser
}

run_no_viewer() {
    log "no native viewer (rqt_image_view / image_view) found"
    log "going straight to browser"
    probe_web_server
    open_browser
}

if command -v rqt_image_view >/dev/null 2>&1; then
    run_rqt
elif command -v ros2 >/dev/null 2>&1; then
    run_image_view
else
    run_no_viewer
fi
