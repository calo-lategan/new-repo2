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
# Qt + GL env that makes rqt_image_view / image_view actually render
# inside the Hiwonder Docker container's forwarded X11 socket. With
# only QT_X11_NO_MITSHM the window chrome appears but the image area
# stays black - the other two unblock the image canvas.
export QT_X11_NO_MITSHM=1        # MIT-SHM unsupported on forwarded socket
export QT_QPA_PLATFORM=xcb       # auto-detect picks wayland/eglfs sometimes
export LIBGL_ALWAYS_SOFTWARE=1   # NVIDIA libGL in the container can't talk
                                 # to the host X server's GLX; sw raster
                                 # of a 640x480 image is cheap

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

# Quick probe that web_video_server is actually serving the topic. We
# prefer `ss` / `netstat` (port listening check) over `curl` because
# web_video_server's root path may return non-2xx and -f would falsely
# report "not responding". v4.1 starts web_video_server from our launch
# file directly - no factory service required.
probe_web_server() {
    local ip; ip=$(detect_ip)
    local listening=0
    if command -v ss >/dev/null 2>&1; then
        ss -tln 2>/dev/null | grep -q ":${WEB_PORT}\b" && listening=1
    elif command -v netstat >/dev/null 2>&1; then
        netstat -tln 2>/dev/null | grep -q ":${WEB_PORT}\b" && listening=1
    elif command -v curl >/dev/null 2>&1; then
        # Last-resort HTTP probe. Drop -f; any HTTP response means it's up.
        curl -sS -m 2 -o /dev/null "http://${ip}:${WEB_PORT}/" 2>/dev/null && listening=1
    fi
    if [ "$listening" = "1" ]; then
        log "web_video_server listening on :${WEB_PORT}"
    else
        err "warning: nothing listening on :${WEB_PORT} - browser will get 'connection refused'."
        err "          web_video_server is supposed to start from our launch file;"
        err "          check:  ros2 node list | grep web_video_server"
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
