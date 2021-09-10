Xvfb $DISPLAY -screen $SCREEN $SCREEN_SIZE &
sleep 5
icewm &
x11vnc -display $DISPLAY -nopw -listen localhost -xkb -ncache_cr -forever &
ln -s /root/noVNC/vnc.html /root/noVNC/index.html
/root/noVNC/utils/novnc_proxy #--vnc localhost:5900
