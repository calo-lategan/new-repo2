#!/usr/bin/env python3
# encoding: utf-8
# @Author: Aiden
# @Date: 2024/11/27
import os
import time
from speech import awake

port = '/dev/ttyUSB0'
kws = awake.WonderEchoPro(port)
# kws = awake.CircleMic(port, 'hello hi wonder', 'mic6_circle', True)

try:  # 如果有风扇，检测前推荐关掉减少干扰
    os.system('pinctrl FAN_PWM op dh')
except:
    pass

kws.start() # 开始检测
print('start...')
while True:
    try:
        if kws.wakeup(): # 检测到唤醒
            print('hello hiwonder')
        time.sleep(0.02)
    except KeyboardInterrupt:
        kws.exit() # 关闭处理
        try:
            os.system('pinctrl FAN_PWM a0')
        except:
            pass
        break

