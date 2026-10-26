"""Capture the current PyBullet GUI window with PIL."""

import os
import time

import win32con
import win32gui
from PIL import ImageGrab


def capture_window(path, title_substr="Bullet Physics ExampleBrowser"):
    os.makedirs(os.path.dirname(path), exist_ok=True)

    hwnds = []

    def callback(hwnd, _):
        if not win32gui.IsWindowVisible(hwnd):
            return
        title = win32gui.GetWindowText(hwnd)
        if title_substr.lower() in title.lower():
            hwnds.append(hwnd)

    win32gui.EnumWindows(callback, None)
    if not hwnds:
        print("screenshot: PyBullet window not found")
        return False

    hwnd = hwnds[0]
    try:
        win32gui.ShowWindow(hwnd, win32con.SW_MAXIMIZE)
        win32gui.SetWindowPos(
            hwnd, win32con.HWND_TOPMOST, 0, 0, 0, 0,
            win32con.SWP_NOMOVE | win32con.SWP_NOSIZE |
            win32con.SWP_SHOWWINDOW,
        )
        win32gui.BringWindowToTop(hwnd)
        win32gui.SetForegroundWindow(hwnd)
    except Exception:
        pass
    time.sleep(1.2)

    # 只截客户区，不截标题栏
    left, top, right, bottom = win32gui.GetClientRect(hwnd)
    sx, sy = win32gui.ClientToScreen(hwnd, (left, top))
    bbox = (sx, sy, sx + right - left, sy + bottom - top)
    ImageGrab.grab(bbox=bbox).save(path)
    print("screenshot:", path)
    return True
