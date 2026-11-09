"""Set only Gazebo's 3-D camera, after its GUI transport service is ready."""
import subprocess
import time


def main():
    request = ('pose: {position: {x: 0, y: -10.8, z: 15.0}, orientation: '
               '{x: -0.316228, y: 0.316228, z: 0.632456, w: 0.632456}}')
    deadline = time.monotonic() + 30
    while time.monotonic() < deadline:
        try:
            result = subprocess.run(
                ['ign', 'service', '-s', '/gui/move_to/pose', '--reqtype',
                 'ignition.msgs.GUICamera', '--reptype', 'ignition.msgs.Boolean',
                 '--timeout', '1000', '--req', request],
                capture_output=True, text=True, timeout=3)
            if 'data: true' in result.stdout:
                print('AWS house overview camera ready', flush=True)
                return
        except subprocess.TimeoutExpired:
            pass
        time.sleep(.5)
    print('Gazebo overview camera unavailable; use mouse to adjust the view.', flush=True)


if __name__ == '__main__':
    main()
