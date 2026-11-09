"""Optional press-to-record Chinese speech input via a local Whisper model."""
import argparse

import rclpy
from std_msgs.msg import String


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--model', default='small', help='Model name or local model path')
    parser.add_argument('--seconds', type=float, default=4.0)
    args = parser.parse_args()
    if args.seconds <= 0:
        parser.error('--seconds must be positive')

    import sounddevice as sd
    from faster_whisper import WhisperModel

    model = WhisperModel(args.model, device='cpu', compute_type='int8')
    rclpy.init()
    node = rclpy.create_node('course_voice')
    publisher = node.create_publisher(String, '/search/command', 10)
    try:
        while True:
            input('回车开始录音，Ctrl+C 退出：')
            print('正在录音…', flush=True)
            samples = sd.rec(int(args.seconds * 16000), samplerate=16000,
                             channels=1, dtype='float32', blocking=True)
            segments, _ = model.transcribe(samples[:, 0], language='zh')
            text = ''.join(segment.text for segment in segments).strip()
            if text:
                print(text)
                publisher.publish(String(data=text))
                rclpy.spin_once(node, timeout_sec=0.1)
    except (KeyboardInterrupt, EOFError):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
