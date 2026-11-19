"""Transcribe local Chinese audio with Whisper."""
import argparse
import json
import math
from pathlib import Path

SAMPLE_RATE = 16000
MAX_AUDIO_SECONDS = 12
COMMAND_WORDS = '厨房 客厅 卧室 健身房 健身区 杂物间 杯子 水杯 瓶子 椅子 清扫 打扫 扫地'
COMMAND_PROMPT = ('请用简体中文记录家庭机器人指令。'
                  '常用词包括厨房、客厅、卧室、健身区、杂物间、寻找、统计、杯子、水杯、瓶子、椅子、清扫。')


def read_recording(path):
    """Read browser Opus or WAV and bound decoded duration before inference."""
    import av
    import numpy as np

    chunks = []
    samples = 0
    resampler = av.AudioResampler(format='flt', layout='mono', rate=SAMPLE_RATE)

    def append(frames):
        nonlocal samples
        for frame in frames:
            samples += frame.samples
            if samples > MAX_AUDIO_SECONDS * SAMPLE_RATE:
                raise ValueError('录音过长，请在 8 秒内说完一条指令。')
            chunks.append(frame.to_ndarray().reshape(-1))

    with av.open(str(path)) as recording:
        if not recording.streams.audio:
            raise ValueError('录音中没有音频轨道。')
        for frame in recording.decode(audio=0):
            append(resampler.resample(frame))
        append(resampler.resample(None))
    if not chunks:
        raise ValueError('没有收到录音。')
    return np.concatenate(chunks)


def transcribe(model, samples, context='navigation'):
    prompt, words = COMMAND_PROMPT,COMMAND_WORDS
    if context=='mapping':
        prompt = '请用简体中文记录机器人运动指令。'
        words = '前进 后退 左转 右转 加速 减速 停止 绕行'
    segments, _ = model.transcribe(samples, language='zh', vad_filter=True,
                                   condition_on_previous_text=False,
                                   initial_prompt=prompt, hotwords=words)
    return ''.join(segment.text for segment in segments).strip()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--model', default='small', help='Model name or local model path')
    parser.add_argument('--seconds', type=float, default=4.0)
    parser.add_argument('--audio-file', type=Path, help='Convert a recording to text without issuing a command')
    parser.add_argument('--output', type=Path)
    parser.add_argument('--context',choices=('mapping','navigation'),default='navigation')
    args = parser.parse_args()
    if not math.isfinite(args.seconds) or not 0 < args.seconds <= MAX_AUDIO_SECONDS:
        parser.error('--seconds must be greater than zero and at most 12')
    if bool(args.audio_file) != bool(args.output):
        parser.error('--audio-file and --output must be used together')

    from faster_whisper import WhisperModel

    if args.audio_file:
        samples = read_recording(args.audio_file)
        model = WhisperModel(args.model, device='cpu', compute_type='int8', local_files_only=True)
        result = dict(text=transcribe(model, samples,args.context))
        args.output.write_text(json.dumps(result, ensure_ascii=False)+'\n', encoding='utf-8')
        return

    import sounddevice as sd
    import rclpy
    from std_msgs.msg import String

    model = WhisperModel(args.model, device='cpu', compute_type='int8')
    rclpy.init()
    node = rclpy.create_node('course_voice')
    publisher = node.create_publisher(String, '/search/command', 10)
    try:
        while True:
            input('回车开始录音，Ctrl+C 退出：')
            print('正在录音…', flush=True)
            samples = sd.rec(int(args.seconds * SAMPLE_RATE), samplerate=SAMPLE_RATE,
                             channels=1, dtype='float32', blocking=True)
            text = transcribe(model, samples[:, 0])
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
