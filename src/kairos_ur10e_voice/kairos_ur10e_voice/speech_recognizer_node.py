"""Speech -> text node.

mode:=mic       offline recognition with Vosk from the default microphone
mode:=keyboard  type the commands in the terminal (handy without a microphone)

Publishes std_msgs/String on /speech/text. Listens to /speech/feedback so it can
mute itself while the robot is talking back (avoids hearing its own voice).
"""
import json
import queue
import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from kairos_ur10e_voice.command_parser import VOCAB


class SpeechRecognizer(Node):
    def __init__(self):
        super().__init__('speech_recognizer')
        self.declare_parameter('mode', 'mic')
        self.declare_parameter('model_path', '~/vosk-model-small-en-us-0.15')
        self.declare_parameter('sample_rate', 16000)
        self.declare_parameter('device', -1)             # -1 = default input device
        self.declare_parameter('wake_word', '')           # e.g. 'robot'
        self.declare_parameter('use_grammar', True)       # restrict Vosk to command vocabulary
        self.declare_parameter('mute_after_feedback', True)

        self.pub = self.create_publisher(String, 'speech/text', 10)
        self.mute_until = 0.0
        if self.get_parameter('mute_after_feedback').value:
            self.create_subscription(String, 'speech/feedback', self._on_feedback, 10)

        mode = self.get_parameter('mode').value
        target = self._run_mic if mode == 'mic' else self._run_keyboard
        threading.Thread(target=target, daemon=True).start()

    # -- helpers -----------------------------------------------------------
    def _on_feedback(self, msg: String):
        self.mute_until = time.time() + 0.4 * len(msg.data.split()) + 1.0

    def _emit(self, text: str):
        text = ' '.join(t for t in text.split() if t != '[unk]').strip()
        if not text:
            return
        wake = self.get_parameter('wake_word').value.strip().lower()
        if wake:
            toks = text.split()
            if wake not in toks:
                return
            toks.remove(wake)
            text = ' '.join(toks)
            if not text:
                return
        self.get_logger().info(f'heard: "{text}"')
        self.pub.publish(String(data=text))

    # -- keyboard ----------------------------------------------------------
    def _run_keyboard(self):
        self.get_logger().info('Keyboard mode: type a command and press Enter.')
        while rclpy.ok():
            try:
                line = input('command> ')
            except EOFError:
                return
            self._emit(line)

    # -- microphone (Vosk) -------------------------------------------------
    def _run_mic(self):
        try:
            import os

            import sounddevice as sd
            from vosk import KaldiRecognizer, Model, SetLogLevel
        except ImportError as e:
            self.get_logger().error(
                f'Missing dependency ({e}). Install with: pip3 install vosk sounddevice')
            return

        SetLogLevel(-1)
        path = os.path.expanduser(self.get_parameter('model_path').value)
        if not os.path.isdir(path):
            self.get_logger().error(
                f'Vosk model not found at {path}. Download vosk-model-small-en-us-0.15 from '
                'https://alphacephei.com/vosk/models and unzip it there, or pass '
                'vosk_model:=<path>.')
            return

        rate = int(self.get_parameter('sample_rate').value)
        model = Model(path)
        if self.get_parameter('use_grammar').value:
            rec = KaldiRecognizer(model, rate, json.dumps(VOCAB + ['[unk]']))
        else:
            rec = KaldiRecognizer(model, rate)

        audio_q: 'queue.Queue[bytes]' = queue.Queue()
        dev = int(self.get_parameter('device').value)

        def callback(indata, frames, t, status):
            audio_q.put(bytes(indata))

        self.get_logger().info('Listening... (say e.g. "move forward two meters")')
        with sd.RawInputStream(samplerate=rate, blocksize=4000, dtype='int16', channels=1,
                               device=None if dev < 0 else dev, callback=callback):
            while rclpy.ok():
                data = audio_q.get()
                if time.time() < self.mute_until:
                    continue
                if rec.AcceptWaveform(data):
                    self._emit(json.loads(rec.Result()).get('text', ''))


def main(args=None):
    rclpy.init(args=args)
    node = SpeechRecognizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
