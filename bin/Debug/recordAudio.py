import sounddevice as sd 
import numpy as np 
import wave
import datetime


wave_length = 30  # 録音する長さ（秒）
sample_rate = 16_000  # サンプリング周波数
data = np.zeros(1)
FILE_NAME = ""
try:
    while(True):
        now = datetime.datetime.now()
        strnow = now.strftime('%Y%m%d%H%M%S')
        FILE_NAME = './audio/'+ strnow +'.wav'  # 保存するファイル名
        print("recording in " +strnow+"...")
        # 録音開始（wave_length秒間録音。wait で録音し終わるまで待つ）
        data = sd.rec(int(wave_length * sample_rate), sample_rate, channels=1)
        sd.wait()
    
        # ノーマライズ。量子化ビット16bitで録音するので int16 の範囲で最大化する
        data = data / data.max() * np.iinfo(np.int16).max

        # float -> int
        data = data.astype(np.int16)

        # ファイル保存
        with wave.open(FILE_NAME, mode='wb') as wb:
            wb.setnchannels(1)  # モノラル
            wb.setsampwidth(2)  # 16bit=2byte
            wb.setframerate(sample_rate)
            wb.writeframes(data.tobytes())  # バイト列に変換


except KeyboardInterrupt:
    # ノーマライズ。量子化ビット16bitで録音するので int16 の範囲で最大化する
    data = data / data.max() * np.iinfo(np.int16).max

    # float -> int
    data = data.astype(np.int16)

    # ファイル保存
    with wave.open(FILE_NAME, mode='wb') as wb:
        wb.setnchannels(1)  # モノラル
        wb.setsampwidth(2)  # 16bit=2byte
        wb.setframerate(sample_rate)
        wb.writeframes(data.tobytes())  # バイト列に変換