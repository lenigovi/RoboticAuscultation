#!/usr/bin/env python
# coding: utf-8

# In[ ]:


import pyaudio
import numpy as np
import scipy.signal
import matplotlib.pyplot as plt

# Constants
FORMAT = pyaudio.paInt16
CHANNELS = 1
RATE = 44100
CHUNK = 1024

p = pyaudio.PyAudio()
stream = p.open(format=FORMAT,
                channels=CHANNELS,
                rate=RATE,
                input=True,
                frames_per_buffer=CHUNK)

print("Recording")
duration = 100
frames = []
for i in range(0, int(RATE / CHUNK * duration)):
    data = stream.read(CHUNK)
    frames.append(data)

print("Recording finished")
stream.stop_stream()
stream.close()
p.terminate()

audio_data = np.frombuffer(b''.join(frames), dtype=np.int16)
plt.figure(figsize=(10, 4))
plt.plot(np.linspace(0, duration, len(audio_data)), audio_data)
plt.title('Recorded Audio Signal')
plt.xlabel('Time (s)')
plt.ylabel('Amplitude')
plt.show()


filtered_audio = scipy.signal.sosfreqz(audio_data, fs=RATE, cutoff=1000, order=5)


plt.figure(figsize=(10, 4))
plt.plot(np.linspace(0, duration, len(filtered_audio)), filtered_audio)
plt.title('Filtered Audio Signal')
plt.xlabel('Time (s)')
plt.ylabel('Amplitude')
plt.show()

