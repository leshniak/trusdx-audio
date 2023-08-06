#!/usr/bin/env python3
# de SQ3SWF 2023
# MacOS fixes by SP3ML

import pyaudio
import serial
import threading
import time

## change these (if you need to)
audio_tx_rate = 11525
audio_rx_rate = 7820
##

rcvd_buf = []  # buffer for received audio
underruns = [0]  # underrun counter
state = {'running': True, 'tx_enabled': False}

def find_audio_device(name, occurance = 0):
    result = [i for i in range(pyaudio.PyAudio().get_device_count()) if name in (pyaudio.PyAudio().get_device_info_by_index(i)['name'])]
    return result[occurance] if len(result) else -1  # return n-th matching device to name, -1 for no match

def receive_serial_audio(serport):
    while state['running']:
        data = serport.read(48)
        rcvd_buf.append(data)

def play_receive_audio(pastream):
    while state['running']:
        if len(rcvd_buf) < 2:
            print(f"UNDERRUN #{underruns[0]} - refilling")
            underruns[0] += 1
            while len(rcvd_buf) < 10:
                time.sleep(0.001)
        if not state['tx_enabled']:
            samples = [0] * (len(rcvd_buf[0]) * 2)
            samples[::2] = [x ^ 0x80 for x in rcvd_buf[0]]  # put audio in left channel
            pastream.write(bytes(samples))
        rcvd_buf.remove(rcvd_buf[0])

def transmit_audio_via_serial(pastream, serport):
    while state['running']:
        to_read = pastream.get_read_available()
        if to_read == 0:
            continue
        samples = pastream.read(to_read, exception_on_overflow = False)
        samples = samples[1::2]  # read audio from right channe;
        samples = [x ^ 0x80 for x in samples]
        min_value = min(samples)
        max_value = max(samples)
        samples = bytes(samples).replace(b"\x3b", b"\x3a")
        if min_value != 128 and max_value != 128:  # if does not contain silence
            if not state['tx_enabled']:
                state['tx_enabled'] = True
                print(">>> TX MODE")
                serport.write(b";TX0;")
            serport.write(samples)
        elif state['tx_enabled']:  # if no signal is detected == silence
            serport.flush()
            time.sleep(0.1)
            state['tx_enabled'] = False
            serport.write(b";RX;")
            serport.flush()
            print(">>> RX MODE")

def main():
    try:
        in_stream = pyaudio.PyAudio().open(
            format = pyaudio.paInt8,
            channels = 2,
            rate = audio_tx_rate,
            input = True,
            input_device_index = find_audio_device("BlackHole 2ch"),
            frames_per_buffer = 0
        )
        out_stream = pyaudio.PyAudio().open(
            format = pyaudio.paInt8,
            channels = 2,
            rate = audio_rx_rate,
            output = True,
            output_device_index = find_audio_device("BlackHole 2ch"),
            frames_per_buffer = 0
        )
        ser = serial.Serial("/dev/tty.wchusbserial110", 115200, write_timeout = 0)

        time.sleep(3)  # wait for device to start after opening serial port
        ser.write(b";MD2;UA2;RX;")  # enable audio streaming
        ser.flush()
        time.sleep(0.01)
        ser.reset_input_buffer()

        print(">>> RX MODE")

        threading.Thread(target=receive_serial_audio, args=(ser,)).start()
        threading.Thread(target=play_receive_audio, args=(out_stream,)).start()
        threading.Thread(target=transmit_audio_via_serial, args=(in_stream,ser)).start()

        # display some stats every 10 seconds
        ts = time.time()
        while True:
            print(f"{int(time.time()-ts)} BUF: {len(rcvd_buf)}")
            time.sleep(10)
    except:
        state['running'] = False
        time.sleep(0.1)
        ser.write(b";UA0;")
        ser.flush()
        ser.cancel_read()
        ser.close()
        pyaudio.PyAudio().terminate()

if __name__ == '__main__':
    main()
