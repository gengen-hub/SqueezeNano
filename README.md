  # Squeeze Nano
  
### minimal LMS SlimProto (Logitech / Lyrion Media Server) wifi player for ESP32 boards with I2S DAC, multi room sync supported
Lighter than Squeezelite, requires ESP32-S2 with 2MB PSRAM at a minimum; S3 (dual core) is preferable
  
**Arduino IDE settings:**  
    Board            : ESP32 S2/S3  
    PSRAM            : QSPI PSRAM Enabled  
    CPU Frequency    : 240MHz  
    USB CDC On Boot  : Enabled (for Serial output)  
    Flash size       : 4MB  
    Partition scheme : Default 4MB with spiffs  
    Library          : "ESP8266Audio" by Earle F. Philhower, III (install via Library Manager)  
 
**Audio path:**  
    HTTP stream -> ring buffer (PSRAM) -> decoder -> PCM FIFO (PSRAM)
                -> dedicated I2S task (new driver on core 3.x, legacy on 2.x)  
    The I2S task starts output at the timestamp sent by LMS ("strm u"), so several players can start in the same millisecond. The elapsed time reported
    to LMS is derived from the number of frames actually handed to the I2S DMA.  
 
**I2S Pins configuration:**  
     `PIN_I2S_BCLK`
     `PIN_I2S_LRC`
     `PIN_I2S_DOUT`
  
**Limitations:**  
    - support mp3, flac and aac only, no raw PCM  
    - no gapless / crossfade  
    - HTTP chunked transfer and ICY metadata are not supported  
    - sample rate is limited by advertising `MaxSampleRate` to the server, so LMS resamples before sending (S2:48kHz S3:96kHz)

**Note:** 
ESP32S2 uses a single core to handle decoding, Wi-Fi, and I2S operations. While MP3 playback should be fine, FLAC files especially those with high bitrates or high compression—may experience audio dropouts. If this occurs, set `ENABLE_FLAC` to `0` and configure the LMS to transcode the audio to MP3.
Keep `MAX_SAMPLE_RATE` set to 48000. 96kHz is likely too demanding for the ESP32S2.  
ESP32S2 has less internal RAM (320KB) than the S3, so a 16KB stack for the loop task may place a significant burden on the system. If you encounter an "out of memory" error shortly after startup, try reducing the `SET_LOOP_TASK_STACK_SIZE` value to approximately 12KB.  
Because these processes run on the same core as the Wi-Fi stack, there may be longer delays in start times compared to the S3.
