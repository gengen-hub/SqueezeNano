  # Squeeze Nano
  
  ### minimal SlimProto (Logitech Media Server) player for ESP32 boards with I2S DAC, multi room audio supported
  ESP32 boards : Requires ESP32-S2 with 2MB PSRAM at a minimum; S3 (dual core) is preferable
  
  Arduino IDE settings:
    Board            : ESP32 S2/S3
    PSRAM            : QSPI PSRAM Enabled
    CPU Frequency    : 240MHz
    USB CDC On Boot  : Enabled (for Serial output)
    Flash size       : 4MB
    Partition scheme : Default 4MB with spiffs
    Library          : "ESP8266Audio" by Earle F. Philhower, III (install via Library Manager)
 
  Audio path:
    HTTP stream -> ring buffer (PSRAM) -> decoder -> PCM FIFO (PSRAM)
                -> dedicated I2S task (new driver on core 3.x, legacy on 2.x)
    The I2S task starts output at the timestamp sent by LMS ("strm u"), so
    several players can start in the same millisecond. The elapsed time reported
    to LMS is derived from the number of frames actually handed to the I2S DMA.
 
  I2S Pins configuration:
     PIN_I2S_BCLK
     PIN_I2S_LRC
     PIN_I2S_DOUT
  
  Limitations:
    - support mp3, flac and aac only, no raw PCM
    - no gapless / crossfade
    - HTTP chunked transfer and ICY metadata are not supported
    - sample rate is limited by advertising MaxSampleRate to the server,
      so LMS resamples before sending (S2:48000 S3:96000)
 
