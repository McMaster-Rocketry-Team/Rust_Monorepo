import { action, makeObservable, observable } from 'mobx'

export type OzysChannelState =
  | {
      connected: false
    }
  | {
      connected: true
      enabled: boolean
      name: string
      id: string
    }

export type OzysDeviceInfo = {
  name: string
  id: string
  model: string
  isRecording: boolean
  channels: OzysChannelState[]
}

// The device sends this data to the app every 10ms
// The readings list contains 20 samples during the last 10ms
// Sampling rate is 2kHz
export type OzysChannelRealtimeReadings = {
  // milliseconds since epoch
  // aligned to the start of the 10ms interval
  timestamp: number

  // Absolute reading values
  // Unit is 1, e.g. 0.01 = 1% of length change
  // 20 readings long, interval between readings is 0.5ms
  readings: Float32Array

  // Standard deviation of all the samples inside the 10ms interval
  // 20 values long, one value for each reading
  noises: Float32Array
}

// The device sends this data to the app every 100ms
// The data contains FFT during the last 100ms
export type OzysChannelRealtimeFft = {
  // milliseconds since epoch
  // aligned to the start of the 100ms interval
  timestamp: number

  // FFT for frequencies below 2kHz, with 10Hz resolution
  // 200 values long
  // e.g. fft_0_to_2k[0] is the power of the 0-10Hz band in the last 100ms
  //      fft_0_to_2k[1] is the power of the 10-20Hz band in the last 100ms
  fft0To2k: Float32Array

  // FFT for frequencies between 2kHz and 20kHz, with 50Hz resolution
  // 360 values long
  // e.g. fft_2k_to_20k[0] is the power of the 2k-2.05k band in the last 100ms
  fft2kTo20k: Float32Array
}

export abstract class OzysDevice {
  protected readingCallbacks: Array<
    (channelId: string, data: OzysChannelRealtimeReadings) => void
  > = []
  protected fftCallbacks: Array<
    (channelId: string, data: OzysChannelRealtimeFft) => void
  > = []

  constructor(public deviceInfo: OzysDeviceInfo) {
    makeObservable(this, {
      deviceInfo: observable,
      renameDevice: action,
      renameChannel: action,
      controlChannel: action,
      controlRecording: action,
    })
  }

  async renameDevice(name: string) {
    this.deviceInfo.name = name
  }

  async renameChannel(channelId: string, name: string) {
    for (const channel of this.deviceInfo.channels) {
      if (channel.connected && channel.id === channelId) {
        channel.name = name
        return
      }
    }
  }

  async controlChannel(channelId: string, enabled: boolean) {
    for (const channel of this.deviceInfo.channels) {
      if (channel.connected && channel.id === channelId) {
        channel.enabled = enabled
        return
      }
    }
  }

  async controlRecording(enabled: boolean) {
    this.deviceInfo.isRecording = enabled
  }

  onRealtimeReadings(
    callback: (channelId: string, data: OzysChannelRealtimeReadings) => void,
  ): () => void {
    this.readingCallbacks.push(callback)
    return () => {
      this.readingCallbacks = this.readingCallbacks.filter(
        (cb) => cb !== callback,
      )
    }
  }

  onRealtimeFft(
    callback: (channelId: string, data: OzysChannelRealtimeFft) => void,
  ): () => void {
    this.fftCallbacks.push(callback)
    return () => {
      this.fftCallbacks = this.fftCallbacks.filter((cb) => cb !== callback)
    }
  }

  abstract disconnect(): void
}
