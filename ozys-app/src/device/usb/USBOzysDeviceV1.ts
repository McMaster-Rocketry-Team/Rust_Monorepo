import { OzysChannelState, OzysDevice, OzysDeviceInfo } from '../OzysDevice'

export class USBOzysDeviceV1 extends OzysDevice {
  constructor(
    private device: USBDevice,
    deviceInfo: OzysDeviceInfo,
    private interruptEpNumber: number,
    private isoEpNumber: number,
  ) {
    super(deviceInfo)
    this.startHandleInterrupt()
  }

  private async startHandleInterrupt() {
    while (true) {
      const result = await this.device.transferIn(this.interruptEpNumber, 64)
      if (result.status !== 'ok') {
        throw new Error('Interrupt transfer failed')
      }
      const type = result.data!.getUint8(0)
      if (type === 0) {
        // Channel connected
        const channelI = result.data!.getUint8(1)
        console.log(`Channel ${channelI} connected`)
        const channelState = await USBOzysDeviceV1.getChannelState(
          this.device,
          channelI,
        )
        this.deviceInfo.channels[channelI] = channelState
      } else if (type === 1) {
        // Channel disconnected
        const channelI = result.data!.getUint8(1)
        console.log(`Channel ${channelI} disconnected`)
        this.deviceInfo.channels[channelI] = { connected: false }
      }
    }
  }

  async renameDevice(name: string): Promise<void> {
    await USBOzysDeviceV1.controlTransferOut(
      this.device,
      outCommands.renameDevice,
      USBOzysDeviceV1.encodeText(name),
    )
    super.renameDevice(name)
  }

  async renameChannel(channelId: string, name: string): Promise<void> {
    const channelI = this.deviceInfo.channels.findIndex((channel) => {
      if (!channel.connected) return false
      return channel.id === channelId
    })

    await USBOzysDeviceV1.controlTransferOut(
      this.device,
      {
        request: 0x44,
        value: channelI,
      },
      USBOzysDeviceV1.encodeText(name),
    )
    super.renameChannel(channelId, name)
  }

  async controlChannel(channelId: string, enabled: boolean): Promise<void> {
    const channelI = this.deviceInfo.channels.findIndex((channel) => {
      if (!channel.connected) return false
      return channel.id === channelId
    })

    await USBOzysDeviceV1.controlTransferOut(
      this.device,
      {
        request: 0x43,
        value: channelI,
      },
      new Uint8Array([enabled ? 1 : 0]),
    )
    super.controlChannel(channelId, enabled)
  }

  async controlRecording(enabled: boolean): Promise<void> {
    await USBOzysDeviceV1.controlTransferOut(
      this.device,
      outCommands.controlRecording,
      new Uint8Array([enabled ? 1 : 0]),
    )
    super.controlRecording(enabled)
  }

  disconnect(): void {
    this.device.close()
  }

  static async init(device: USBDevice): Promise<USBOzysDeviceV1> {
    console.log(device.configuration)
    console.log(device.configuration!.interfaces[1].alternates[0].endpoints)
    const interruptEpNumber =
      device.configuration!.interfaces[1].alternates[0].endpoints[0]
        .endpointNumber
    const isoEpNumber =
      device.configuration!.interfaces[1].alternates[0].endpoints[1]
        .endpointNumber

    const deviceNameResult = await this.controlTransferIn(
      device,
      inCommands.requestDeviceName,
    )
    const deviceName = this.decodeText(deviceNameResult)

    const deviceIdResult = await this.controlTransferIn(
      device,
      inCommands.requestDeviceId,
    )
    const deviceId = this.decodeText(deviceIdResult)

    const deviceModelResult = await this.controlTransferIn(
      device,
      inCommands.requestDeviceModel,
    )
    const deviceModel = this.decodeText(deviceModelResult)

    const deviceIsRecordingResult = await this.controlTransferIn(
      device,
      inCommands.requestDeviceIsRecording,
    )
    const deviceIsRecording = !!deviceIsRecordingResult.getUint8(0)

    const deviceChannelCountResult = await this.controlTransferIn(
      device,
      inCommands.requestDeviceChannelCount,
    )
    const deviceChannelCount = deviceChannelCountResult.getUint8(0)

    const channels: OzysChannelState[] = []
    for (let i = 0; i < deviceChannelCount; i++) {
      channels.push(await this.getChannelState(device, i))
    }

    return new USBOzysDeviceV1(
      device,
      {
        name: deviceName,
        id: deviceId,
        model: deviceModel,
        isRecording: deviceIsRecording,
        channels,
      },
      interruptEpNumber,
      isoEpNumber,
    )
  }

  static decodeText(data: DataView) {
    const text = new TextDecoder().decode(data)
    return text.split('\0')[0]
  }

  static encodeText(text: string) {
    if (text.length > 64) {
      throw new Error('Text is too long')
    }
    if (text.length < 64) {
      text = text.padEnd(64, '\0')
    }
    return new TextEncoder().encode(text)
  }

  static async getChannelState(
    device: USBDevice,
    channelI: number,
  ): Promise<OzysChannelState> {
    const channelConnectedResult = await this.controlTransferIn(device, {
      request: 0x42,
      value: channelI,
      length: 1,
    })
    const channelConnected = !!channelConnectedResult.getUint8(0)
    if (!channelConnected) {
      return { connected: false }
    }

    const channelEnabledResult = await this.controlTransferIn(device, {
      request: 0x43,
      value: channelI,
      length: 1,
    })
    const channelEnabled = !!channelEnabledResult.getUint8(0)

    const channelNameResult = await this.controlTransferIn(device, {
      request: 0x44,
      value: channelI,
      length: 64,
    })
    const channelName = this.decodeText(channelNameResult)

    const channelIdResult = await this.controlTransferIn(device, {
      request: 0x45,
      value: channelI,
      length: 64,
    })
    const channelId = this.decodeText(channelIdResult)

    return {
      connected: true,
      enabled: channelEnabled,
      name: channelName,
      id: channelId,
    }
  }

  static async controlTransferIn(
    device: USBDevice,
    {
      request,
      value,
      length,
    }: { request: number; value: number; length: number },
  ) {
    const result = await device.controlTransferIn(
      {
        requestType: 'vendor',
        recipient: 'device',
        index: 0,
        request,
        value,
      },
      length,
    )
    if (result.status! !== 'ok') {
      throw new Error('USB transfer failed')
    }
    return result.data!
  }

  static async controlTransferOut(
    device: USBDevice,
    { request, value }: { request: number; value: number },
    data: Uint8Array,
  ) {
    const result = await device.controlTransferOut(
      {
        requestType: 'vendor',
        recipient: 'device',
        index: 0,
        request,
        value,
      },
      data,
    )
    if (result.status! !== 'ok') {
      throw new Error('USB transfer failed')
    }
  }
}

const inCommands = {
  requestDeviceName: {
    request: 0x41,
    value: 0,
    length: 64,
  },
  requestDeviceId: {
    request: 0x41,
    value: 1,
    length: 64,
  },
  requestDeviceModel: {
    request: 0x41,
    value: 2,
    length: 64,
  },
  requestDeviceIsRecording: {
    request: 0x41,
    value: 3,
    length: 1,
  },
  requestDeviceChannelCount: {
    request: 0x41,
    value: 4,
    length: 1,
  },
} as const satisfies Record<
  string,
  { request: number; value: number; length: number }
>

const outCommands = {
  renameDevice: {
    request: 0x41,
    value: 0,
  },
  controlRecording: {
    request: 0x41,
    value: 3,
  },
} as const satisfies Record<string, { request: number; value: number }>
