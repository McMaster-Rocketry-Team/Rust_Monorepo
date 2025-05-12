import init, {
  CanBusMessageEnum,
  processCanBusFrame,
  CanBusExtendedId,
} from 'firmware-common-ffi'

export type CanBusMessage = {
  timestamp: number
  id: CanBusExtendedId
  crc: number
  message: CanBusMessageEnum
}

export class IcarusDevice {
  private constructor(
    private device: USBDevice,
    private isoEpIn: USBEndpoint,
    // private isoEpOut: USBEndpoint,
    private onMessage: (message: CanBusMessage) => void,
    private onDisconnect: () => void,
  ) {
    this.startHandleIsoIn()
  }

  private async startHandleIsoIn() {
    while (true) {
      let transferResult

      try {
        transferResult = await this.device.isochronousTransferIn(
          this.isoEpIn.endpointNumber,
          [13 * 4],
        )
      } catch (e) {
        console.warn(e)
        this.onDisconnect()
        break
      }

      for (const packet of transferResult.packets) {
        const packetData = packet.data

        if (!packetData) continue

        for (let i = 0; i < packetData.byteLength; i += 13) {
          const id = packetData.getUint32(i, false)
          const dataLength = packetData.getUint8(i + 4)
          const data = new Uint8Array(dataLength)

          for (let j = 0; j < dataLength; j++) {
            data[j] = packetData.getUint8(i + 5 + j)
          }
          const result = processCanBusFrame(BigInt(Date.now()), id, data)

          if ('Message' in result) {
            this.onMessage(result.Message)
          }
        }
      }
    }
  }

  static async init(
    device: USBDevice,
    onMessage: (message: CanBusMessage) => void,
    onDisconnect: () => void,
  ): Promise<IcarusDevice> {
    await init()
    const endpoints =
      device.configuration!.interfaces[1].alternates[0].endpoints

    if (endpoints.length != 2) {
      throw new Error('Expect 2 end points')
    }

    return new IcarusDevice(
      device,
      endpoints[0],
      // endpoints[1],
      onMessage,
      onDisconnect,
    )
  }
}

export async function requestIcarusDevice(
  onMessage: (message: CanBusMessage) => void,
  onDisconnect: () => void,
): Promise<IcarusDevice | null> {
  let device

  try {
    device = await navigator.usb.requestDevice({
      filters: [{ vendorId: 0x120a, productId: 0x0004 }],
    })
  } catch (e) {
    return null
  }

  await device.open()
  await device.selectConfiguration(1)
  await device.claimInterface(1)

  IcarusDevice.init(device, onMessage, onDisconnect)

  return null
}
