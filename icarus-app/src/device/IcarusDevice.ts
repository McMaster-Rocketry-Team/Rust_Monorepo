import {
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
    private epIn: USBEndpoint,
    // private isoEpOut: USBEndpoint,
    private onMessage: (message: CanBusMessage) => void,
    private onDisconnect: () => void,
  ) {
    this.startHandleIsoIn()
  }

  private async startHandleIsoIn() {
    while (true) {
      let transferResult: USBInTransferResult

      try {
        transferResult = await this.device.transferIn(
          this.epIn.endpointNumber,
          64,
        )
      } catch (e) {
        console.warn(e)
        this.onDisconnect()
        break
      }

      const packetData = transferResult.data

      if (!packetData) continue

      const canFrames = packetData.getUint8(0)

      for (let i = 0; i < canFrames; i += 13) {
        const id = packetData.getUint32(1 + i, false)
        const dataLength = packetData.getUint8(1 + i + 4)
        const data = new Uint8Array(dataLength)

        for (let j = 0; j < dataLength; j++) {
          data[j] = packetData.getUint8(1 + i + 5 + j)
        }
        const result = processCanBusFrame(BigInt(Date.now()), id, data)

        if ('Message' in result) {
          this.onMessage(result.Message)
        }
      }
    }
  }

  static async init(
    device: USBDevice,
    onMessage: (message: CanBusMessage) => void,
    onDisconnect: () => void,
  ): Promise<IcarusDevice> {
    console.log(device.configuration)
    const endpoints =
      device.configuration!.interfaces[1].alternates[0].endpoints

    // if (endpoints.length != 2) {
    //   throw new Error('Expect 2 end points')
    // }

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
