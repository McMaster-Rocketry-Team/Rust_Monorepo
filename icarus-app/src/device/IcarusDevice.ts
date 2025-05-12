export type CanBusFrame = {
  timestamp: BigInt
  id: number
  data: Uint8Array
}

export class IcarusDevice {
  private constructor(
    private device: USBDevice,
    private isoEpIn: USBEndpoint,
    private isoEpOut: USBEndpoint,
    private onFrame: (frame: CanBusFrame) => void,
    private onDisconnect: () => void,
  ) {
    this.startHandleIsoIn()
  }

  private async startHandleIsoIn() {
    while (true) {
      let result
      try {
        result = await this.device.isochronousTransferIn(
          this.isoEpIn.endpointNumber,
          [13 * 4],
        )
      } catch (e) {
        console.warn(e)
        this.onDisconnect()
        break
      }

      for (const packet of result.packets) {
        const packetData = packet.data

        if (!packetData) continue

        for (let i = 0; i < packetData.byteLength; i += 13) {
          const id = packetData.getUint32(i, false)
          const dataLength = packetData.getUint8(i + 4)
          const data = new Uint8Array(dataLength)

          for (let j = 0; j < dataLength; j++) {
            data[i] = packetData.getUint8(i + 5 + j)
          }
          this.onFrame({
            timestamp: BigInt(Date.now()),
            id,
            data,
          })
        }
      }
    }
  }

  static async init(
    device: USBDevice,
    onFrame: (frame: CanBusFrame) => void,
    onDisconnect: () => void,
  ): Promise<IcarusDevice> {
    console.log(device.configuration)
    const endpoints =
      device.configuration!.interfaces[1].alternates[0].endpoints

    if (endpoints.length != 2) {
      console.log(endpoints)
      throw new Error('Expect 2 end points')
    }

    return new IcarusDevice(
      device,
      endpoints[0],
      endpoints[1],
      onFrame,
      onDisconnect,
    )
  }
}

export async function requestIcarusDevice(): Promise<IcarusDevice | null> {
  const device = await navigator.usb.requestDevice({
    filters: [{ vendorId: 0x120a, productId: 0x0004 }],
  })

  await device.open()
  await device.selectConfiguration(1)
  await device.claimInterface(1)

  IcarusDevice.init(
    device,
    (frame) => {
      console.log(frame)
    },
    () => {
      console.log('ICARUS disconnected')
    },
  )

  return null
}
