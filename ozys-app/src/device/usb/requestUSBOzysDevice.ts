import { OzysDevice } from '../OzysDevice'
import { USBOzysDeviceV1 } from './USBOzysDeviceV1'

export async function requestUSBOzysDevice(): Promise<OzysDevice | null> {
  // TODO we need to check if the browser supports WebUSB
  // TODO we need to get a usb product id from https://pid.codes/
  // For testing, OZYS V2 is 0x1209:0x0002, OZYS V3 is 0x1209:0x0003
  const device = await navigator.usb.requestDevice({
    filters: [
      { vendorId: 0x1209, productId: 0x0002 },
      { vendorId: 0x1209, productId: 0x0003 },
    ],
  })

  await device.open()
  await device.selectConfiguration(1)
  await device.claimInterface(1)

  // Get protocol version
  const result = await device.controlTransferIn(
    {
      requestType: 'vendor',
      recipient: 'device',
      index: 0,
      request: 0x40,
      value: 0,
    },
    1,
  )
  if (result.status !== 'ok') {
    throw new Error('Failed to get OZYS protocol version')
  }
  const protocolVersion = result.data?.getUint8(0)
  if (protocolVersion === 1) {
    return USBOzysDeviceV1.init(device)
  }

  return null
}
