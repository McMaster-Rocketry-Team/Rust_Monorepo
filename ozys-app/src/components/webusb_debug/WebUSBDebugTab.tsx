import { useState } from 'react'
import { requestUSBOzysDevice } from '../../device/usb/requestUSBOzysDevice'
import { OzysDevice } from '../../device/OzysDevice'

export const WebUSBDebugTab = () => {
  const [device, setDevice] = useState<OzysDevice | null>(null)

  return (
    <div>
      <h1>WebUSB Debug</h1>
      <p>
        {device
          ? `Device: ${device.deviceInfo.name} ${device.deviceInfo.model} ${device.deviceInfo.id}`
          : 'No device selected'}
      </p>
      <button
        type='button'
        className='border border-gray-400'
        onClick={async () => {
          const device = await requestUSBOzysDevice()

          console.log('Device:', device)
          setDevice(device)
        }}
      >
        List Devices
      </button>
      <button
        type='button'
        className='border border-gray-400'
        onClick={async () => {
          // const result = await device!.controlTransferIn(
          //   {
          //     requestType: 'vendor',
          //     recipient: 'device',
          //     request: 101,
          //     value: 201,
          //     index: 0,
          //   },
          //   5,
          // )
          // console.log('Control Transfer Result:', result)
          // alert('Device response: ' + new TextDecoder().decode(result.data))
        }}
      >
        Send Control Transfer
      </button>
      <button
        type='button'
        className='border border-gray-400'
        onClick={async () => {
          // const result = await device!.isochronousTransferIn(1, [64])
          // console.log('Isochronous Transfer Result:', result.packets[0]?.status)

          // const dataView = result.packets[0]?.data
          // const array = Array.from(
          //   { length: dataView?.byteLength ?? 0 },
          //   (_, i) => dataView?.getUint8(i),
          // )
          // console.log(array)
        }}
      >
        Receive Isochronous Transfer
      </button>
    </div>
  )
}
