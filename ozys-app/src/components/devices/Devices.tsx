import { observer } from 'mobx-react-lite'
import { useOzysDevicesManager } from '../../device/OzysDevicesManager'
import { MockOzysDevice } from '../../device/MockOzysDevice'
import { DeviceCard } from './DeviceCard'
import { MicOzysDevice } from '../../device/MicOzysDevice'

export const Devices = observer(() => {
  const devicesManager = useOzysDevicesManager()

  return (
    <div className='w-full h-full relative overflow-x-hidden pt-4 pb-24'>
      {devicesManager.devices.map((device) => (
        <DeviceCard key={device.deviceInfo.id} device={device} />
      ))}{' '}
      <div className='fixed bottom-0 left-0 py-4 ml-12 w-52 bg-white'>
        <button
          className='border-2 px-4 w-48 rounded-lg hover:bg-[#E2E2E2]'
          onClick={() => {
            devicesManager.addDevice(new MockOzysDevice())
          }}
        >
          Add Mock Device
        </button>
        <br />
        <button
          className='border-2 px-4 w-48 rounded-lg hover:bg-[#E2E2E2]'
          onClick={() => {
            devicesManager.addDevice(new MicOzysDevice())
          }}
        >
          Add Mic Device
        </button>
        <br />
        <button className='border-2 px-4 w-48 rounded-lg hover:bg-[#E2E2E2]'>
          Add USB Device
        </button>
      </div>
    </div>
  )
})
