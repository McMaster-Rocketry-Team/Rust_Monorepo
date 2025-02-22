import { observer } from 'mobx-react-lite'
import { OzysDevice } from '../../device/OzysDevice'
import ChannelCard from './ChannelCard'
import menuIcon from '../../assets/menu.png'
import { useState, useEffect, useRef } from 'react'
import { useOzysDevicesManager } from '../../device/OzysDevicesManager'

export const DeviceCard = observer((props: { device: OzysDevice }) => {
  const deviceInfo = props.device.deviceInfo
  const devicesManager = useOzysDevicesManager()

  const [menuOpen, setMenuOpen] = useState(false)
  const [renameInput, setRenameInput] = useState(false)
  const [renameValue, setRenameValue] = useState(deviceInfo.name)

  const toggleMenu = () => {
    setMenuOpen(!menuOpen)
    console.log('Menu Open:', deviceInfo.id)
  }

  const renameDevice = async () => {
    props.device.renameDevice(renameValue)
    setRenameInput(false)
    setMenuOpen(false)
  }

  const closePopup = () => {
    if (menuOpen) {
      setMenuOpen(false)
    }
  }

  useEffect(() => {
    window.addEventListener('resize', () => {
      closePopup()
    })
  })

  return (
    <div className='flex flex-col w-full pb-8 gap-4 p-4' onClick={closePopup}>
      <div className='flex justify-between mr-2'>
        {renameInput ? (
          <form
            className='w-72 flex gap-4 mr-4'
            onSubmit={(e) => {
              e.preventDefault() // Prevent default form submission
              renameDevice()
            }}
          >
            <input
              type='text'
              placeholder='Enter new name'
              className='px-2 py-1 w-36 bg-gray-100 focus:outline-gray-400 rounded-lg'
              defaultValue={deviceInfo.name}
              onChange={(e) => {
                setRenameValue(e.target.value)
              }}
            />
            <button
              className='bg-gray-100 px-6 rounded-lg hover:bg-gray-200'
              type='submit'
            >
              Submit
            </button>
          </form>
        ) : (
          <h1 className='text-lg font-semibold'>{deviceInfo.name}</h1>
        )}

        <div className='h-6 w-6'>
          <button onClick={toggleMenu}>
            <img src={menuIcon} alt='' />
          </button>

          {menuOpen ? (
            <div
              className={`flex flex-col items-start bg-[#F7F7F7] z-50 w-[150px] drop-shadow-lg divide-y divide-gray-300 -translate-x-36`}
            >
              <button
                className='px-2 py-1 w-full text-center hover:bg-[#E2E2E2]'
                onClick={() => devicesManager.disconnectDevice(deviceInfo.id)}
              >
                Disconnect Device
              </button>
              <button
                className='px-2 py-1 w-full text-center hover:bg-[#E2E2E2]'
                onClick={() => setRenameInput(true)}
              >
                Rename Device
              </button>
            </div>
          ) : (
            ''
          )}
        </div>
      </div>

      {deviceInfo.channels
        .filter((channel) => channel.connected)
        .map((channel) => (
          <ChannelCard
            key={channel.id}
            sensorData={channel}
            device={props.device}
          />
        ))}
    </div>
  )
})
