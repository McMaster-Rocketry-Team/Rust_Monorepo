import menuIcon from '../../assets/menu.png'
import { useState } from 'react'
import { OzysDevice } from '../../device/OzysDevice'

interface ISensorData {
  connected: true
  enabled: boolean
  name: string
  id: string
}

export default function ChannelCard(props: {
  device: OzysDevice
  sensorData: ISensorData
}) {
  const name = props.sensorData.name
  const channelId = props.sensorData.id

  const [menuOpen, setMenuOpen] = useState(false)
  const [renameInput, setRenameInput] = useState(false)
  const [renameValue, setRenameValue] = useState(name)

  const toggleMenu = () => {
    setMenuOpen(!menuOpen)
  }

  const renameChannel = async () => {
    await props.device.renameChannel(channelId, renameValue)
    setRenameInput(false)
    setMenuOpen(false)
  }

  const toggleChannel = async () => {
    await props.device.controlChannel(channelId, !props.sensorData.enabled)
    console.log(props.sensorData.enabled)
  }

  const closePopup = () => {
    if (menuOpen) {
      setMenuOpen(false)
    }
  }

  return (
    <div
      className='w-full h-24 bg-[#F5F5F5] rounded-lg p-3'
      onClick={closePopup}
    >
      <div className='flex justify-between'>
        {renameInput ? (
          <form className='w-12 flex gap-4'>
            <input
              type='text'
              placeholder='Enter new name'
              className='px-2 py-1 w-36 bg-white focus:outline-gray-300 rounded-lg'
              defaultValue={name}
              onChange={(e) => setRenameValue(e.target.value)}
            />
            <button
              className='bg-white px-6 rounded-lg hover:bg-gray-100'
              type='submit'
              onClick={renameChannel}
            >
              Submit
            </button>
          </form>
        ) : (
          <h1 className='text-lg flex flex-row gap-3 items-center'>
            {name}{' '}
            <div
              className={`${
                props.sensorData.enabled ? 'bg-green-500' : 'bg-red-500'
              } w-2 h-2 rounded-full`}
            ></div>
          </h1>
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
                onClick={() => setRenameInput(true)}
              >
                Rename Channel
              </button>
              <button
                className='px-2 py-1 w-full text-center hover:bg-[#E2E2E2]'
                onClick={toggleChannel}
              >
                {props.sensorData.enabled
                  ? 'Disable Channel'
                  : 'Enable Channel'}
              </button>
            </div>
          ) : (
            ''
          )}
        </div>
      </div>
    </div>
  )
}
