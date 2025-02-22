import { useTabAtom } from '../../workspace/useTabAtom'
import { useOzysDevicesManager } from '../../device/OzysDevicesManager'
import { useEffect, useRef, useState } from 'react'
import { useRaf } from 'rooks'
import { observer } from 'mobx-react-lite'
import { SelectedChannel, SpectrogramCanvas } from './SpectrogramCanvas'

export const Spectrogram = observer(() => {
  const devicesManager = useOzysDevicesManager()
  const [selectedChannel, setSelectedChannel] =
    useTabAtom<SelectedChannel | null>('selectedChannel', null)

  // CHANGE SPEED
  const [msPerPixel, setMsPerPixel] = useTabAtom('msPerPixel', 0)

  const canvasContainerRef = useRef<HTMLDivElement>(null)
  const canvasRef = useRef<SpectrogramCanvas | null>(null)
  const [isMenuOpen, setIsMenuOpen] = useTabAtom('isMenuOpen', false)

  useEffect(() => {
    if (canvasContainerRef.current) {
      console.log(msPerPixel)
      canvasRef.current = new SpectrogramCanvas(
        msPerPixel,
        canvasContainerRef.current,
        devicesManager,
      )
    }
    return () => {
      canvasRef.current?.dispose()
    }
  }, [])

  useEffect(() => {
    canvasRef.current?.setMsPerPixel(devicesManager.chartScale)
  }, [devicesManager.chartScale])

  useRaf(() => {
    if (canvasRef.current && selectedChannel) {
      canvasRef.current.draw([selectedChannel])
    }
    // console.log("draw")
  }, true)

  const toggleMenu = () => setIsMenuOpen((prev) => !prev)

  return (
    <div style={{ width: '100%', height: '100%', position: 'relative' }}>
      <button
        onClick={toggleMenu}
        style={{
          position: 'absolute',
          top: '10px',
          left: '10px',
          zIndex: 10,
        }}
        className='bg-white border-gray-200 border-2 rounded-lg px-4 py-1 hover:hover:bg-[#E2E2E2]'
      >
        {isMenuOpen ? 'Close Menu' : 'Open Menu'}
      </button>

      {isMenuOpen && (
        <div
          style={{
            position: 'absolute',
            zIndex: 12,
          }}
          className='max-h-[40%] min-h-48 overflow-y-auto overflow-x-hidden bg-white border-gray-200 border-2 rounded-lg p-4 top-16 left-3 bg-opacity-90'
        >
          <h4>Select Channel</h4>

          {devicesManager.activeChannels.map(({ device, channel }) => (
            <div key={channel.id} className='mt-2 flex gap-2'>
              <input
                type='radio'
                name='channel'
                checked={selectedChannel?.channelId === channel.id}
                onChange={() => {
                  setSelectedChannel({
                    channelId: channel.id,
                    color: '#000000',
                  })
                }}
              />
              <p>
                {device.deviceInfo.name} - {channel.name}
              </p>
            </div>
          ))}
        </div>
      )}

      <div className='w-full h-full flex flex-row items-center'>
        {/* Need to add frequency scale */}
        <div className='absolute max-w-20 left-3'>Frequency 0 - 20kHz</div>
        <div className='h-full w-full pt-20 pb-20 mr-12 ml-24'>
          <div className='h-full w-full'>
            <div
              ref={canvasContainerRef}
              style={{
                display: 'block',
                width: '100%',
                height: '100%',
                overflow: 'hidden',
              }}
              className='border-[#E2E2E2] border-2'
              // Scroll to change speed -need to link to respective channel for strain
              onWheel={(e) => {
                setMsPerPixel((prev) => {
                  const newMsPerPixel = prev * (1 + e.deltaY / 1000)
                  devicesManager.setScale(newMsPerPixel)
                  return newMsPerPixel
                })
              }}
            />
          </div>

          {/* Need to add time scale */}
          <div className='text-center'>Time</div>
        </div>
      </div>
    </div>
  )
})
