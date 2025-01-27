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
  const [msPerPixel, setMsPerPixel] = useTabAtom('msPerPixel', 10)

  const canvasContainerRef = useRef<HTMLDivElement>(null)
  const canvasRef = useRef<SpectrogramCanvas | null>(null)
  const [isMenuOpen, setIsMenuOpen] = useTabAtom('isMenuOpen', false)

  useEffect(() => {
    if (canvasContainerRef.current) {
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
    canvasRef.current?.setMsPerPixel(msPerPixel)
  }, [msPerPixel])

  useRaf(() => {
    if (canvasRef.current && selectedChannel) {
      canvasRef.current.draw([selectedChannel])
    }
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
      >
        {isMenuOpen ? 'Close Menu' : 'Open Menu'}
      </button>

      {isMenuOpen && (
        <div
          style={{
            position: 'absolute',
            top: '40px',
            left: '10px',
            padding: '10px',
            border: '1px solid black',
            backgroundColor: 'white',
            zIndex: 10,
          }}
        >
          <h4>Select Channel</h4>

          {devicesManager.activeChannels.map(({ device, channel }) => (
            <div key={channel.id} className='mt-2 flex gap-2'>
              <input
                type='radio'
                name='channel'
                checked={selectedChannel?.channelId === channel.id}
                onChange={() =>
                  setSelectedChannel({
                    channelId: channel.id,
                    color: '#000000',
                  })
                }
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
        <div className='mx-4'>Frequency 0 - 20kHz</div>
        <div className='h-full w-full pt-12 pb-20 mr-12'>
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
