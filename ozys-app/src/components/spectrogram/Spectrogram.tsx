import { useTabAtom } from '../../workspace/useTabAtom'
import { useOzysDevicesManager } from '../../device/OzysDevicesManager'
import { useEffect, useRef, useState } from 'react'
import { useRaf } from 'rooks'
import { observer } from 'mobx-react-lite'
import { produce } from 'immer'
import { SelectedChannel, SpectrogramCanvas } from './SpectrogramCanvas'

export const Spectrogram = observer(() => {
  const devicesManager = useOzysDevicesManager()
  const [selectedChannels, setSelectedChannels] = useTabAtom<SelectedChannel[]>(
    'selectedChannels',
    [],
  )
  const [msPerPixel, setMsPerPixel] = useTabAtom('msPerPixel', 10)

  const canvasContainerRef = useRef<HTMLDivElement>(null)
  const canvasRef = useRef<SpectrogramCanvas | null>(null)
  const [hoverInfo, setHoverInfo] = useState<{
    x: number
    dataIndex: number | null
  } | null>(null)
  const [isMenuOpen, setIsMenuOpen] = useTabAtom('isMenuOpen', false)

  useEffect(() => {
    canvasRef.current = new SpectrogramCanvas(
      msPerPixel,
      canvasContainerRef.current!,
      devicesManager,
    )
    return () => {
      canvasRef.current!.dispose()
    }
  }, [])

  useEffect(() => {
    canvasRef.current?.setMsPerPixel(msPerPixel)
  }, [msPerPixel])

  useRaf(() => {
    if (canvasRef.current) {
      canvasRef.current.draw(selectedChannels)
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
          <h4>Data Configurations</h4>
          {devicesManager.activeChannels.map(({ device, channel }) => {
            const selectedChannel = selectedChannels.find(
              (c) => c.channelId === channel.id,
            )
            return (
              <div key={channel.id} className='mt-2 flex gap-2'>
                <input
                  type='checkbox'
                  checked={!!selectedChannel}
                  onChange={(e) => {
                    if (e.target.checked) {
                      setSelectedChannels((prev) => [
                        ...prev,
                        { channelId: channel.id, color: '#000000' },
                      ])
                    } else {
                      setSelectedChannels((prev) =>
                        prev.filter((c) => c.channelId !== channel.id),
                      )
                    }
                  }}
                />
                <p>
                  {device.deviceInfo.name} - {channel.name}
                </p>
                <input
                  type='color'
                  value={selectedChannel?.color || '#000000'}
                  onChange={(e) => {
                    setSelectedChannels((prev) =>
                      produce(prev, (draft) => {
                        draft.find((c) => c.channelId === channel.id)!.color =
                          e.target.value
                      }),
                    )
                  }}
                />
              </div>
            )
          })}
        </div>
      )}

      <div
        ref={canvasContainerRef}
        style={{
          display: 'block',
          width: '100%',
          height: '100%',
          overflow: 'hidden',
        }}
        onWheel={(e) => {
          setMsPerPixel((prev) => {
            const newMsPerPixel = prev * (1 + e.deltaY / 1000)
            return newMsPerPixel
          })
        }}
      />
    </div>
  )
})
