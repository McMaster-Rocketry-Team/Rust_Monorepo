import { useTabAtom } from '../../workspace/useTabAtom'
import { useOzysDevicesManager } from '../../device/OzysDevicesManager'
import { useEffect, useRef, useState } from 'react'
import { useRaf } from 'rooks'
import { observer } from 'mobx-react-lite'
import { produce } from 'immer'
import { SelectedChannel, StrainGraphCanvas } from './StrainGraphCanvas'

export const StrainGraph = observer(() => {
  const devicesManager = useOzysDevicesManager()
  const [selectedChannels, setSelectedChannels] = useTabAtom<SelectedChannel[]>(
    'selectedChannels',
    [],
  )
  const [msPerPixel, setMsPerPixel] = useTabAtom('msPerPixel', 10)

  const canvasContainerRef = useRef<HTMLDivElement>(null)
  const canvasRef = useRef<StrainGraphCanvas | null>(null)
  const [hoverInfo, setHoverInfo] = useState<{
    x: number
    dataIndex: number | null
  } | null>(null)
  const [isMenuOpen, setIsMenuOpen] = useTabAtom('isMenuOpen', false)

  useEffect(() => {
    canvasRef.current = new StrainGraphCanvas(
      msPerPixel,
      canvasContainerRef.current!,
      devicesManager,
    )
    return () => {
      canvasRef.current!.dispose()
    }
  }, [])

  useEffect(() => {
    canvasRef.current?.setMsPerPixel(devicesManager.chartScale)
  }, [devicesManager.chartScale])

  useRaf(() => {
    if (canvasRef.current) {
      canvasRef.current.draw(selectedChannels)
    }
  }, true)

  // Render graph on canvas
  // useEffect(() => {
  //   const canvas = canvasRef.current
  //   if (!canvas) return

  //   const ctx = canvas.getContext('2d')
  //   if (!ctx) return

  //   const resizeCanvas = () => {
  //     if (canvas.parentElement) {
  //       canvas.width = canvas.parentElement.clientWidth
  //       canvas.height = canvas.parentElement.clientHeight
  //     }
  //   }
  //   resizeCanvas() // Initial resize

  //   const handleResize = () => resizeCanvas()
  //   window.addEventListener('resize', handleResize)

  //   // Clear canvas
  //   ctx.clearRect(0, 0, canvas.width, canvas.height)

  //   // Draw grid lines
  //   ctx.strokeStyle = '#e0e0e0'
  //   for (let x = 0; x <= canvas.width; x += 50) {
  //     ctx.beginPath()
  //     ctx.moveTo(x, 0)
  //     ctx.lineTo(x, canvas.height)
  //     ctx.stroke()
  //   }
  //   for (let y = 0; y <= canvas.height; y += 50) {
  //     ctx.beginPath()
  //     ctx.moveTo(0, y)
  //     ctx.lineTo(canvas.width, y)
  //     ctx.stroke()
  //   }

  //   // Define scales
  //   const timeScale = canvas.width / 100
  //   const valueScale = canvas.height / 200

  //   // Draw sensor lines
  //   selectedChannels.forEach(({ color }) => {
  //     ctx.beginPath()
  //     ctx.strokeStyle = color
  //     // TODO
  //     // data.forEach((point, index) => {
  //     //   const x = index * timeScale
  //     //   const y =
  //     //     canvas.height / 2 -
  //     //     (point[key as keyof typeof point] as number) * valueScale
  //     //   if (index === 0) ctx.moveTo(x, y)
  //     //   else ctx.lineTo(x, y)
  //     // })
  //     ctx.stroke()
  //   })

  //   // Draw hover line and values
  //   if (hoverInfo && hoverInfo.dataIndex !== null) {
  //     const { x, dataIndex } = hoverInfo
  //     const hoveredPoint = data[dataIndex]

  //     // Draw vertical hover line
  //     ctx.beginPath()
  //     ctx.strokeStyle = 'gray'
  //     ctx.setLineDash([5, 5])
  //     ctx.moveTo(x, 0)
  //     ctx.lineTo(x, canvas.height)
  //     ctx.stroke()
  //     ctx.setLineDash([])

  //     // Draw hover data values
  //     ctx.font = '12px Arial'
  //     ctx.fillStyle = 'black'
  //     ctx.textAlign = 'left'

  //     const textX = x + 10
  //     const textYStart = 20
  //     ctx.fillText(`Time: ${hoveredPoint.time}ms`, textX, textYStart)
  //     // TODO
  //     // selectedChannelsWithInfo.forEach(({ key, color }, idx) => {
  //     //   ctx.fillText(
  //     //     `${key}: ${(
  //     //       hoveredPoint[key as keyof typeof hoveredPoint] as number
  //     //     ).toFixed(2)}`,
  //     //     textX,
  //     //     textYStart + 15 * (idx + 1),
  //     //   )
  //     // })
  //   }

  //   return () => window.removeEventListener('resize', handleResize)
  // }, [data, hoverInfo, selectedChannels])

  // Handle mouse hover
  // const handleMouseMove = (event: React.MouseEvent) => {
  //   const canvas = canvasRef.current
  //   if (!canvas) return

  //   const rect = canvas.getBoundingClientRect()
  //   const mouseX = event.clientX - rect.left

  //   const timeScale = canvas.width / 100
  //   const dataIndex = Math.floor(mouseX / timeScale)

  //   if (dataIndex >= 0 && dataIndex < data.length) {
  //     setHoverInfo({ x: mouseX, dataIndex })
  //   } else {
  //     setHoverInfo(null)
  //   }
  // }

  // const handleMouseLeave = () => {
  //   setHoverInfo(null)
  // }

  const toggleMenu = () => setIsMenuOpen((prev) => !prev)

  return (
    <div style={{ width: '100%', height: '100%', position: 'relative' }}>
      {/* Toggle Button */}
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

      {/* Popup Menu */}
      {isMenuOpen && (
        <div
          style={{
            position: 'absolute',
            zIndex: 12,
          }}
          className='max-h-[40%] min-h-48 overflow-y-auto overflow-x-hidden bg-white border-gray-200 border-2 rounded-lg p-4 top-16 left-3 bg-opacity-90'
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

      {/* Graph Canvas */}
      <div className='w-auto h-full mr-12 ml-24'>
        <div
          ref={canvasContainerRef}
          style={{
            display: 'block',
            width: '100%',
            height: '100%',
          }}
          onWheel={(e) => {
            setMsPerPixel((prev) => {
              const newMsPerPixel = prev * (1 + e.deltaY / 1000)
              devicesManager.setScale(newMsPerPixel)
              return newMsPerPixel
            })
          }}
        />
      </div>
    </div>
  )
})
