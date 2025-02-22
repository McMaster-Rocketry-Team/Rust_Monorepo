import { Remote } from 'comlink'
import type { RealtimeSpectrogramPlayer } from '../../database/RealtimeSpectrogramPlayer'
import { OzysDevicesManager } from '../../device/OzysDevicesManager'
import { Mutex } from 'async-mutex'
import { CircularBuffer } from '../../utils/CircularBuffer'
import { debounce } from 'lodash-es'

export type SelectedChannel = {
  channelId: string
  color: string
}

export class SpectrogramCanvas {
  private players: Map<string, Remote<RealtimeSpectrogramPlayer>> = new Map()
  private playersMutex = new Mutex()
  private canvas: HTMLCanvasElement
  private ctx: CanvasRenderingContext2D
  private selectedChannels: SelectedChannel[] = []
  private disposed = false
  private isDrawing = false
  private resizeObserver: ResizeObserver

  private tempCanvas: HTMLCanvasElement
  private tempCtx: CanvasRenderingContext2D

  constructor(
    private msPerPixel: number,
    private container: HTMLDivElement,
    private devicesManager: OzysDevicesManager,
  ) {
    this.canvas = document.createElement('canvas')
    this.canvas.width = container.clientWidth
    this.canvas.height = container.clientHeight
    container.appendChild(this.canvas)
    this.ctx = this.canvas.getContext('2d')!

    this.tempCanvas = document.createElement('canvas')
    this.tempCanvas.width = this.canvas.width
    this.tempCanvas.height = this.canvas.height
    const tempCtx = this.tempCanvas.getContext('2d')
    if (!tempCtx) {
      throw new Error('Failed to get temp context')
    }
    this.tempCtx = tempCtx

    this.resizeObserver = new ResizeObserver(debounce(() => this.resize(), 100))
    this.resizeObserver.observe(container)
  }

  async draw(selectedChannels: SelectedChannel[]) {
    if (this.disposed) return
    if (this.isDrawing) {
      console.warn('Skipping frame')
      return
    }
    this.isDrawing = true

    const channelsDiff = this.diffSelectedChannels(
      this.selectedChannels,
      selectedChannels,
    )

    this.selectedChannels = selectedChannels

    if (channelsDiff.changed !== 0) {
      await this.playersMutex.runExclusive(async () => {
        for (const { channelId } of channelsDiff.added) {
          const player =
            await this.devicesManager.createRealtimeSpectrogramPlayer(
              channelId,
              {
                duration: 1000,
                sampleCount: 1024,
                startTimestamp: Date.now() - 1000,
                minFrequency: 0,
                maxFrequency: 20000,
                frequencySampleCount: 256,
              },
            )
          this.players.set(channelId, player)
        }
        for (const { channelId } of channelsDiff.removed) {
          this.players.get(channelId)?.dispose()
          this.players.delete(channelId)
        }
      })
    }

    const promises = selectedChannels.map(async ({ channelId }) => {
      const player = this.players.get(channelId)
      if (!player) return

      let newData = await player.getNewData()
      // console.log(newData)
      if (newData.length > 0) {
        // if (true) {

        // if(newData.length == 0){
        //   newData = [{timestamp: Date.now(), fft: new Float32Array(10)}]
        // }

        const freqWidth = 0.93 / this.msPerPixel
        const shiftAmount = newData.length * freqWidth

        if (
          this.tempCanvas.width !== this.canvas.width ||
          this.tempCanvas.height !== this.canvas.height
        ) {
          this.tempCanvas.width = this.canvas.width
          this.tempCanvas.height = this.canvas.height
        }

        // save previous data
        this.tempCtx.clearRect(
          0,
          0,
          this.tempCanvas.width,
          this.tempCanvas.height,
        )
        this.tempCtx.drawImage(this.canvas, 0, 0)

        // shift to left
        this.ctx.clearRect(0, 0, this.canvas.width, this.canvas.height)
        this.ctx.drawImage(
          this.tempCanvas,
          shiftAmount,
          0,
          this.canvas.width - shiftAmount,
          this.canvas.height,
          0,
          0,
          this.canvas.width - shiftAmount,
          this.canvas.height,
        )

        // draw new block of data
        for (let i = 0; i < newData.length; i++) {
          const data = newData[i]
          if (data?.fft) {
            for (let j = 0; j < data.fft.length; j++) {
              const freq = data.fft[j]
              const freqHeight = this.canvas.height / data.fft.length
              const color = this.getColor(freq)

              this.ctx.fillStyle = color

              this.ctx.fillRect(
                this.canvas.width - shiftAmount + i * freqWidth,
                this.canvas.height - j * freqHeight,
                freqWidth,
                freqHeight,
              )
            }
          }
        }
      }
    })
    await Promise.all(promises)
    this.isDrawing = false
  }

  // Temporary colour assignments
  getColor(value: number): string {
    // value from 0 to 200 for mic fft
    value = Math.abs(value)

    let r = 0
    let g = 0
    let b = 0

    if (value < 85) {
      r = 255
      g = Math.round((255 * value) / 100)
      b = 0
    } else if (value < 170) {
      r = Math.round(255 - (255 * (value - 85)) / 100)
      g = 255
      b = 0
    } else {
      r = 0
      g = 255
      b = Math.round((255 * (value - 170)) / 100)
    }

    return `rgba(${r}, ${g}, ${b}, 1)`
  }

  dispose() {
    if (this.disposed) return
    this.disposed = true
    this.resizeObserver.disconnect()
    this.canvas.remove()
    this.playersMutex.runExclusive(async () => {
      for (const player of this.players.values()) {
        player.dispose()
      }
    })
  }

  setMsPerPixel(msPerPixel: number) {
    if (msPerPixel === this.msPerPixel) return
    this.msPerPixel = msPerPixel
  }

  private diffSelectedChannels(
    old: SelectedChannel[],
    newChannels: SelectedChannel[],
  ) {
    const removed = old.filter(
      (oldChannel) =>
        !newChannels.find(
          (newChannel) => newChannel.channelId === oldChannel.channelId,
        ),
    )
    const added = newChannels.filter(
      (newChannel) =>
        !old.find(
          (oldChannel) => oldChannel.channelId === newChannel.channelId,
        ),
    )
    return { removed, added, changed: removed.length + added.length }
  }

  private resize() {
    if (this.disposed) return
    if (
      this.canvas.width == this.container.clientWidth &&
      this.canvas.height == this.container.clientHeight
    ) {
      return
    }

    this.canvas.width = this.container.clientWidth
    this.canvas.height = this.container.clientHeight
  }
}
