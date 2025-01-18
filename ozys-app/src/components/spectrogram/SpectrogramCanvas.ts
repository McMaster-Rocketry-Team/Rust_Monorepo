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
      this.playersMutex.runExclusive(async () => {
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
      const newData = await player.getNewData()
      console.log(`Channel ${channelId} Data:`, newData)
    })

    await Promise.all(promises)
    this.isDrawing = false
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
    this.canvas.width = this.container.clientWidth
    this.canvas.height = this.container.clientHeight
  }
}
