import { Remote } from 'comlink'
import type { RealtimeStrainGraphPlayer } from '../../database/RealtimeStrainGraphPlayer'
import { OzysDevicesManager } from '../../device/OzysDevicesManager'
import { Mutex } from 'async-mutex'
import { CircularBuffer } from '../../utils/CircularBuffer'
import { debounce } from 'lodash-es'

export type SelectedChannel = {
  channelId: string
  color: string
}

type PlayerWrapper = {
  player: Remote<RealtimeStrainGraphPlayer>
  width: number
  windowDuration: number
  readings: CircularBuffer<{
    timestamp: number
    reading: number
  } | null>
}

export class StrainGraphCanvas {
  private players: Map<string, PlayerWrapper> = new Map()
  private playersMutex = new Mutex()
  private canvas: HTMLCanvasElement
  private ctx: CanvasRenderingContext2D
  private selectedChannels: SelectedChannel[] = []
  private disposed = false
  private isDrawing = false
  private resizeObserver: ResizeObserver

  private width!: number
  private height!: number
  private windowDuration!: number
  private sampleRate!: number
  private sampleDuration!: number

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

    this.resize(true)
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
    const now = Date.now()

    // start is inclusive
    const start = this.alignToSampleDuration(
      now - this.windowDuration + this.sampleDuration - 200,
    )

    // end is also inclusive
    const end = start + this.windowDuration - this.sampleDuration

    const channelsDiff = this.diffSelectedChannels(
      this.selectedChannels,
      selectedChannels,
    )
    this.selectedChannels = selectedChannels
    if (channelsDiff.changed !== 0) {
      this.playersMutex.runExclusive(async () => {
        for (const { channelId } of channelsDiff.added) {
          this.players.set(channelId, await this.createPlayer(channelId))
        }
        for (const { channelId } of channelsDiff.removed) {
          this.players.get(channelId)?.player.dispose()
          this.players.delete(channelId)
        }
      })
    }

    const promises = []
    for (const { channelId } of selectedChannels) {
      const player = this.players.get(channelId)
      if (!player) continue

      promises.push(
        player.player.getNewData().then((newData) => {
          const readings = player.readings
          for (const data of newData) {
            readings.addLast(data)
          }
        }),
      )
    }
    await Promise.all(promises)

    this.ctx.clearRect(0, 0, this.width, this.height)
    for (const { channelId, color } of selectedChannels) {
      const player = this.players.get(channelId)
      if (!player) continue
      if (
        player.width !== this.width ||
        player.windowDuration !== this.windowDuration
      ) {
        continue
      }

      const readings = player.readings

      // console.log(readings)

      this.ctx.beginPath()
      this.ctx.strokeStyle = color
      this.ctx.lineWidth = 1

      let firstPoint = true
      readings.forEach((reading) => {
        if (reading === null) {
          this.ctx.stroke()
          this.ctx.beginPath()
          firstPoint = true
        } else {
          const x = Math.round(
            (reading.timestamp - start) / this.sampleDuration,
          )
          const y = reading.reading * 20 + this.height / 2
          if (firstPoint) {
            this.ctx.moveTo(x, y)
            firstPoint = false
          } else {
            this.ctx.lineTo(x, y)
          }
        }
      })

      this.ctx.stroke()
    }

    this.isDrawing = false
  }

  dispose() {
    if (this.disposed) return
    this.disposed = true
    this.resizeObserver.disconnect()
    this.canvas.remove()
    this.playersMutex.runExclusive(async () => {
      for (const player of this.players.values()) {
        player.player.dispose()
      }
    })
  }

  setMsPerPixel(msPerPixel: number) {
    if (msPerPixel === this.msPerPixel) return
    this.msPerPixel = msPerPixel
    this.calculateWindowDuration()
    this.recreatePlayers()
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

  private resize(initial: boolean = false) {
    if (this.disposed) return
    if (
      !initial &&
      this.canvas.width == this.container.clientWidth &&
      this.canvas.height == this.container.clientHeight
    ) {
      return
    }

    this.canvas.width = this.container.clientWidth
    this.canvas.height = this.container.clientHeight
    this.width = this.container.clientWidth
    this.height = this.container.clientHeight
    this.calculateWindowDuration()

    if (!initial) {
      this.recreatePlayers()
    }
  }

  private calculateWindowDuration() {
    this.windowDuration = this.width * this.msPerPixel
    this.sampleRate = this.width / (this.windowDuration / 1000)
    this.sampleDuration = this.windowDuration / this.width
  }

  private recreatePlayers() {
    this.playersMutex.runExclusive(async () => {
      const newPlayers = new Map<string, PlayerWrapper>()
      for (const channelId of this.players.keys()) {
        newPlayers.set(channelId, await this.createPlayer(channelId))
      }

      const oldPlayers = this.players
      this.players = newPlayers
      for (const player of oldPlayers.values()) {
        player.player.dispose()
      }
    })
  }

  private async createPlayer(channelId: string): Promise<PlayerWrapper> {
    const width = this.width
    const windowDuration = this.windowDuration

    const player = await this.devicesManager.createRealtimeStrainGraphPlayer(
      channelId,
      {
        duration: windowDuration + 400,
        sampleCount: width,
        startTimestamp: this.alignToSampleDuration(
          Date.now() - windowDuration - 400,
        ),
      },
    )

    return {
      player,
      width,
      windowDuration,
      readings: new CircularBuffer(width),
    }
  }

  private alignToSampleDuration(timestamp: number) {
    return timestamp - (timestamp % this.sampleDuration)
  }
}
