import {
  action,
  computed,
  makeObservable,
  observable,
} from 'mobx'
import { OzysDevice } from './OzysDevice'
import {
  createContext,
  PropsWithChildren,
  useContext,
  useLayoutEffect,
  useState,
} from 'react'
import DatabaseWorker from '../database/DatabaseWorker?worker'
import * as Comlink from 'comlink'
import type { DatabaseWorkerType } from '../database/DatabaseWorker'

class OzysDevicesManager {
  public devices: OzysDevice[] = []
  private dbWorkerScript = new DatabaseWorker()
  private dbWorker = Comlink.wrap<DatabaseWorkerType>(this.dbWorkerScript)

  get activeChannels() {
    const result = []
    for (const device of this.devices) {
      for (const channel of device.deviceInfo.channels) {
        if (channel.connected && channel.enabled) {
          result.push({
            device,
            channel,
          })
        }
      }
    }

    return result
  }

  constructor() {
    this.dbWorker.init()
    makeObservable(this, {
      devices: observable,
      activeChannels: computed,
      addDevice: action,
      disconnectDevice: action,
      disconnectAllDevices: action,
    })
    console.log('OzysDevicesManager created')
  }

  addDevice(device: OzysDevice) {
    device.onRealtimeReadings((channelId, data) => {
      this.dbWorker.onRealtimeReadings(device.deviceInfo.id, channelId, data)
    })
    device.onRealtimeFft((channelId, data) => {
      this.dbWorker.onRealtimeFft(device.deviceInfo.id, channelId, data)
    })
    this.devices.push(device)
  }

  disconnectDevice(deviceId: string) {
    const i = this.devices.findIndex(
      (device) => device.deviceInfo.id === deviceId,
    )
    if (i >= 0) {
      const device = this.devices.splice(i, 1)[0]
      device.disconnect()
    }
  }

  disconnectAllDevices() {
    this.devices.forEach((device) => device.disconnect())
    this.devices = []
    this.dbWorkerScript.terminate()
    console.log('OzysDevicesManager terminated')
  }

  async createRealtimeReadingsPlayer(
    channelId: string,
    sampleRate: number,
    targetSampleOffset: number,
  ) {
    return await this.dbWorker.createRealtimeReadingsPlayer(
      channelId,
      sampleRate,
      targetSampleOffset,
    )
  }
}

const ozysDevicesManagerContext = createContext<OzysDevicesManager | undefined>(
  undefined,
)

export const OzysDevicesManagerProvider = (props: PropsWithChildren) => {
  const [manager, setManager] = useState<OzysDevicesManager | undefined>()
  useLayoutEffect(() => {
    const manager = new OzysDevicesManager()
    setManager(manager)
    return () => {
      manager.disconnectAllDevices()
      setManager(undefined)
    }
  }, [])

  if (!manager) {
    return null
  }

  return (
    <ozysDevicesManagerContext.Provider value={manager}>
      {props.children}
    </ozysDevicesManagerContext.Provider>
  )
}

export const useOzysDevicesManager = () => {
  return useContext(ozysDevicesManagerContext)!
}
