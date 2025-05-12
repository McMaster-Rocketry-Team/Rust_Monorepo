import { Button } from '@heroui/button'
import { getCanBusNodeTypes } from 'firmware-common-ffi'
import { useMemo, useRef, useState } from 'react'
import { produce } from 'immer'
import { sortBy } from 'lodash-es'

import { ThemeSwitch } from '../components/theme-switch'
import {
  CanBusMessage,
  IcarusDevice,
  requestIcarusDevice,
} from '../device/IcarusDevice'
import { CanMessageRow } from '../components/CanMessageRow'

const messageTypeOrder: Record<string, number> = {
  Reset: 0,
  UnixTime: 1,
  NodeStatus: 2,
  BaroMeasurement: 3,
  IMUMeasurement: 4,
  BrightnessMeasurement: 5,
  AmpStatus: 6,
  AmpOverwrite: 7,
  AmpControl: 8,
  PayloadEPSStatus: 9,
  PayloadEPSOutputOverwrite: 10,
  PayloadEPSSelfTest: 11,
  AvionicsStatus: 12,
  IcarusStatus: 13,
  DataTransfer: 14,
  Ack: 15,
}

const nodeTypeOrder: Record<string, number> = {
  'void lake': 0,
  amp: 1,
  icarus: 2,
  'payload activation': 3,
  'payload rocket wifi': 4,
  ozys: 5,
  bulkhead: 6,
  'payload esp1': 7,
  'payload esp2': 8,
  'aero rust': 9,
}

export default function IndexPage() {
  const deviceRef = useRef<IcarusDevice>()
  const [latestMessages, setLatestMessages] = useState<
    Record<number, { message: CanBusMessage; count: number }>
  >({})
  const sortedMessages = sortBy(
    Object.entries(latestMessages),
    (entry) => entry[0],
  )

  const nodeTypeLookupMap = useMemo(() => {
    const nodeTypeLookupMap = new Map<number, string>()
    const nodeTypes = getCanBusNodeTypes()

    for (const [name, nodeType] of Object.entries(nodeTypes)) {
      nodeTypeLookupMap.set(nodeType, name.replaceAll('_', ' '))
    }

    return nodeTypeLookupMap
  }, [])

  return (
    <div>
      <div className='flex justify-between items-center p-2'>
        <Button
          radius='sm'
          onPress={async () => {
            const device = await requestIcarusDevice(
              (message) => {
                const messageType = Object.keys(message.message)[0]

                if (messageType === 'PreUnixTime') return

                let key = 0

                key += (messageTypeOrder[messageType] ?? 50) * 10000
                const nodeType =
                  nodeTypeLookupMap.get(message.id.node_type) ?? ''

                key += (nodeTypeOrder[nodeType] ?? 50) * 100
                key += message.id.node_id

                setLatestMessages((old) =>
                  produce(old, (draft) => {
                    if (key in draft) {
                      draft[key].count++
                      draft[key].message = message
                    } else {
                      draft[key] = {
                        count: 1,
                        message,
                      }
                    }
                  }),
                )
              },
              () => {
                deviceRef.current = undefined
              },
            )

            if (!device) {
              return
            }

            deviceRef.current = device
          }}
        >
          Connect
        </Button>
        <ThemeSwitch />
      </div>

      <div className='grid grid-cols-[230px_120px_max-content_1fr_150px_max-content] gap-x-4'>
        <div className='grid grid-cols-subgrid col-span-full p-4 border-b sticky top-0 z-10 bg-white dark:bg-black'>
          <div>Message Type</div>
          <div>Node Type</div>
          <div>Node ID</div>
          <div>Data</div>
          <div>Received Time</div>
          <div>Count</div>
        </div>
        {sortedMessages.map(([key, { message, count }]) => (
          <CanMessageRow
            key={key}
            count={count}
            message={message}
            nodeTypeLookupMap={nodeTypeLookupMap}
          />
        ))}
      </div>
    </div>
  )
}
