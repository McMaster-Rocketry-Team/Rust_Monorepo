import { Button } from '@heroui/button'
import {
  getCanBusNodeTypes,
  CanBusExtendedId,
  newBaroMeasurementMessage,
  newPayloadEPSStatusMessage,
  newIcarusStatusMessage,
  newIMUMeasurementMessage,
  newBrightnessMeasurementMessage,
} from 'firmware-common-ffi'
import { useMemo, useRef, useState } from 'react'

import { ThemeSwitch } from '../components/theme-switch'
import {
  CanBusMessage,
  IcarusDevice,
  requestIcarusDevice,
} from '../device/IcarusDevice'
import { CanMessageRow } from '../components/CanMessageRow'

export default function IndexPage() {
  const deviceRef = useRef<IcarusDevice>()
  const [latestMessages, setLatestMessages] = useState<
    Record<string, CanBusMessage>
  >({
    Reset: {
      timestamp: 0,
      id: new CanBusExtendedId(0, 10, 1, 2),
      crc: 1,
      message: {
        Reset: {
          node_id: 2,
          reset_all: false,
        },
      },
    },
    UnixTime: {
      timestamp: 0,
      id: new CanBusExtendedId(0, 10, 1, 2),
      crc: 1,
      message: {
        UnixTime: {
          timestamp_us: 10000000,
        },
      },
    },
    NodeStatus: {
      timestamp: 0,
      id: new CanBusExtendedId(0, 10, 1, 2),
      crc: 1,
      message: {
        NodeStatus: {
          uptime_s: 10,
          health: 'Healthy',
          mode: 'Operational',
          custom_status: 0,
        },
      },
    },
    BaroMeasurement: {
      timestamp: 0,
      id: new CanBusExtendedId(0, 10, 1, 2),
      crc: 1,
      message: {
        BaroMeasurement: newBaroMeasurementMessage(
          BigInt(123456),
          103325.3,
          25.5,
        ),
      },
    },
    IMUMeasurement: {
      timestamp: 0,
      id: new CanBusExtendedId(0, 10, 1, 2),
      crc: 1,
      message: {
        IMUMeasurement: newIMUMeasurementMessage(
          BigInt(12345),
          {
            x: 1.1,
            y: 1.2,
            z: 1.3,
          },
          {
            x: 1.4,
            y: 1.5,
            z: 1.6,
          },
        ),
      },
    },
    BrightnessMeasurement: {
      timestamp: 0,
      id: new CanBusExtendedId(0, 10, 1, 2),
      crc: 1,
      message: {
        BrightnessMeasurement: newBrightnessMeasurementMessage(
          BigInt(100000),
          1000.0,
        ),
      },
    },
    AmpStatus: {
      timestamp: 0,
      id: new CanBusExtendedId(0, 10, 1, 2),
      crc: 1,
      message: {
        AmpStatus: {
          shared_battery_mv: 10000,
          out1: {
            overwrote: true,
            status: 'PowerGood',
          },
          out2: {
            overwrote: false,
            status: 'PowerBad',
          },
          out3: {
            overwrote: false,
            status: 'Disabled',
          },
          out4: {
            overwrote: false,
            status: 'PowerGood',
          },
        },
      },
    },
    AmpOverwrite: {
      timestamp: 0,
      id: new CanBusExtendedId(0, 10, 1, 2),
      crc: 1,
      message: {
        AmpOverwrite: {
          out1: 'NoOverwrite',
          out2: 'ForceEnabled',
          out3: 'ForceDisabled',
          out4: 'NoOverwrite',
        },
      },
    },
    AmpControl: {
      timestamp: 0,
      id: new CanBusExtendedId(0, 10, 1, 2),
      crc: 1,
      message: {
        AmpControl: {
          out1_enable: true,
          out2_enable: true,
          out3_enable: true,
          out4_enable: true,
        },
      },
    },
    PayloadEPSStatus: {
      timestamp: 0,
      id: new CanBusExtendedId(0, 10, 1, 2),
      crc: 1,
      message: {
        PayloadEPSStatus: newPayloadEPSStatusMessage(
          1000,
          25.4,
          2000,
          20.1,
          {
            current_ma: 100,
            overwrote: true,
            status: 'PowerGood',
          },
          {
            current_ma: 100,
            overwrote: true,
            status: 'PowerBad',
          },
          {
            current_ma: 100,
            overwrote: false,
            status: 'Disabled',
          },
        ),
      },
    },
    PayloadEPSOutputOverwrite: {
      timestamp: 0,
      id: new CanBusExtendedId(0, 10, 1, 2),
      crc: 1,
      message: {
        PayloadEPSOutputOverwrite: {
          out_3v3: 'NoOverwrite',
          out_5v: 'ForceEnabled',
          out_9v: 'ForceDisabled',
          node_id: 2,
        },
      },
    },
    PayloadEPSSelfTest: {
      timestamp: 0,
      id: new CanBusExtendedId(0, 10, 1, 2),
      crc: 1,
      message: {
        PayloadEPSSelfTest: {
          battery1_ok: true,
          battery2_ok: true,
          out_3v3_ok: true,
          out_5v_ok: true,
          out_9v_ok: true,
        },
      },
    },
    AvionicsStatus: {
      timestamp: 0,
      id: new CanBusExtendedId(0, 10, 1, 2),
      crc: 1,
      message: {
        AvionicsStatus: {
          flight_stage: 'ReadyToLaunch',
        },
      },
    },
    IcarusStatus: {
      timestamp: 0,
      id: new CanBusExtendedId(0, 10, 1, 2),
      crc: 1,
      message: {
        IcarusStatus: newIcarusStatusMessage(0.5, 1000, 30),
      },
    },
    DataTransfer: {
      timestamp: 0,
      id: new CanBusExtendedId(0, 10, 1, 2),
      crc: 1,
      message: {
        DataTransfer: {
          data: [0, 1, 2, 3, 4, 5, 0, 0],
          data_len: 6,
          start_of_transfer: true,
          end_of_transfer: true,
          data_type: 'Firmware',
          destination_node_id: 10,
        },
      },
    },
    Ack: {
      timestamp: 0,
      id: new CanBusExtendedId(0, 10, 1, 2),
      crc: 1,
      message: {
        Ack: {
          crc: 1,
          node_id: 10,
        },
      },
    },
  })

  const nodeTypeLookupMap = useMemo(() => {
    const nodeTypeLookupMap = new Map<number, string>()
    const nodeTypes = getCanBusNodeTypes()

    for (const [name, nodeType] of Object.entries(nodeTypes)) {
      nodeTypeLookupMap.set(nodeType, name.replaceAll('_', ' '))
    }

    return nodeTypeLookupMap
  }, [])

  const resetMessage: CanBusMessage | undefined = latestMessages['Reset']
  const unixTimeMessage: CanBusMessage | undefined = latestMessages['UnixTime']
  const nodeStatusMessage: CanBusMessage | undefined =
    latestMessages['NodeStatus']
  const baroMeasurementMessage: CanBusMessage | undefined =
    latestMessages['BaroMeasurement']
  const imuMeasurementMessage: CanBusMessage | undefined =
    latestMessages['IMUMeasurement']
  const brightnessMeasurementMessage: CanBusMessage | undefined =
    latestMessages['BrightnessMeasurement']
  const ampStatusMessage: CanBusMessage | undefined =
    latestMessages['AmpStatus']
  const ampOverwriteMessage: CanBusMessage | undefined =
    latestMessages['AmpOverwrite']
  const ampControlMessage: CanBusMessage | undefined =
    latestMessages['AmpControl']
  const payloadEPSStatusMessage: CanBusMessage | undefined =
    latestMessages['PayloadEPSStatus']
  const payloadEPSOutputOverwriteMessage: CanBusMessage | undefined =
    latestMessages['PayloadEPSOutputOverwrite']
  const payloadEPSSelfTestMessage: CanBusMessage | undefined =
    latestMessages['PayloadEPSSelfTest']
  const avionicsStatusMessage: CanBusMessage | undefined =
    latestMessages['AvionicsStatus']
  const icarusStatusMessage: CanBusMessage | undefined =
    latestMessages['IcarusStatus']
  const dataTransferMessage: CanBusMessage | undefined =
    latestMessages['DataTransfer']
  const ackMessage: CanBusMessage | undefined = latestMessages['Ack']

  return (
    <div>
      <ThemeSwitch />
      <Button
        onPress={async () => {
          const device = await requestIcarusDevice(
            (message) => {
              setLatestMessages((old) => ({
                ...old,
                [Object.keys(message.message)[0]]: message,
              }))
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
      <div className='grid grid-cols-[230px_120px_max-content_1fr_150px_max-content] gap-x-4'>
        <div className='grid grid-cols-subgrid col-span-full p-4 border-b sticky top-0 z-10 bg-white dark:bg-black'>
          <div>Message Type</div>
          <div>Node Type</div>
          <div>Node ID</div>
          <div>Data</div>
          <div>Received Time</div>
          <div>Count</div>
        </div>
        {Object.values(latestMessages).map((message, i) => (
          <CanMessageRow
            key={i}
            message={message}
            nodeTypeLookupMap={nodeTypeLookupMap}
          />
        ))}
      </div>
    </div>
  )
}
