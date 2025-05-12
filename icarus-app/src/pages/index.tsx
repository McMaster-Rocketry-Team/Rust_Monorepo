import { Button } from '@heroui/button'
import {
  getCanBusNodeTypes,
  CanBusExtendedId,
  ResetMessage,
  UnixTimeMessage,
  NodeStatusMessage,
  BaroMeasurementMessage,
  IMUMeasurementMessage,
  BrightnessMeasurementMessage,
  AmpStatusMessage,
} from 'firmware-common-ffi'
import { ReactNode, useMemo, useRef, useState } from 'react'

import { ThemeSwitch } from '../components/theme-switch'
import {
  CanBusMessage,
  IcarusDevice,
  requestIcarusDevice,
} from '../device/IcarusDevice'

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
        BaroMeasurement: {
          pressure_raw: 200,
          temperature: 100,
          timestamp_us: 1000000,
        },
      },
    },
    IMUMeasurement: {
      timestamp: 0,
      id: new CanBusExtendedId(0, 10, 1, 2),
      crc: 1,
      message: {
        IMUMeasurement: {
          acc_raw: [1, 2, 3],
          gyro_raw: [1, 2, 3],
          timestamp_us: 100000,
        },
      },
    },
    BrightnessMeasurement: {
      timestamp: 0,
      id: new CanBusExtendedId(0, 10, 1, 2),
      crc: 1,
      message: {
        BrightnessMeasurement: {
          brightness_raw: 1000,
          timestamp_us: 100000,
        },
      },
    },
    AmpStatus: {
      timestamp: 0,
      id: new CanBusExtendedId(0, 10, 1, 2),
      crc: 1,
      message: {
        AmpStatus: {
          shared_battery_mv: 10000,
          out1: 'PowerGood',
          out2: 'PowerBad',
          out3: 'Disabled',
          out4: 'PowerGood',
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
        PayloadEPSStatus: {
          battery1_mv: 1000,
          battery1_temperature: 1000,
          battery2_mv: 1000,
          battery2_temperature: 1000,
          output_3v3: {
            current_ma: 100,
            overwrote: false,
            status: 'PowerGood',
          },
          output_5v: {
            current_ma: 100,
            overwrote: true,
            status: 'PowerGood',
          },
          output_9v: {
            current_ma: 100,
            overwrote: true,
            status: 'PowerBad',
          },
        },
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
        IcarusStatus: {
          extended_inches: 0.5,
          servo_current: 1000,
          servo_angular_velocity: 0,
        },
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
    latestMessages['DataTransferReset']
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
      <div className='grid grid-cols-[max-content_max-content_max-content_1fr_200px] gap-x-4'>
        <div className='grid grid-cols-subgrid col-span-full p-4 border-b'>
          <div>Message Type</div>
          <div>Node Type</div>
          <div>Node ID</div>
          <div>Data</div>
          <div>Received Time</div>
        </div>
        <MessageRow
          message={resetMessage}
          messageType='Reset'
          nodeTypeLookupMap={nodeTypeLookupMap}
        >
          {(message: ResetMessage) => (
            <>
              <LabeledData
                data={message.node_id}
                dataWidth={50}
                label='reset node id'
              />
              <LabeledData
                data={message.reset_all ? 'true' : 'false'}
                dataWidth={60}
                label='reset all'
              />
            </>
          )}
        </MessageRow>
        <MessageRow
          message={unixTimeMessage}
          messageType='UnixTime'
          nodeTypeLookupMap={nodeTypeLookupMap}
        >
          {(message: UnixTimeMessage) => (
            <>
              <LabeledData
                data={message.timestamp_us}
                dataWidth={200}
                label='timestamp us'
              />
            </>
          )}
        </MessageRow>
        <MessageRow
          message={nodeStatusMessage}
          messageType='NodeStatus'
          nodeTypeLookupMap={nodeTypeLookupMap}
        >
          {(message: NodeStatusMessage) => (
            <>
              <LabeledData
                data={message.uptime_s}
                dataWidth={75}
                label='uptime s'
              />
              <LabeledData
                data={message.health}
                dataWidth={100}
                label='health'
              />
              <LabeledData data={message.mode} dataWidth={100} label='mode' />
            </>
          )}
        </MessageRow>
        <MessageRow
          message={baroMeasurementMessage}
          messageType='BaroMeasurement'
          nodeTypeLookupMap={nodeTypeLookupMap}
        >
          {(message: BaroMeasurementMessage) => (
            <>
              <LabeledData data={'TODO'} dataWidth={100} label='pressure' />
              <LabeledData
                data={message.temperature / 10}
                dataWidth={100}
                label='temperature'
              />
            </>
          )}
        </MessageRow>
        <MessageRow
          message={imuMeasurementMessage}
          messageType='IMUMeasurement'
          nodeTypeLookupMap={nodeTypeLookupMap}
        >
          {(message: IMUMeasurementMessage) => (
            <>
              <LabeledData data={'TODO'} dataWidth={100} label='acc' />
              <LabeledData data={'TODO'} dataWidth={100} label='gyro' />
            </>
          )}
        </MessageRow>
        <MessageRow
          message={brightnessMeasurementMessage}
          messageType='BrightnessMeasurement'
          nodeTypeLookupMap={nodeTypeLookupMap}
        >
          {(message: BrightnessMeasurementMessage) => (
            <>
              <LabeledData data={'TODO'} dataWidth={100} label='brightness' />
            </>
          )}
        </MessageRow>
        <MessageRow
          message={ampStatusMessage}
          messageType='AmpStatus'
          nodeTypeLookupMap={nodeTypeLookupMap}
        >
          {(message: AmpStatusMessage) => (
            <>
              <LabeledData data={'TODO'} dataWidth={100} label='brightness' />
            </>
          )}
        </MessageRow>
      </div>
    </div>
  )
}

function MessageRow(props: {
  nodeTypeLookupMap: Map<number, string>
  messageType: string
  message: CanBusMessage | undefined
  children: (message: any) => ReactNode
}) {
  if (!props.message) {
    return (
      <div className='grid grid-cols-subgrid col-span-full p-4'>
        <div>{props.messageType}</div>
      </div>
    )
  }

  let nodeTypeStr =
    props.nodeTypeLookupMap.get(props.message.id.node_type) ?? 'unknown'

  return (
    <div className='grid grid-cols-subgrid col-span-full p-4'>
      <div>{props.messageType}</div>
      <div>{nodeTypeStr}</div>
      <div className='font-mono'>{props.message.id.node_id}</div>
      <div className='flex items-center gap-4'>
        {props.children(
          (props.message.message as Record<string, any>)[props.messageType],
        )}
      </div>
      <div>{props.message.timestamp}</div>
    </div>
  )
}

function LabeledData(props: {
  label: ReactNode
  data: ReactNode
  dataWidth: number
}) {
  return (
    <div>
      <span className='opacity-70'>{props.label}: </span>
      <span
        className='font-mono inline-block'
        style={{ width: props.dataWidth }}
      >
        {props.data}
      </span>
    </div>
  )
}
