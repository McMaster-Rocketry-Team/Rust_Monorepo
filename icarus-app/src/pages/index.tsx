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
  newBaroMeasurementMessage,
  newPayloadEPSStatusMessage,
  newIcarusStatusMessage,
  newIMUMeasurementMessage,
  baroMeasurementMessageGetPressure,
  baroMeasurementMessageGetTemperature,
  baroMeasurementMessageGetAltitude,
  newBrightnessMeasurementMessage,
  imuMeasurementMessageGetAcc,
  imuMeasurementMessageGetGyro,
  brightnessMeasurementMessageGetBrightness,
  AmpOutputStatus,
  AmpControlMessage,
  PayloadEPSStatusMessage,
  payloadEPSStatusMessageGetBattery1Temperature,
  payloadEPSStatusMessageGetBattery2Temperature,
  PayloadEPSOutputStatus,
  PayloadEPSSelfTestMessage,
  AvionicsStatusMessage,
  IcarusStatusMessage,
  icarusStatusMessageGetExtendedInches,
  icarusStatusMessageGetServoCurrent,
  DataTransferMessage,
  AckMessage,
  PayloadEPSOutputOverwriteMessage,
  AmpOverwriteMessage,
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
        <div className='grid grid-cols-subgrid col-span-full p-4 border-b'>
          <div>Message Type</div>
          <div>Node Type</div>
          <div>Node ID</div>
          <div>Data</div>
          <div>Received Time</div>
          <div>Count</div>
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
                data={message.reset_all ? 'T' : 'F'}
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
              <LabeledData
                data={baroMeasurementMessageGetPressure(message)}
                dataWidth={100}
                label='pressure'
              />
              <LabeledData
                data={baroMeasurementMessageGetAltitude(message)}
                dataWidth={100}
                label='altitude'
              />
              <LabeledData
                data={baroMeasurementMessageGetTemperature(message)}
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
          {(message: IMUMeasurementMessage) => {
            const acc = imuMeasurementMessageGetAcc(message)
            const gyro = imuMeasurementMessageGetGyro(message)

            return (
              <>
                <div>
                  <span className='opacity-70'>acc: </span>
                  <span className='font-mono inline-block w-16'>
                    {Math.round(acc.x * 10) / 10}
                  </span>
                  <span className='font-mono inline-block w-16'>
                    {Math.round(acc.y * 10) / 10}
                  </span>
                  <span className='font-mono inline-block w-16'>
                    {Math.round(acc.z * 10) / 10}
                  </span>
                </div>
                <div>
                  <span className='opacity-70'>gyro: </span>
                  <span className='font-mono inline-block w-16'>
                    {Math.round(gyro.x * 10) / 10}
                  </span>
                  <span className='font-mono inline-block w-16'>
                    {Math.round(gyro.y * 10) / 10}
                  </span>
                  <span className='font-mono inline-block w-16'>
                    {Math.round(gyro.z * 10) / 10}
                  </span>
                </div>
              </>
            )
          }}
        </MessageRow>
        <MessageRow
          message={brightnessMeasurementMessage}
          messageType='BrightnessMeasurement'
          nodeTypeLookupMap={nodeTypeLookupMap}
        >
          {(message: BrightnessMeasurementMessage) => (
            <>
              <LabeledData
                data={brightnessMeasurementMessageGetBrightness(message)}
                dataWidth={100}
                label='brightness'
              />
            </>
          )}
        </MessageRow>
        <MessageRow
          message={ampStatusMessage}
          messageType='AmpStatus'
          nodeTypeLookupMap={nodeTypeLookupMap}
        >
          {(message: AmpStatusMessage) => {
            function getAmpOutPutStatusText(
              status: AmpOutputStatus,
            ): ReactNode {
              const overwrote = status.overwrote ? 'OW' : 'AT'
              let statusText = <></>

              if (status.status === 'Disabled') {
                statusText = <span>D</span>
              } else if (status.status === 'PowerGood') {
                statusText = <span>PG</span>
              } else if (status.status === 'PowerBad') {
                statusText = <span className='text-red-500'>PB</span>
              }

              return (
                <>
                  <span>{overwrote},</span>
                  {statusText}
                </>
              )
            }

            return (
              <>
                <LabeledData
                  data={message.shared_battery_mv / 1000}
                  dataWidth={40}
                  label='shared bat v'
                />
                <LabeledData
                  data={getAmpOutPutStatusText(message.out1)}
                  dataWidth={60}
                  label='out 1'
                />
                <LabeledData
                  data={getAmpOutPutStatusText(message.out2)}
                  dataWidth={60}
                  label='out 2'
                />
                <LabeledData
                  data={getAmpOutPutStatusText(message.out3)}
                  dataWidth={60}
                  label='out 3'
                />
                <LabeledData
                  data={getAmpOutPutStatusText(message.out4)}
                  dataWidth={60}
                  label='out 4'
                />
              </>
            )
          }}
        </MessageRow>
        <MessageRow
          message={ampOverwriteMessage}
          messageType='AmpOverwrite'
          nodeTypeLookupMap={nodeTypeLookupMap}
        >
          {(message: AmpOverwriteMessage) => (
            <>
              <LabeledData data={message.out1} dataWidth={130} label='out 1' />
              <LabeledData data={message.out2} dataWidth={130} label='out 2' />
              <LabeledData data={message.out3} dataWidth={130} label='out 3' />
              <LabeledData data={message.out4} dataWidth={130} label='out 4' />
            </>
          )}
        </MessageRow>
        <MessageRow
          message={ampControlMessage}
          messageType='AmpControl'
          nodeTypeLookupMap={nodeTypeLookupMap}
        >
          {(message: AmpControlMessage) => (
            <>
              <LabeledData
                data={message.out1_enable ? 'T' : 'F'}
                dataWidth={50}
                label='out1 en'
              />
              <LabeledData
                data={message.out2_enable ? 'T' : 'F'}
                dataWidth={50}
                label='out2 en'
              />
              <LabeledData
                data={message.out3_enable ? 'T' : 'F'}
                dataWidth={50}
                label='out3 en'
              />
              <LabeledData
                data={message.out4_enable ? 'T' : 'F'}
                dataWidth={50}
                label='out4 en'
              />
            </>
          )}
        </MessageRow>
        <MessageRow
          message={payloadEPSStatusMessage}
          messageType='PayloadEPSStatus'
          nodeTypeLookupMap={nodeTypeLookupMap}
        >
          {(message: PayloadEPSStatusMessage) => {
            function getPayloadOutPutStatusText(
              status: PayloadEPSOutputStatus,
            ): ReactNode {
              const overwrote = status.overwrote ? 'OW' : 'AT'
              let statusText = <></>

              if (status.status === 'Disabled') {
                statusText = <span>D</span>
              } else if (status.status === 'PowerGood') {
                statusText = <span>PG</span>
              } else if (status.status === 'PowerBad') {
                statusText = <span className='text-red-500'>PB</span>
              }

              return (
                <>
                  <span>{overwrote},</span>
                  {statusText},<span>{status.current_ma}mA</span>
                </>
              )
            }

            return (
              <div>
                <div className='flex gap-4'>
                  <LabeledData
                    data={message.battery1_mv / 1000}
                    dataWidth={40}
                    label='bat 1 v'
                  />
                  <LabeledData
                    data={payloadEPSStatusMessageGetBattery1Temperature(
                      message,
                    )}
                    dataWidth={55}
                    label='bat 1 temp'
                  />
                  <LabeledData
                    data={message.battery2_mv / 1000}
                    dataWidth={40}
                    label='bat 2 v'
                  />
                  <LabeledData
                    data={payloadEPSStatusMessageGetBattery2Temperature(
                      message,
                    )}
                    dataWidth={55}
                    label='bat 2 temp'
                  />
                </div>
                <div className='flex gap-4'>
                  <LabeledData
                    data={getPayloadOutPutStatusText(message.output_3v3)}
                    dataWidth={120}
                    label='out 3v3'
                  />
                  <LabeledData
                    data={getPayloadOutPutStatusText(message.output_5v)}
                    dataWidth={120}
                    label='out 5v'
                  />
                  <LabeledData
                    data={getPayloadOutPutStatusText(message.output_9v)}
                    dataWidth={120}
                    label='out 9v'
                  />
                </div>
              </div>
            )
          }}
        </MessageRow>
        <MessageRow
          message={payloadEPSOutputOverwriteMessage}
          messageType='PayloadEPSOutputOverwrite'
          nodeTypeLookupMap={nodeTypeLookupMap}
        >
          {(message: PayloadEPSOutputOverwriteMessage) => (
            <>
              <LabeledData
                data={message.node_id}
                dataWidth={30}
                label='node id'
              />
              <LabeledData
                data={message.out_3v3}
                dataWidth={130}
                label='out 3v3'
              />
              <LabeledData
                data={message.out_5v}
                dataWidth={130}
                label='out 5v'
              />
              <LabeledData
                data={message.out_9v}
                dataWidth={130}
                label='out 9v'
              />
            </>
          )}
        </MessageRow>
        <MessageRow
          message={payloadEPSSelfTestMessage}
          messageType='PayloadEPSSelfTest'
          nodeTypeLookupMap={nodeTypeLookupMap}
        >
          {(message: PayloadEPSSelfTestMessage) => {
            function getTrueFalseColoredText(value: boolean) {
              if (value) {
                return <span>T</span>
              } else {
                return <span className='text-red-500'>F</span>
              }
            }

            return (
              <>
                <LabeledData
                  data={getTrueFalseColoredText(message.battery1_ok)}
                  dataWidth={20}
                  label='bat 1 ok'
                />
                <LabeledData
                  data={getTrueFalseColoredText(message.battery2_ok)}
                  dataWidth={20}
                  label='bat 2 ok'
                />
                <LabeledData
                  data={getTrueFalseColoredText(message.out_3v3_ok)}
                  dataWidth={20}
                  label='out 3v3 ok'
                />
                <LabeledData
                  data={getTrueFalseColoredText(message.out_5v_ok)}
                  dataWidth={20}
                  label='out 5v ok'
                />
                <LabeledData
                  data={getTrueFalseColoredText(message.out_9v_ok)}
                  dataWidth={20}
                  label='out 9v ok'
                />
              </>
            )
          }}
        </MessageRow>
        <MessageRow
          message={avionicsStatusMessage}
          messageType='AvionicsStatus'
          nodeTypeLookupMap={nodeTypeLookupMap}
        >
          {(message: AvionicsStatusMessage) => (
            <>
              <LabeledData
                data={message.flight_stage}
                dataWidth={200}
                label='flight stage'
              />
            </>
          )}
        </MessageRow>
        <MessageRow
          message={icarusStatusMessage}
          messageType='IcarusStatus'
          nodeTypeLookupMap={nodeTypeLookupMap}
        >
          {(message: IcarusStatusMessage) => (
            <>
              <LabeledData
                data={icarusStatusMessageGetExtendedInches(message)}
                dataWidth={50}
                label='extended in'
              />
              <LabeledData
                data={icarusStatusMessageGetServoCurrent(message)}
                dataWidth={80}
                label='servo current A'
              />
              <LabeledData
                data={message.servo_angular_velocity}
                dataWidth={80}
                label='servo angular velocity'
              />
            </>
          )}
        </MessageRow>
        <MessageRow
          message={dataTransferMessage}
          messageType='DataTransfer'
          nodeTypeLookupMap={nodeTypeLookupMap}
        >
          {(message: DataTransferMessage) => (
            <>
              <LabeledData
                data={message.data_type}
                dataWidth={100}
                label='type'
              />
              <LabeledData
                data={message.destination_node_id}
                dataWidth={30}
                label='destination node'
              />
              <LabeledData
                data={message.data_len}
                dataWidth={20}
                label='data len'
              />
              <LabeledData
                data={message.start_of_transfer ? 'T' : 'F'}
                dataWidth={20}
                label='start'
              />
              <LabeledData
                data={message.end_of_transfer ? 'T' : 'F'}
                dataWidth={20}
                label='end'
              />
              <LabeledData
                data={dataTransferMessage?.crc}
                dataWidth={70}
                label='crc'
              />
            </>
          )}
        </MessageRow>
        <MessageRow
          message={ackMessage}
          messageType='Ack'
          nodeTypeLookupMap={nodeTypeLookupMap}
        >
          {(message: AckMessage) => (
            <>
              <LabeledData
                data={message.node_id}
                dataWidth={30}
                label='node id'
              />
              <LabeledData data={message.crc} dataWidth={70} label='crc' />
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
      <div className='font-mono'>0</div>
    </div>
  )
}

function LabeledData(props: {
  label: ReactNode
  data: ReactNode
  dataWidth: number
}) {
  let data

  if (typeof props.data === 'number') {
    data = Math.round(props.data * 100) / 100
  } else {
    data = props.data
  }

  return (
    <div>
      <span className='opacity-70'>{props.label}: </span>
      <span
        className='font-mono inline-block'
        style={{ width: props.dataWidth }}
      >
        {data}
      </span>
    </div>
  )
}
