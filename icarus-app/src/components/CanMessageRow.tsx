import { ReactNode, useState } from 'react'
import {
  ResetMessage,
  UnixTimeMessage,
  NodeStatusMessage,
  BaroMeasurementMessage,
  IMUMeasurementMessage,
  BrightnessMeasurementMessage,
  AmpStatusMessage,
  baroMeasurementMessageGetPressure,
  baroMeasurementMessageGetTemperature,
  baroMeasurementMessageGetAltitude,
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
import { twMerge } from 'tailwind-merge'
import { useInterval } from 'usehooks-ts'

import { CanBusMessage } from '../device/IcarusDevice'

export function CanMessageRow(props: {
  nodeTypeLookupMap: Map<number, string>
  message: CanBusMessage
  count: number
}) {
  const [receivedSecondsAgo, setReceivedSecondsAgo] = useState(0)

  useInterval(() => {
    const now = Date.now()

    setReceivedSecondsAgo(Math.floor((now - props.message.timestamp) / 1000))
  }, 100)

  let nodeTypeStr =
    props.nodeTypeLookupMap.get(props.message.id.node_type) ?? 'unknown'
  const messageType = Object.keys(props.message.message)[0]
  const messageAny: any = Object.values(props.message.message)[0]

  if (messageType === 'PreUnixTime') {
    return <></>
  }

  let inner = (
    <div>
      unknown message:{' '}
      <span className='font-mono'>{props.message.id.message_type}</span>
    </div>
  )

  if (messageType === 'Reset') {
    const message = messageAny as ResetMessage

    inner = (
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
    )
  } else if (messageType === 'UnixTime') {
    const message = messageAny as UnixTimeMessage

    inner = (
      <>
        <LabeledData
          data={message.timestamp_us}
          dataWidth={200}
          label='timestamp us'
        />
      </>
    )
  } else if (messageType === 'NodeStatus') {
    const message = messageAny as NodeStatusMessage

    inner = (
      <>
        <LabeledData data={message.uptime_s} dataWidth={75} label='uptime s' />
        <LabeledData
          data={message.health}
          dataWidth={100}
          highlightKey={message.health}
          label='health'
        />
        <LabeledData
          data={message.mode}
          dataWidth={130}
          highlightKey={message.mode}
          label='mode'
        />
      </>
    )
  } else if (messageType === 'BaroMeasurement') {
    const message = messageAny as BaroMeasurementMessage

    inner = (
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
    )
  } else if (messageType === 'IMUMeasurement') {
    const message = messageAny as IMUMeasurementMessage
    const acc = imuMeasurementMessageGetAcc(message)
    const gyro = imuMeasurementMessageGetGyro(message)

    inner = (
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
  } else if (messageType === 'BrightnessMeasurement') {
    const message = messageAny as BrightnessMeasurementMessage

    inner = (
      <>
        <LabeledData
          data={brightnessMeasurementMessageGetBrightness(message)}
          dataWidth={100}
          label='brightness'
        />
      </>
    )
  } else if (messageType === 'AmpStatus') {
    const message = messageAny as AmpStatusMessage

    function getAmpOutPutStatusText(status: AmpOutputStatus): ReactNode {
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

    inner = (
      <>
        <LabeledData
          data={message.shared_battery_mv / 1000}
          dataWidth={40}
          label='shared bat v'
        />
        <LabeledData
          data={getAmpOutPutStatusText(message.out1)}
          dataWidth={60}
          highlightKey={message.out1}
          label='out 1'
        />
        <LabeledData
          data={getAmpOutPutStatusText(message.out2)}
          dataWidth={60}
          highlightKey={message.out2}
          label='out 2'
        />
        <LabeledData
          data={getAmpOutPutStatusText(message.out3)}
          dataWidth={60}
          highlightKey={message.out3}
          label='out 3'
        />
        <LabeledData
          data={getAmpOutPutStatusText(message.out4)}
          dataWidth={60}
          highlightKey={message.out4}
          label='out 4'
        />
      </>
    )
  } else if (messageType === 'AmpOverwrite') {
    const message = messageAny as AmpOverwriteMessage

    inner = (
      <>
        <LabeledData
          data={message.out1}
          dataWidth={130}
          highlightKey={message.out1}
          label='out 1'
        />
        <LabeledData
          data={message.out2}
          dataWidth={130}
          highlightKey={message.out2}
          label='out 2'
        />
        <LabeledData
          data={message.out3}
          dataWidth={130}
          highlightKey={message.out3}
          label='out 3'
        />
        <LabeledData
          data={message.out4}
          dataWidth={130}
          highlightKey={message.out4}
          label='out 4'
        />
      </>
    )
  } else if (messageType === 'AmpControl') {
    const message = messageAny as AmpControlMessage

    inner = (
      <>
        <LabeledData
          data={message.out1_enable ? 'T' : 'F'}
          dataWidth={50}
          highlightKey={message.out1_enable}
          label='out1 en'
        />
        <LabeledData
          data={message.out2_enable ? 'T' : 'F'}
          dataWidth={50}
          highlightKey={message.out2_enable}
          label='out2 en'
        />
        <LabeledData
          data={message.out3_enable ? 'T' : 'F'}
          dataWidth={50}
          highlightKey={message.out3_enable}
          label='out3 en'
        />
        <LabeledData
          data={message.out4_enable ? 'T' : 'F'}
          dataWidth={50}
          highlightKey={message.out4_enable}
          label='out4 en'
        />
      </>
    )
  } else if (messageType === 'PayloadEPSStatus') {
    const message = messageAny as PayloadEPSStatusMessage

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

    inner = (
      <div>
        <div className='flex gap-4'>
          <LabeledData
            data={message.battery1_mv / 1000}
            dataWidth={40}
            label='bat 1 v'
          />
          <LabeledData
            data={payloadEPSStatusMessageGetBattery1Temperature(message)}
            dataWidth={55}
            label='bat 1 temp'
          />
          <LabeledData
            data={message.battery2_mv / 1000}
            dataWidth={40}
            label='bat 2 v'
          />
          <LabeledData
            data={payloadEPSStatusMessageGetBattery2Temperature(message)}
            dataWidth={55}
            label='bat 2 temp'
          />
        </div>
        <div className='flex gap-4'>
          <LabeledData
            data={getPayloadOutPutStatusText(message.output_3v3)}
            dataWidth={120}
            highlightKey={message.output_3v3}
            label='out 3v3'
          />
          <LabeledData
            data={getPayloadOutPutStatusText(message.output_5v)}
            dataWidth={120}
            highlightKey={message.output_5v}
            label='out 5v'
          />
          <LabeledData
            data={getPayloadOutPutStatusText(message.output_9v)}
            dataWidth={120}
            highlightKey={message.output_9v}
            label='out 9v'
          />
        </div>
      </div>
    )
  } else if (messageType === 'PayloadEPSOutputOverwrite') {
    const message = messageAny as PayloadEPSOutputOverwriteMessage

    inner = (
      <>
        <LabeledData
          data={message.node_id}
          dataWidth={30}
          highlightKey={message.node_id}
          label='node id'
        />
        <LabeledData
          data={message.out_3v3}
          dataWidth={130}
          highlightKey={message.out_3v3}
          label='out 3v3'
        />
        <LabeledData
          data={message.out_5v}
          dataWidth={130}
          highlightKey={message.out_5v}
          label='out 5v'
        />
        <LabeledData
          data={message.out_9v}
          dataWidth={130}
          highlightKey={message.out_9v}
          label='out 9v'
        />
      </>
    )
  } else if (messageType === 'PayloadEPSSelfTest') {
    const message = messageAny as PayloadEPSSelfTestMessage

    function getTrueFalseColoredText(value: boolean) {
      if (value) {
        return <span>T</span>
      } else {
        return <span className='text-red-500'>F</span>
      }
    }

    inner = (
      <>
        <LabeledData
          data={getTrueFalseColoredText(message.battery1_ok)}
          dataWidth={20}
          highlightKey={message.battery1_ok}
          label='bat 1 ok'
        />
        <LabeledData
          data={getTrueFalseColoredText(message.battery2_ok)}
          dataWidth={20}
          highlightKey={message.battery2_ok}
          label='bat 2 ok'
        />
        <LabeledData
          data={getTrueFalseColoredText(message.out_3v3_ok)}
          dataWidth={20}
          highlightKey={message.out_3v3_ok}
          label='out 3v3 ok'
        />
        <LabeledData
          data={getTrueFalseColoredText(message.out_5v_ok)}
          dataWidth={20}
          highlightKey={message.out_5v_ok}
          label='out 5v ok'
        />
        <LabeledData
          data={getTrueFalseColoredText(message.out_9v_ok)}
          dataWidth={20}
          highlightKey={message.out_9v_ok}
          label='out 9v ok'
        />
      </>
    )
  } else if (messageType === 'AvionicsStatus') {
    const message = messageAny as AvionicsStatusMessage

    inner = (
      <>
        <LabeledData
          data={message.flight_stage}
          dataWidth={200}
          highlightKey={message.flight_stage}
          label='flight stage'
        />
      </>
    )
  } else if (messageType === 'IcarusStatus') {
    const message = messageAny as IcarusStatusMessage

    inner = (
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
    )
  } else if (messageType === 'DataTransfer') {
    const message = messageAny as DataTransferMessage

    inner = (
      <>
        <LabeledData data={message.data_type} dataWidth={100} label='type' />
        <LabeledData
          data={message.destination_node_id}
          dataWidth={30}
          label='destination node'
        />
        <LabeledData data={message.data_len} dataWidth={20} label='data len' />
        <LabeledData
          data={message.start_of_transfer ? 'T' : 'F'}
          dataWidth={20}
          highlightKey={message.start_of_transfer}
          label='start'
        />
        <LabeledData
          data={message.end_of_transfer ? 'T' : 'F'}
          dataWidth={20}
          highlightKey={message.end_of_transfer}
          label='end'
        />
      </>
    )
  } else if (messageType === 'Ack') {
    const message = messageAny as AckMessage

    inner = (
      <>
        <LabeledData data={message.node_id} dataWidth={30} label='node id' />
        <LabeledData data={message.crc} dataWidth={70} label='crc' />
      </>
    )
  }

  return (
    <div className='grid grid-cols-subgrid col-span-full p-4'>
      <div>{messageType}</div>
      <div>{nodeTypeStr}</div>
      <div className='font-mono'>{props.message.id.node_id}</div>
      <div className='flex items-center gap-4'>{inner}</div>
      <div>
        <span className='font-mono'>{receivedSecondsAgo}</span>s ago
      </div>
      <div className='font-mono'>{props.count}</div>
    </div>
  )
}

function LabeledData(props: {
  label: ReactNode
  data: ReactNode
  dataWidth: number
  highlightKey?: any
}) {
  let data

  if (typeof props.data === 'number') {
    data = Math.round(props.data * 100) / 100
  } else {
    data = props.data
  }

  return (
    <div>
      <span className='opacity-70'>{props.label}:</span>
      <span
        className='font-mono inline-block'
        style={{ width: props.dataWidth }}
      >
        <span
          key={JSON.stringify(props.highlightKey)}
          className={twMerge(
            'px-1',
            props.highlightKey === undefined ? '' : 'enter-highlight',
          )}
          style={{ width: props.dataWidth }}
        >
          {data}
        </span>
      </span>
    </div>
  )
}
