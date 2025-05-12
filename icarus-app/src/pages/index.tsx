import { Button } from '@heroui/button'
import init, {
  getCanBusNodeTypes,
  CanBusMessageEnum,
  encodeCanBusMessage,
  processCanBusFrame,
} from 'firmware-common-ffi'

import { ThemeSwitch } from '../components/theme-switch'
import { requestIcarusDevice } from '../device/IcarusDevice'

export default function IndexPage() {
  return (
    <div>
      <ThemeSwitch />
      <Button
        onPress={async () => {
          await requestIcarusDevice()
          // await init()
          // const message: CanBusMessageEnum = {
          //   DataTransfer: {
          //     data: [
          //       1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 0, 0, 0,
          //       0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
          //     ],
          //     data_len: 16,
          //     start_of_transfer: true,
          //     end_of_transfer: true,
          //     data_type: 'Data',
          //     destination_node_id: 10,
          //   },
          // }

          // const buffer = new Uint8Array(64)
          // const encodedFrame = encodeCanBusMessage(
          //   message,
          //   getCanBusNodeTypes().icarus,
          //   0,
          //   buffer,
          // )

          // console.log(encodedFrame)

          // for (let i = 0; i < encodedFrame.len; i += 8) {
          //   const subArray = buffer.subarray(
          //     i,
          //     Math.min(i + 8, encodedFrame.len),
          //   )
          //   const result = processCanBusFrame(
          //     BigInt(0),
          //     encodedFrame.id,
          //     subArray,
          //   )
          //   console.log(result)
          // }
        }}
      >
        Run
      </Button>
    </div>
  )
}
