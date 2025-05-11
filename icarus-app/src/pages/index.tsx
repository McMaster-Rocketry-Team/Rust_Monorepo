import { Button } from '@heroui/button'
import init, { getCanBusNodeTypes } from 'firmware-common-ffi'

import { ThemeSwitch } from '../components/theme-switch'

export default function IndexPage() {
  return (
    <div>
      <ThemeSwitch />
      <Button
        onPress={async () => {
          await init()
          console.log('wasm loaded')
          console.log(getCanBusNodeTypes())
        }}
      >
        Run
      </Button>
    </div>
  )
}
