import { defineConfig } from 'vite'
import react from '@vitejs/plugin-react'
import wasmPack from 'vite-plugin-wasm-pack'
import { viteStaticCopy } from 'vite-plugin-static-copy'

// https://vitejs.dev/config/
export default defineConfig({
  plugins: [
    react(),
    wasmPack('../firmware-common-ffi'),
    viteStaticCopy({
      targets: [
        {
          src: './node_modules/firmware-common-ffi/firmware_common_ffi_bg.wasm',
          dest: '.',
        },
      ],
    }),
  ],
  resolve: {
    alias: {
      '@tabler/icons-react': '@tabler/icons-react/dist/esm/icons/index.mjs',
    },
  },
})
