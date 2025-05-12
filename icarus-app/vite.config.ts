import { defineConfig } from 'vite'
import react from '@vitejs/plugin-react'
import wasmPack from 'vite-plugin-wasm-pack'
import { viteStaticCopy } from 'vite-plugin-static-copy'
import { VitePWA } from 'vite-plugin-pwa'

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
    VitePWA({
      registerType: 'autoUpdate',
      injectRegister: 'script',
      workbox: {
        globPatterns: ['**/*.{js,css,html,png,wasm}'],
      },
      includeAssets: ['favicon.ico', '180.png'],
      manifest: {
        name: 'ICARUS App',
        short_name: 'ICARUS App',
        description: 'ICARUS App',
        theme_color: '#ffffff',
        icons: [
          {
            src: '192.png',
            sizes: '192x192',
            type: 'image/png',
          },
          {
            src: '512.png',
            sizes: '512x512',
            type: 'image/png',
          },
        ],
      },
    }),
  ],
  resolve: {
    alias: {
      '@tabler/icons-react': '@tabler/icons-react/dist/esm/icons/index.mjs',
    },
  },
})
