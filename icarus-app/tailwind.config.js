import { heroui } from '@heroui/theme'

/** @type {import('tailwindcss').Config} */
export default {
  content: [
    './index.html',
    './src/layouts/**/*.{js,ts,jsx,tsx,mdx}',
    './src/pages/**/*.{js,ts,jsx,tsx,mdx}',
    './src/components/**/*.{js,ts,jsx,tsx,mdx}',
    './node_modules/@heroui/theme/dist/**/*.{js,ts,jsx,tsx}',
  ],
  theme: {
    extend: {
      fontFamily: {
        mono: ['"Jetbrains Mono"', 'ui-monospace'],
      },
      keyframes: {
        'enter-highlight': {
          '0%': { backgroundColor: 'rgba(233, 123, 43, 1)' },
          '100%': { backgroundColor: 'rgba(233, 123, 43, 0)' },
        },
        enter: {
          '0%':   { opacity: '0', transform: 'translateY(0.5rem)' },
          '100%': { opacity: '1', transform: 'translateY(0)' }
        },
      },
      animation: {
        'enter-highlight': 'enter-highlight 200ms linear forwards',
      },
    },
  },
  darkMode: 'class',
  plugins: [heroui()],
}
