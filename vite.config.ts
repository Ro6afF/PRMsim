import { defineConfig } from 'vite'
import react from '@vitejs/plugin-react'

// https://vite.dev/config/
export default defineConfig({
  base: '/PRMsim/',
  plugins: [react()],
  resolve: {
    dedupe: ['three']
  },
  optimizeDeps: {
    include: [
      'three',
      'three/addons/controls/OrbitControls',
      'three/addons/controls/TransformControls'
    ]
  }
})
