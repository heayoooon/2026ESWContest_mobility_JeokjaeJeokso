import path from 'node:path'
import { defineConfig } from 'vite'
import react from '@vitejs/plugin-react'
import tailwindcss from '@tailwindcss/vite'

// 빌드 결과(dist/)는 디스플레이 파이의 twin_server.py 가 http://localhost:8090/ 에서 서빙한다.
// PC 에서 `npm run dev` 로 볼 때는 /api 를 디스플레이 파이 백엔드로 넘긴다.
export default defineConfig({
  plugins: [react(), tailwindcss()],
  resolve: { alias: { '@': path.resolve(import.meta.dirname, './src') } },
  server: { proxy: { '/api': 'http://100.90.251.42:8090' } },
  build: { target: 'es2022', chunkSizeWarningLimit: 1500 },
})
