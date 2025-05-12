import React from 'react'
import ReactDOM from 'react-dom/client'
import { BrowserRouter } from 'react-router-dom'
import init from 'firmware-common-ffi'

import App from './App.tsx'
import { Provider } from './provider.tsx'
import './styles/globals.css'

init().then(() =>
  ReactDOM.createRoot(document.getElementById('root')!).render(
    <React.StrictMode>
      <BrowserRouter>
        <Provider>
          <App />
        </Provider>
      </BrowserRouter>
    </React.StrictMode>,
  ),
)
