import {
  Layout,
  Model,
  TabNode,
  TabSetNode,
  BorderNode,
  ITabSetRenderValues,
} from 'flexlayout-react'
import 'flexlayout-react/style/light.css'

import { Devices } from './devices/Devices'
import { MouseEvent, useEffect, useMemo, useRef, useState } from 'react'

import addIcon from '../assets/add.svg'
import { defaultLayout } from '../workspace/defaultLayout'
import { onAllowDrop } from '../workspace/onAllowDrop'
import { TabIdProvider } from '../workspace/useTabId'
import { StrainGraph } from './straingraph/StrainGraph'
import { Spectrogram } from './spectrogram/Spectrogram'
import { useDebounce } from 'rooks'
import { WebUSBDebugTab } from './webusb_debug/WebUSBDebugTab'

type tabType = 'Strain Graph' | 'Spectrogram' | 'WebUSB Debug'
interface position {
  x: number
  y: number
}

export default function FlexLayout() {
  // Refs and state
  const layoutRef = useRef<Layout | null>(null)
  const [openTabMenu, setOpenTabMenu] = useState<boolean>(false)
  const [currentNode, setCurrentNode] = useState<TabSetNode | BorderNode>()
  const [mousePos, setMousePos] = useState<position>({ x: 0, y: 0 })

  const initModel = useMemo(() => {
    let model = Model.fromJson(defaultLayout)
    try {
      const initModelJson = JSON.parse(localStorage.getItem('model')!)
      model = Model.fromJson(initModelJson)
    } catch (e) {}
    model.setOnAllowDrop(onAllowDrop)
    return model
  }, [])

  const saveModel = useDebounce((model: Model) => {
    localStorage.setItem('model', JSON.stringify(model.toJson()))
  }, 500)

  const factory = (node: TabNode) => {
    const tab = node.getName()
    let component
    if (tab === 'Devices') {
      component = <Devices />
    } else if (tab === 'Strain Graph') {
      component = <StrainGraph />
    } else if (tab === 'Spectrogram') {
      // component = <button>{node.getName()}</button>
      component = <Spectrogram />
    } else if (tab === 'WebUSB Debug') {
      component = <WebUSBDebugTab />
    } else {
      component = <h1>Unknown Tab</h1>
    }
    return <TabIdProvider value={node.getId()}>{component}</TabIdProvider>
  }

  const onAddFromTabSetButton = (
    node: TabSetNode | BorderNode,
    event: MouseEvent,
  ) => {
    setOpenTabMenu(true)
    setCurrentNode(node)
    setMousePos({ x: event.clientX + 20, y: event.clientY + 20 })
  }

  const createTab = (type: tabType) => {
    if (layoutRef.current && currentNode) {
      const addedTab = layoutRef.current.addTabToTabSet(currentNode.getId(), {
        type: 'tab',
        name: type,
      })
      console.log('Added tab:', addedTab)
    }
  }

  const closePopup = () => {
    if (openTabMenu) {
      setOpenTabMenu(false)
    }
  }

  useEffect(() => {
    window.addEventListener('resize', () => {
      closePopup()
    })
  })

  const newTab = (
    node: TabSetNode | BorderNode,
    renderValues: ITabSetRenderValues,
  ) => {
    if (node instanceof TabSetNode) {
      renderValues.stickyButtons.push(
        <button
          key='add-button'
          style={{
            width: '1.1em',
            height: '1.1em',
            border: 'none',
            background: 'transparent',
            cursor: 'pointer',
          }}
          title='Add Tab'
          onClick={(event) => onAddFromTabSetButton(node, event)}
        >
          <img
            src={addIcon}
            alt='Add'
            key='Add button'
            style={{ width: '1.1em', height: '1.1em' }}
            className='flexlayout__tab_toolbar_button'
          />
        </button>,
      )
    }
  }

  return (
    <div className='' onClick={closePopup}>
      {/* Popup tab selection */}
      {openTabMenu ? (
        <div
          className={`flex flex-col items-start bg-[#F7F7F7] z-50 w-[150px] drop-shadow-lg absolute divide-y divide-gray-300`}
          style={{
            left: `${mousePos.x}px`,
            top: `${mousePos.y}px`,
          }}
        >
          <button
            className='px-2 py-1 w-full text-center hover:bg-[#E2E2E2]'
            onClick={() => createTab('Strain Graph')}
          >
            Strain Graph
          </button>
          <button
            className='px-2 py-1 w-full text-center hover:bg-[#E2E2E2]'
            onClick={() => createTab('Spectrogram')}
          >
            Spectrogram
          </button>
          <button
            className='px-2 py-1 w-full text-center hover:bg-[#E2E2E2]'
            onClick={() => createTab('WebUSB Debug')}
          >
            WebUSB Debug
          </button>
        </div>
      ) : null}

      <Layout
        ref={layoutRef}
        model={initModel}
        factory={factory}
        onRenderTabSet={newTab}
        realtimeResize
        onModelChange={saveModel}
      />
    </div>
  )
}
