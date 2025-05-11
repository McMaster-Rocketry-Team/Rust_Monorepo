import './App.css'
import init, {getCanBusNodeTypes} from 'firmware-common-ffi';

function App() {
  return (
    <div>
      hello
      <button onClick={async ()=>{
        await init()
        console.log("wasm loaded")
        console.log(getCanBusNodeTypes())
      }}>
        Run
      </button>
    </div>
  )
}

export default App
