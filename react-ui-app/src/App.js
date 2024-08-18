import logo from './logo.svg';
import './App.css';
// import RosComponent from './ros.jsx'
import CameraFeed64 from './rosCameraController.jsx';
import VideoFeed from './webrtcClient.jsx';
import {RosProvider} from './rosContext.js'
import RosTopicController from './rosTopicController.jsx'
import Terminal from './terminal.jsx'

function App() {
  return (
    <RosProvider>
      <div className="App">
        <header className="App-header">
          <img src={logo} className="App-logo" alt="logo" />
          <p>
            Edit <code>src/App.js</code> and save to reload.
          </p>
          <a
            className="App-link"
            href="https://reactjs.org"
            target="_blank"
            rel="noopener noreferrer"
          >
            Learn React
          </a>
        </header>
        <CameraFeed64></CameraFeed64>
        <VideoFeed></VideoFeed>
        {/* <RosComponent/> */}
        <RosTopicController/>
       <Terminal></Terminal>
      </div>
    </RosProvider>
  );
}

export default App;
