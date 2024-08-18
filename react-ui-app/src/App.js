import './App.css';
import CameraFeed64 from './rosCameraController.jsx';
import VideoFeed from './webrtcClient.jsx';
import {RosProvider} from './rosContext.js'
import RosTopicController from './rosTopicController.jsx'
import RosLineGraph from './rosGraph.jsx'
import RosHistogram from './rosHistogram.jsx'
import RosTopicSubscriber from './rosTopicSubscriber.jsx';
import Terminal from './terminal.jsx'
import ThreeDPlot from './ros3dplot.jsx';

function App() {
  return (
    <RosProvider>
      <div className="App">
        <ThreeDPlot></ThreeDPlot>
        <RosTopicSubscriber></RosTopicSubscriber>
       <RosLineGraph></RosLineGraph>
       <RosHistogram></RosHistogram>
        <CameraFeed64></CameraFeed64>
        <VideoFeed></VideoFeed>
        <RosTopicController/>
       <Terminal></Terminal>
      </div>
    </RosProvider>
  );
}

export default App;
