import CameraFeed64 from '../rosCameraController.jsx';
import VideoFeed from '../webrtcClient.jsx';
import RosTopicController from '../rosTopicController.jsx'
import RosLineGraph from '../rosGraph.jsx'
import RosHistogram from '../rosHistogram.jsx'
import RosTopicSubscriber from '../rosTopicSubscriber.jsx';
import Terminal from '../terminal.jsx'
import ThreeDPlot from '../ros3dplot.jsx';

function Main() {
    return (
        <div>
            <h1>Main Page</h1>
            <ThreeDPlot></ThreeDPlot>
            <RosTopicSubscriber></RosTopicSubscriber>
            <RosLineGraph></RosLineGraph>
            <RosHistogram></RosHistogram>
            <CameraFeed64></CameraFeed64>
            <VideoFeed></VideoFeed>
            <RosTopicController />
            <Terminal></Terminal>
        </div>
    );
}

export default Main;
