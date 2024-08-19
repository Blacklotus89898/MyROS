import CameraFeed64 from '../rosCameraController.jsx';
import VideoFeed from '../webrtcClient.jsx';
import RosTopicController from '../rosTopicController.jsx'
import RosLineGraph from '../rosGraph.jsx'
import RosHistogram from '../rosHistogram.jsx'
import RosTopicSubscriber from '../rosTopicSubscriber.jsx';
import Terminal from '../terminal.jsx'
import ThreeDPlot from '../ros3dplot.jsx';
import RosFlexBox from '../rosFlexBox.jsx';
import RosResizableBox from '../rosResizeBox.jsx';
import RosNodeController from '../rosNodeController.jsx';

function Main() {
    return (
        <div style={{ border: '4px solid black' }}>
            <h1>Main Page</h1>
            <div style={{ display: 'flex', flexWrap: 'wrap' }}>
                <RosResizableBox>
                    <CameraFeed64 />
                </RosResizableBox>
                <RosResizableBox>
                    <VideoFeed />
                </RosResizableBox>
                <RosTopicController />
                <RosTopicSubscriber />
                <RosResizableBox>
                    <RosLineGraph />
                </RosResizableBox>
                <RosResizableBox>
                    <RosHistogram />
                </RosResizableBox>
                 <RosFlexBox>
                    <RosNodeController />
                </RosFlexBox> 
                    <RosFlexBox>
                    <Terminal />
                </RosFlexBox>
                <RosResizableBox>
                    <ThreeDPlot/>
                </RosResizableBox>
            </div>
        </div>
    );
}

export default Main;
