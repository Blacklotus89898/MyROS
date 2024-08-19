import React, { useEffect, useRef, useState } from 'react';
import ROSLIB from 'roslib';
import { useRos } from './rosContext';
// import RosTopicForm from './RosTopicForm'; // Import the RosTopicForm component
import RosTopicForm from './rosTopicForm.jsx';

const CameraFeed64 = () => {
  const [imageSrc, setImageSrc] = useState(null);
  const [topicName, setTopicName] = useState('/camera_frames_0');
  const [messageType, setMessageType] = useState('std_msgs/String');
  const { ros } = useRos();
  const canvasRef = useRef(null);

  useEffect(() => {
    if (ros) {
      const topic = new ROSLIB.Topic({
        ros: ros,
        name: topicName,
        messageType: messageType,
      });

      topic.subscribe((message) => {
        const base64Image = message.data;

        const image = new Image();
        image.src = `data:image/jpeg;base64,${base64Image}`;

        image.onload = () => {
          setImageSrc(image.src);
        };
      });

      return () => {
        topic.unsubscribe();
      };
    }
  }, [ros, topicName, messageType]);

  const captureImage = () => {
    if (imageSrc && canvasRef.current) {
      const canvas = canvasRef.current;
      const ctx = canvas.getContext('2d');
      const image = new Image();
      image.src = imageSrc;

      image.onload = () => {
        canvas.width = image.width;
        canvas.height = image.height;
        ctx.drawImage(image, 0, 0);

        const link = document.createElement('a');
        link.href = canvas.toDataURL('image/jpeg');
        link.download = 'captured_image.jpg';
        link.click();
      };
    }
  };

  // Callback function to handle the form submission
  const handleFormSubmit = ({ topicName, messageType }) => {
    setTopicName(topicName);
    setMessageType(messageType);
  };

  return (
    <div style={{ border: '1px solid black', padding: '10px', display: 'flex', flexDirection: 'column', height: '100%' }}>
      <h2>Camera Feed Base64</h2>
      
      {/* Use RosTopicForm component */}
      <RosTopicForm onMessageReceived={handleFormSubmit} />

      {imageSrc ? (
        <img src={imageSrc} alt="Camera Feed" style={{ width: '100%', height: 'auto', border: '1px solid black' }} />
      ) : (
        <p>Loading...</p>
      )}
      <canvas ref={canvasRef} style={{ display: 'none' }} />
      <button onClick={captureImage} style={{ marginTop: '10px' }}>
        Capture Image
      </button>
    </div>
  );
};

export default CameraFeed64;
