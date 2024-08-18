import React, { useEffect, useRef, useState } from 'react';
import ROSLIB from 'roslib';
import { useRos } from './rosContext'; // Custom hook to access ROS connection

const CameraFeed64 = () => {
  const [imageSrc, setImageSrc] = useState(null);
  const { ros } = useRos();

  useEffect(() => {
    if (ros) {
      // Subscribe to the ROS topic
      const topic = new ROSLIB.Topic({
        ros: ros,
        name: '/camera_frames_0', // Adjust topic name if necessary
        messageType: 'std_msgs/String',
      });

      topic.subscribe((message) => {
        const base64Image = message.data;

        // Decode base64 to binary data
        const image = new Image();
        image.src = `data:image/jpeg;base64,${base64Image}`;

        // Set the image source URL
        image.onload = () => {
          setImageSrc(image.src);
        };
      });

      // Clean up subscription on unmount
      return () => {
        topic.unsubscribe();
      };
    }
  }, [ros]);

  return (
    <div>
      <h2>Camera Feed</h2>
      {imageSrc ? <img src={imageSrc} alt="Camera Feed" style={{ width: '640px', height: '480px', border: '1px solid black' }} /> : <p>Loading...</p>}
    </div>
  );
};

export default CameraFeed64;
