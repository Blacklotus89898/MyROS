import React, { useEffect, useRef, useState } from 'react';
import ROSLIB from 'roslib';
import { useRos } from './rosContext';

const CameraFeed64 = () => {
  const [imageSrc, setImageSrc] = useState(null);
  const { ros } = useRos();
  const canvasRef = useRef(null);

  useEffect(() => {
    if (ros) {
      const topic = new ROSLIB.Topic({
        ros: ros,
        name: '/camera_frames_0',
        messageType: 'std_msgs/String',
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
  }, [ros]);

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

  return (
    <div>
      <h2>Camera Feed Base64</h2>
      {imageSrc ? <img src={imageSrc} alt="Camera Feed" style={{ width: '640px', height: '480px', border: '1px solid black' }} /> : <p>Loading...</p>}
      <canvas ref={canvasRef} style={{ display: 'none' }} />
      <h3>
        <button onClick={captureImage} style={{ display: 'block', marginTop: '10px' }}>
          Capture Image
        </button>
      </h3>
    </div>
  );
};

export default CameraFeed64;
