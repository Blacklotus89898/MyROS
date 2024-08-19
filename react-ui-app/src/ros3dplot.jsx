import React, { useEffect, useRef, useState } from 'react';
import ROSLIB from 'roslib';
import { useRos } from './rosContext';

const ThreeDPlotCanvas = ({ topic = '/ui_topic', type = 'std_msgs/Float32MultiArray' }) => {
  const canvasRef = useRef(null);
  const { ros } = useRos();
  const [dataPoints, setDataPoints] = useState([]);
  const [yaw, setYaw] = useState(0); // Horizontal rotation angle (radians)
  const [pitch, setPitch] = useState(0); // Vertical rotation angle (radians)
  const [roll, setRoll] = useState(0); // Roll rotation angle (radians)

  useEffect(() => {
    if (ros) {
      const rosTopic = new ROSLIB.Topic({
        ros: ros,
        name: topic,
        messageType: type,
      });

      rosTopic.subscribe((msg) => {
        if (msg && Array.isArray(msg.data)) {
          const points = [];
          for (let i = 0; i < msg.data.length; i += 3) {
            if (msg.data[i + 2] !== undefined) { // Ensure there are enough values
              points.push([msg.data[i], msg.data[i + 1], msg.data[i + 2]]);
            }
          }
          setDataPoints(points);
        }
      });

      return () => {
        rosTopic.unsubscribe();
      };
    }
  }, [ros, topic, type]);

  useEffect(() => {
    const canvas = canvasRef.current;
    const ctx = canvas.getContext('2d');

    if (!ctx) {
      console.error('Canvas context not found');
      return;
    }

    const drawPlot = () => {
      const width = canvas.width;
      const height = canvas.height;
      const centerX = width / 2;
      const centerY = height / 2;
      const axisLength = Math.min(width, height) / 3; // Define the axis length here
      const scale = 300; // Adjusted scale factor for better visualization

      ctx.clearRect(0, 0, width, height);

      // Define rotation matrices for yaw, pitch, and roll
      const cosYaw = Math.cos(yaw);
      const sinYaw = Math.sin(yaw);
      const cosPitch = Math.cos(pitch);
      const sinPitch = Math.sin(pitch);
      const cosRoll = Math.cos(roll);
      const sinRoll = Math.sin(roll);

      // Rotation matrix for 3D rotation
      const rotateX = (x, y, z) => [
        x,
        y * cosPitch - z * sinPitch,
        y * sinPitch + z * cosPitch
      ];

      const rotateY = (x, y, z) => [
        x * cosYaw + z * sinYaw,
        y,
        -x * sinYaw + z * cosYaw
      ];

      const rotateZ = (x, y, z) => [
        x * cosRoll - y * sinRoll,
        x * sinRoll + y * cosRoll,
        z
      ];

      // Draw X Axis
      const [xAxisEndX, xAxisEndY, xAxisEndZ] = rotateX(axisLength, 0, 0);
      const [xAxisEndX2, xAxisEndY2, xAxisEndZ2] = rotateY(xAxisEndX, xAxisEndY, xAxisEndZ);
      const [xAxisEndX3, xAxisEndY3, xAxisEndZ3] = rotateZ(xAxisEndX2, xAxisEndY2, xAxisEndZ2);

      ctx.strokeStyle = '#f00';
      ctx.lineWidth = 2;
      ctx.beginPath();
      ctx.moveTo(centerX, centerY);
      ctx.lineTo(centerX + xAxisEndX3, centerY - xAxisEndY3);
      ctx.stroke();

      // Draw Y Axis
      const [yAxisEndX, yAxisEndY, yAxisEndZ] = rotateX(0, axisLength, 0);
      const [yAxisEndX2, yAxisEndY2, yAxisEndZ2] = rotateY(yAxisEndX, yAxisEndY, yAxisEndZ);
      const [yAxisEndX3, yAxisEndY3, yAxisEndZ3] = rotateZ(yAxisEndX2, yAxisEndY2, yAxisEndZ2);

      ctx.strokeStyle = '#0f0';
      ctx.beginPath();
      ctx.moveTo(centerX, centerY);
      ctx.lineTo(centerX + yAxisEndX3, centerY - yAxisEndY3);
      ctx.stroke();

      // Draw Z Axis
      const [zAxisEndX, zAxisEndY, zAxisEndZ] = rotateX(0, 0, axisLength);
      const [zAxisEndX2, zAxisEndY2, zAxisEndZ2] = rotateY(zAxisEndX, zAxisEndY, zAxisEndZ);
      const [zAxisEndX3, zAxisEndY3, zAxisEndZ3] = rotateZ(zAxisEndX2, zAxisEndY2, zAxisEndZ2);

      ctx.strokeStyle = '#00f';
      ctx.beginPath();
      ctx.moveTo(centerX, centerY);
      ctx.lineTo(centerX + zAxisEndX3, centerY - zAxisEndY3);
      ctx.stroke();

      // Draw axis labels
      ctx.fillStyle = '#000';
      ctx.font = '16px Arial';
      ctx.textAlign = 'center';
      ctx.textBaseline = 'middle';

      // X Axis Label
      ctx.fillStyle = '#f00';
      ctx.fillText('X', centerX + xAxisEndX3 + 20, centerY - xAxisEndY3);

      // Y Axis Label
      ctx.fillStyle = '#0f0';
      ctx.fillText('Y', centerX + yAxisEndX3 + 20, centerY - yAxisEndY3);

      // Z Axis Label
      ctx.fillStyle = '#00f';
      ctx.fillText('Z', centerX + zAxisEndX3 + 20, centerY - zAxisEndY3);

      // Draw points
      ctx.fillStyle = '#f00'; // Red color for points
      dataPoints.forEach((point) => {
        if (point.length >= 3) {
          const [x, y, z] = point;

          // Rotate point
          let [rotX, rotY, rotZ] = rotateX(x, y, z);
          [rotX, rotY, rotZ] = rotateY(rotX, rotY, rotZ);
          [rotX, rotY, rotZ] = rotateZ(rotX, rotY, rotZ);

          // Perspective projection
          const projectedX = centerX + (rotX * scale) / (rotZ + scale);
          const projectedY = centerY - (rotY * scale) / (rotZ + scale);

          // Ensure coordinates are within canvas bounds
          if (projectedX >= 0 && projectedX <= width && projectedY >= 0 && projectedY <= height) {
            ctx.beginPath();
            ctx.arc(projectedX, projectedY, 5, 0, 2 * Math.PI);
            ctx.fill();
          }
        }
      });
    };

    drawPlot();
  }, [dataPoints, yaw, pitch, roll]);

  return (
    <div style={{ border: '1px solid black', padding: '10px' }}>
      <h2>3D Plot Using Canvas</h2>
      <canvas ref={canvasRef} width={800} height={600} style={{ border: '1px solid black', height: '100%', width: '100%' }} />
      <div style={{ marginTop: '10px' }}>
        <label htmlFor="yawSlider">Horizontal Rotation (degrees): </label>
        <input
          id="yawSlider"
          type="range"
          min="0"
          max="360"
          value={yaw * (180 / Math.PI)} // Convert radians to degrees
          onChange={(e) => setYaw(e.target.value * (Math.PI / 180))} // Convert degrees to radians
        />
        <span>{(yaw * (180 / Math.PI)).toFixed(2)}°</span>
      </div>
      <div style={{ marginTop: '10px' }}>
        <label htmlFor="pitchSlider">Vertical Rotation (degrees): </label>
        <input
          id="pitchSlider"
          type="range"
          min="-90"
          max="90"
          value={pitch * (180 / Math.PI)} // Convert radians to degrees
          onChange={(e) => setPitch(e.target.value * (Math.PI / 180))} // Convert degrees to radians
        />
        <span>{(pitch * (180 / Math.PI)).toFixed(2)}°</span>
      </div>
      <div style={{ marginTop: '10px' }}>
        <label htmlFor="rollSlider">Roll Rotation (degrees): </label>
        <input
          id="rollSlider"
          type="range"
          min="-180"
          max="180"
          value={roll * (180 / Math.PI)} // Convert radians to degrees
          onChange={(e) => setRoll(e.target.value * (Math.PI / 180))} // Convert degrees to radians
        />
        <span>{(roll * (180 / Math.PI)).toFixed(2)}°</span>
      </div>
    </div>
  );
};

export default ThreeDPlotCanvas;
