import React, { useEffect, useRef, useState } from 'react';
import ROSLIB from 'roslib';
import { useRos } from './rosContext';

const ThreeDPlotCanvas = ({ topic = '/ui_topic', type = 'std_msgs/Float32MultiArray' }) => {
  const canvasRef = useRef(null);
  const { ros } = useRos();
  const [dataPoints, setDataPoints] = useState([]);
  const [angle, setAngle] = useState(Math.PI / 4); // Initial angle (45 degrees)

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
            if (msg.data[i + 2] !== undefined) { // Check if there are enough values
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
      const scale = 200; // Scale factor for projection

      ctx.clearRect(0, 0, width, height);

      // Draw axes
      const axisLength = Math.min(width, height) / 2;

      // X Axis - Red
      ctx.strokeStyle = '#f00';
      ctx.lineWidth = 2;
      ctx.beginPath();
      ctx.moveTo(centerX, centerY);
      ctx.lineTo(centerX + axisLength, centerY);
      ctx.stroke();

      // Y Axis - Green
      ctx.strokeStyle = '#0f0';
      ctx.beginPath();
      ctx.moveTo(centerX, centerY);
      ctx.lineTo(centerX, centerY - axisLength);
      ctx.stroke();

      // Z Axis - Blue
      ctx.strokeStyle = '#00f';
      const zAxisX = centerX + axisLength * Math.cos(angle);
      const zAxisY = centerY - axisLength * Math.sin(angle);
      ctx.beginPath();
      ctx.moveTo(centerX, centerY);
      ctx.lineTo(zAxisX, zAxisY);
      ctx.stroke();

      // Draw axis labels
      ctx.fillStyle = '#000';
      ctx.font = '16px Arial';
      ctx.textAlign = 'center';
      ctx.textBaseline = 'middle';

      // X Axis Label
      ctx.fillStyle = '#f00';
      ctx.fillText('X', centerX + axisLength + 20, centerY);

      // Y Axis Label
      ctx.fillStyle = '#0f0';
      ctx.fillText('Y', centerX, centerY - axisLength - 20);

      // Z Axis Label
      ctx.fillStyle = '#00f';
      ctx.fillText('Z', zAxisX + 20, zAxisY);

      // Draw points
      ctx.fillStyle = '#f00'; // Red color for points
      dataPoints.forEach((point) => {
        if (point.length >= 3) {
          const [x, y, z] = point;

          // Simple angle projection
          const projectedX = centerX + (x * Math.cos(angle) - z * Math.sin(angle)) * scale / (z + scale);
          const projectedY = centerY - (y * Math.cos(angle) - z * Math.sin(angle)) * scale / (z + scale);

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
  }, [dataPoints, angle]);

  return (
    <div style={{ border: '1px solid black', padding: '10px' }}>
      <h2>3D Plot Using Canvas</h2>
      <canvas ref={canvasRef} width={800} height={600} style={{ border: '1px solid black' }} />
      <div style={{ marginTop: '10px' }}>
        <label htmlFor="angleSlider">Projection Angle (degrees): </label>
        <input
          id="angleSlider"
          type="range"
          min="0"
          max="180"
          value={angle * (180 / Math.PI)} // Convert radians to degrees
          onChange={(e) => setAngle(e.target.value * (Math.PI / 180))} // Convert degrees to radians
        />
        <span>{(angle * (180 / Math.PI)).toFixed(2)}°</span>
      </div>
    </div>
  );
};

export default ThreeDPlotCanvas;
