import React, { useEffect, useRef, useState } from 'react';
import ROSLIB from 'roslib';
import { useRos } from './rosContext';

const RosLineGraph = ({ topic = '/ui_topic', type = 'std_msgs/Float32MultiArray' }) => {
    const canvasRef = useRef(null);
    const { ros } = useRos();
    const [dataPoints, setDataPoints] = useState([]);
    const [listener, setListener] = useState(null);
    const [tpc, setTopic] = useState(topic);
    useEffect(() => {
        if (ros) {
            handleListener();
        }
    }, [ros]);

    useEffect(() => {
        const canvas = canvasRef.current;
        const ctx = canvas.getContext('2d');
        if (ctx) {
            drawGraph(ctx, dataPoints);
        }
    }, [dataPoints]);

    const handleListener = () => {
        if (listener) listener.unsubscribe();

        const newListener = new ROSLIB.Topic({
            ros: ros,
            name: tpc,
            messageType: type,
        });

        newListener.subscribe((msg) => {
            if (msg && Array.isArray(msg.data)) {
                setDataPoints((prevData) => {
                    const newData = [...prevData, ...msg.data];
                    return newData.slice(-10);
                });
            }
        });

        setListener(newListener);
    };

    const drawGraph = (ctx, dataPoints) => {
        const width = ctx.canvas.width;
        const height = ctx.canvas.height;
        const padding = 50;
        const graphWidth = width - 2 * padding;
        const graphHeight = height - 2 * padding;

        ctx.clearRect(0, 0, width, height);

        ctx.strokeStyle = '#000';
        ctx.lineWidth = 2;
        ctx.beginPath();
        ctx.moveTo(padding, padding);
        ctx.lineTo(padding, height - padding);
        ctx.lineTo(width - padding, height - padding);
        ctx.stroke();

        ctx.strokeStyle = '#ddd';
        ctx.lineWidth = 1;
        const gridStep = 50;
        for (let i = padding; i < width - padding; i += gridStep) {
            ctx.beginPath();
            ctx.moveTo(i, padding);
            ctx.lineTo(i, height - padding);
            ctx.stroke();
        }
        for (let i = height - padding; i > padding; i -= gridStep) {
            ctx.beginPath();
            ctx.moveTo(padding, i);
            ctx.lineTo(width - padding, i);
            ctx.stroke();
        }

        if (dataPoints.length === 0) return;

        // Normalize data
        const maxValue = Math.max(...dataPoints);
        const minValue = Math.min(...dataPoints);
        const xScale = graphWidth / (dataPoints.length - 1);
        const yScale = graphHeight / (maxValue - minValue);

        // Draw lines
        ctx.strokeStyle = '#f00';
        ctx.lineWidth = 2;
        ctx.beginPath();
        dataPoints.forEach((point, index) => {
            const x = padding + index * xScale;
            const y = height - padding - (point - minValue) * yScale;

            const boundedY = Math.max(padding, Math.min(y, height - padding));

            if (index === 0) {
                ctx.moveTo(x, boundedY);
            } else {
                ctx.lineTo(x, boundedY);
            }

            ctx.fillStyle = '#000';
            ctx.font = '10px Arial';
            const text = point.toFixed(2);

            const textWidth = ctx.measureText(text).width;
            const textX = Math.min(Math.max(x + 5, padding), width - padding - textWidth);
            const textY = Math.max(Math.min(boundedY - 5, height - padding - 5), padding + 10);

            ctx.fillText(text, textX, textY);
        });
        ctx.stroke();

        ctx.fillStyle = '#000';
        ctx.font = '12px Arial';
        ctx.textAlign = 'center';
        ctx.fillText('X Axis', width / 2, height - 10);
        ctx.save();
        ctx.rotate(-Math.PI / 2);
        ctx.textAlign = 'center';
        ctx.fillText('Y Axis', -height / 2, 10);
        ctx.restore();
    };

    const downloadCSV = () => {
        const csvRows = [];
        csvRows.push(['Index', 'Value']);

        dataPoints.forEach((point, index) => {
            csvRows.push([index, point.toFixed(2)]);
        });

        const csvContent = csvRows.map(row => row.join(',')).join('\n');
        const blob = new Blob([csvContent], { type: 'text/csv' });
        const url = URL.createObjectURL(blob);
        const a = document.createElement('a');
        a.href = url;
        a.download = 'line_graph_data.csv';
        a.click();
        URL.revokeObjectURL(url);
    };
    const handleTopicChange = () => {
        if (listener) listener.unsubscribe();
        handleListener();
    };

    return (
        <div style={{ border: '1px solid black', padding: '10px' }}>
            <h2>Line Graph</h2>
            <div>
                <input
                    type="text"
                    placeholder="Topic name"
                    value={tpc}
                    onChange={(e) => setTopic(e.target.value)}
                    style={{ marginRight: '10px' }}
                />
                <button onClick={handleTopicChange}>Update Topic</button>
            </div>
            <canvas ref={canvasRef} style={{ border: '1px solid black', width: '100%', height: '100%' }} />
            <button onClick={downloadCSV} style={{ marginTop: '10px' }}>Download CSV</button>
        </div>
    );
};

export default RosLineGraph;
