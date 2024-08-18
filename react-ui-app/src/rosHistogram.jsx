import React, { useEffect, useRef, useState } from 'react';
import ROSLIB from 'roslib';
import { useRos } from './rosContext';

const RosHistogram = ({ topic = '/ui_topic', type = 'std_msgs/Float32MultiArray' }) => {
    const canvasRef = useRef(null);
    const { ros } = useRos();
    const [frequencyData, setFrequencyData] = useState({});
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
            drawHistogram(ctx, frequencyData);
        }
    }, [frequencyData]);

    const handleListener = () => {
        if (listener) listener.unsubscribe();

        const newListener = new ROSLIB.Topic({
            ros: ros,
            name: tpc,
            messageType: type,
        });

        newListener.subscribe((msg) => {
            if (msg && Array.isArray(msg.data)) {
                // Compute cumulative frequency data
                setFrequencyData((prevData) => {
                    const newFrequencyData = { ...prevData };
                    msg.data.forEach(value => {
                        newFrequencyData[value] = (newFrequencyData[value] || 0) + 1;
                    });
                    return newFrequencyData;
                });
            }
        });

        setListener(newListener);
    };

    const drawHistogram = (ctx, frequencyData) => {
        const width = ctx.canvas.width;
        const height = ctx.canvas.height;
        const padding = 50;
        const maxFrequency = Math.max(...Object.values(frequencyData), 1);
        const uniqueValues = Object.keys(frequencyData).map(Number).sort((a, b) => a - b);
        const barWidth = (width - 2 * padding) / uniqueValues.length;

        // Clear the canvas
        ctx.clearRect(0, 0, width, height);

        // Draw axis
        ctx.strokeStyle = '#000';
        ctx.lineWidth = 2;
        ctx.beginPath();
        ctx.moveTo(padding, height - padding);
        ctx.lineTo(width - padding, height - padding);
        ctx.lineTo(width - padding, padding);
        ctx.stroke();

        // Draw y-axis labels
        ctx.textAlign = 'right';
        ctx.textBaseline = 'middle';
        ctx.font = '12px Arial';
        const yTicks = 10;
        const yStep = maxFrequency / yTicks;
        for (let i = 0; i <= yTicks; i++) {
            const y = height - padding - (i * (height - 2 * padding) / yTicks);
            ctx.beginPath();
            ctx.moveTo(padding - 5, y);
            ctx.lineTo(padding, y);
            ctx.stroke();
            ctx.fillText(Math.round(i * yStep), padding - 10, y);
        }

        // Draw bars and x-axis labels
        ctx.fillStyle = '#f00';
        ctx.font = '12px Arial';
        ctx.textAlign = 'center';
        uniqueValues.forEach((value, index) => {
            const barHeight = (frequencyData[value] / maxFrequency) * (height - 2 * padding);
            const x = padding + index * barWidth + barWidth / 2;
            const y = height - padding - barHeight;

            // Draw bar
            ctx.fillRect(x - barWidth / 2, y, barWidth - 1, barHeight);

            // Draw X-axis label
            ctx.fillText(value, x, height - padding + 15);
        });

        // Draw Y-axis label
        ctx.save();
        ctx.rotate(-Math.PI / 2);
        ctx.textAlign = 'center';
        ctx.fillText('Frequency', -height / 2, 15);
        ctx.restore();
    };

    const downloadCSV = () => {
        const csvRows = [];
        csvRows.push(['Value', 'Frequency']); // Header row

        Object.entries(frequencyData).forEach(([value, frequency]) => {
            csvRows.push([value, frequency]);
        });

        const csvContent = csvRows.map(row => row.join(',')).join('\n');
        const blob = new Blob([csvContent], { type: 'text/csv' });
        const url = URL.createObjectURL(blob);
        const a = document.createElement('a');
        a.href = url;
        a.download = 'histogram_data.csv';
        a.click();
        URL.revokeObjectURL(url);
    };

    return (
        <div style={{ border: '1px solid black', padding: '10px' }}>
            <h2>Histogram</h2>
            <h3>
                <input
                    type="text"
                    placeholder="Topic name"
                    value={tpc}
                    onChange={(e) => setTopic(e.target.value)}
                />
            </h3>
            <canvas ref={canvasRef} width={600} height={400} style={{ border: '1px solid black' }} />
            <button onClick={downloadCSV} style={{ marginTop: '10px' }}>Download CSV</button>
        </div>
    );
};

export default RosHistogram;
