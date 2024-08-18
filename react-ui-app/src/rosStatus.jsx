import React, { useEffect, useState } from 'react';
import ROSLIB from 'roslib';
import { useRos } from './rosContext';

const RosStatus = () => {
    const { ros, connection } = useRos();
    const [status, setStatus] = useState('');

    useEffect(() => {
        if (ros && connection) {
            setStatus(connection);
        }
    }, [ros, connection]);

    return (
        <div>
            <h1>ROS Web Interface</h1>
            <h2>The ROS connection status: {status}</h2>
            {ros ? <h2>The ROS connection sddktatus: {JSON.stringify(ros._events)}</h2> : <h2>Ros is Null</h2>}
        </div>
    );
};

export default RosStatus;
