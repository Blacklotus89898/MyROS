import React, { useEffect, useState } from 'react';
import { useRos } from './rosContext';

const RosStatus = () => {
    const { ros, connection } = useRos();
    const [status, setStatus] = useState('');
    const [show, setShowStatus] = useState(false);

    useEffect(() => {
        if (ros && connection) {
            setStatus(connection);
        }
    }, [ros, connection]);

    const statusColor = status === 'Connected' ? 'green' : 'red';

    const toggleStatus = () => {
        setShowStatus(prevState => !prevState);
    };

    return (
        <div style={{ border: '0px solid black', padding: '10px' }}>
            <h1>Ros Status</h1>
            <h2 >The ROS connection status:
                <b style={{ color: statusColor }} >
                    {status}
                </b>
            </h2>
            <button onClick={toggleStatus}> {show ? 'Hide Detail' : 'Show Info'}</button>
            {show &&
                <div>
                    {ros ? <h4>Details: {JSON.stringify(ros)}</h4> : <h4>ROS context is null</h4>}
                </div>
            }
        </div>
    );
};

export default RosStatus;
