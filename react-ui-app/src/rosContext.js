import React, { createContext, useContext, useEffect, useState } from 'react';
import ROSLIB from 'roslib';

const RosContext = createContext(null);

// Provider used to share the connection to all components
export const RosProvider = ({ children }) => {
  const [ros, setRos] = useState(null);
  const [connection, setConnection] = useState('Not Connected');

  useEffect(() => {
    const rosInstance = new ROSLIB.Ros({
      url: 'ws://localhost:9090', //url of the rosbridge websocket
    });
    rosInstance.on('connection', () => {
      console.log('Connected to ROS!');
      setConnection("Connected");
    });

    rosInstance.on('error', (error) => {
      console.error('Error connecting to ROS:', error);
      setConnection(error);
    });

    rosInstance.on('close', () => {
      console.log('Connection to ROS closed.');
      setConnection("Connection Closed");
    });
    setRos(rosInstance);
  }, []);

  return (
    <RosContext.Provider value={{ros, connection}}>
      {children}
    </RosContext.Provider>
  );
};

// Custom hook 
export const useRos = () => {
  const context = useContext(RosContext);
  if (context === null) {
    // throw new Error('useRos must be used within a RosProvider or the context is null');
  }
  return context;
};

