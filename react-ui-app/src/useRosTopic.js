import { useEffect, useState } from 'react';
import ROSLIB from 'roslib';
import { useRos } from './rosContext';

const useRosTopic = (topicName, messageType) => {
  const { ros } = useRos();
  const [message, setMessage] = useState(null);

  useEffect(() => {
    if (ros && topicName && messageType) {
      const topic = new ROSLIB.Topic({
        ros: ros,
        name: topicName,
        messageType: messageType,
      });

      const handleMessage = (msg) => {
        setMessage(msg);
      };

      topic.subscribe(handleMessage);

      return () => {
        topic.unsubscribe(handleMessage);
      };
    }
  }, [ros, topicName, messageType]);

  return message;
};

export default useRosTopic;
