import React, { useEffect, useState } from 'react';
import ROSLIB from 'roslib';
import {useRos} from './rosContext';

const RosTopicController = () => {
  const [message, setMessage] = useState('');
  const [topic, setTopic] = useState('/chatter');
  const [type, setType] = useState('std_msgs/String');
  const [msg, setMsg] = useState('');
  const {ros} = useRos();
  const [listener, setListener] = useState(null);

  useEffect(() => {
    if (ros != null) {
    handleListener();
    }
  }, [ros]);

  const handleListener = () => {
    if (listener != null) listener.unsubscribe();
    setListener( new ROSLIB.Topic({
      ros: ros,
      name: topic,
      messageType: type,
    }));
    if (listener != null) {
    listener.subscribe((msg) => {
        if (msg != null) {
      console.log('Received message:', msg);
      setMessage(msg.data);
        } else {
      setMessage("No msg")

        }
    });

    }

  }

const  handlePublish =() => {
    listener.publish( new ROSLIB.Message({data: msg}) );
  }


  return (
    <div>
        <h2>Topic Controller</h2>
        <h3><input type="text" placeholder='Topic name' value={topic} onChange={(e) => setTopic(e.target.value)} /></h3>
        <h3><input type="text" placeholder='Topic type: std_msgs/String' value={type} onChange={(e) => setType(e.target.value)} /></h3>
        <h3><input type="text" placeholder='Topic message' value={msg} onChange={(e) => setMsg(e.target.value)} /></h3>
        <h3><button onClick={handlePublish}>Publish message</button></h3>
        <h3><button onClick={handleListener}>Listen to topic</button></h3>

      <p>Latest message: {message}</p>
    </div>
  );
};

export default RosTopicController;

