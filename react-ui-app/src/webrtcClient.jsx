import React, { useEffect, useRef, useState } from 'react';
// need improvement, only functional in chrome, require webrtc_server.py
const waitForIceGatheringComplete = (pc) => {
  return new Promise((resolve) => {
    const checkState = () => {
      if (pc.iceGatheringState === 'complete') {
        pc.removeEventListener('icegatheringstatechange', checkState);
        resolve();
      }
    };

    if (pc.iceGatheringState === 'complete') {
      resolve();
    } else {
      pc.addEventListener('icegatheringstatechange', checkState);
    }
  });
};
const VideoFeed = () => {
  const [pc, setPc] = useState(null);
  const [error, setError] = useState(null);
  const videoElementRef = useRef(null);
  const hostIp = "127.0.0.1";
  const deviceId = 0;

  useEffect(() => {
    return () => {
      stop();
    };
  }, []);

  const negotiate = async () => {
    try {
      if (!pc) return;

      pc.addTransceiver('video', { direction: 'recvonly' });

      const offer = await pc.createOffer();
      await pc.setLocalDescription(offer);

      await waitForIceGatheringComplete(pc);

      const offerData = { sdp: pc.localDescription?.sdp, type: pc.localDescription?.type };
      console.log("Offer data:", offerData);

      const response = await fetch(`http://${hostIp}:8081/offer?id=${deviceId}`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify(offerData),
      });

      console.log("Fetch response:", response);

      const answer = await response.json();
      console.log("Answer data:", answer);

      await pc.setRemoteDescription(new RTCSessionDescription(answer));
    } catch (err) {
      console.error('Error in negotiate:', err);
      setError(`Error: ${err.message || err}`);
    }
  };

  const start = async () => {
    const config = { sdpSemantics: 'unified-plan', iceServers: [] };
    const peerConnection = new RTCPeerConnection(config);

    peerConnection.addEventListener('track', (evt) => {
      if (evt.track.kind === 'video' && videoElementRef.current) {
        videoElementRef.current.srcObject = evt.streams[0];
      } else {
        console.error("Video element not found");
        setError("Video element not found");
      }
    });

    setPc(peerConnection);
    await negotiate();
  };

  const stop = () => {
    if (pc) {
      setTimeout(() => {
        pc.close();
        setPc(null);
      }, 500);
    }
    setError(null);
  };

  return (
    <div style={{ border: '1px solid black', padding: '10px' }}>
      <h2>Camera Feed Webrtc</h2>
      <video
        ref={videoElementRef}
        autoPlay
        style={{ border: '1px solid black', width: '100%', height: '100%' }}
      />
      {error && <div style={{ color: 'red' }}>{`Error: ${error}`}</div>}
      <button onClick={start}>Start</button>
      <button onClick={stop}>Stop</button>
    </div>
  );
};

export default VideoFeed;
