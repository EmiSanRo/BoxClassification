import React, { useEffect, useRef } from "react";
import { Button, Typography } from "@mui/material";
import ROSLIB from "roslib";
import SendIcon from '@mui/icons-material/Send'

const ROSBRIDGE_URL = "ws://localhost:9090";

const RosSendButton = ({ topic, mt, datamsg, label }) => {
  const rosRef = useRef(null);
  const topicRef = useRef(null);

  useEffect(() => {
    // Solo crea la conexión y el publisher una vez
    rosRef.current = new ROSLIB.Ros({ url: ROSBRIDGE_URL });
    topicRef.current = new ROSLIB.Topic({
      ros: rosRef.current,
      name: topic,
      messageType: mt,
    });

    return () => {
      if (topicRef.current) topicRef.current.unadvertise();
      if (rosRef.current) rosRef.current.close();
    };
  }, [topic, mt]);

  const handleClick = () => {
    const msg = new ROSLIB.Message(datamsg); // datamsg debe ser objeto con los campos del mensaje
    topicRef.current.publish(msg);
  };

  return (
    <Button
      variant="contained"
      color="error"
      onClick={handleClick}
      sx={{ m: 1 }}
      endIcon={<SendIcon />}
    >
      <Typography variant="button">{label}</Typography>
    </Button>
  );
};

export default RosSendButton;
