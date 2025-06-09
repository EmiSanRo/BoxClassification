import React, { useEffect, useRef, useState } from "react";
import { Typography } from "@mui/material";
import {deepPurple} from "@mui/material/colors";
import ROSLIB from "roslib";

const ROSBRIDGE_URL = "ws://localhost:9090";

const DetectedMessagesShow = ({ topic, mt, label }) => {
  const [value, setValue] = useState("");
  const rosRef = useRef(null);
  const topicRef = useRef(null);

  useEffect(() => {
    rosRef.current = new ROSLIB.Ros({ url: ROSBRIDGE_URL });
    topicRef.current = new ROSLIB.Topic({
      ros: rosRef.current,
      name: topic,
      messageType: mt,
    });

    // Suscribirse y guardar el último mensaje recibido
    topicRef.current.subscribe((msg) => {
      setValue(
        msg.data !== undefined
          ? msg.data
          : JSON.stringify(msg, null, 1)
      );
    });

    return () => {
      if (topicRef.current) topicRef.current.unsubscribe();
      if (rosRef.current) rosRef.current.close();
    };
  }, [topic, mt]);

  return (
    <Typography variant="subtitle1">
      <span>{label}:</span><span style={{ color: deepPurple[900] }}>{value === "" ? "Esperando..." : value}</span>
    </Typography>
  );
};

export default DetectedMessagesShow;
