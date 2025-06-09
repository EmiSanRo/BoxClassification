import React, { useEffect, useRef, useState } from "react";
import ROSLIB from "roslib";
import {
  Card,
  CardHeader,
  CardContent,
  CardMedia,
  Typography,
  CircularProgress,
  Box,
} from "@mui/material";

const ROSBRIDGE_URL = "ws://localhost:9090";

const RosCameraView = ({ title, topic }) => {
  const [imgSrc, setImgSrc] = useState(null);
  const rosRef = useRef(null);
  const topicRef = useRef(null);

  useEffect(() => {
    rosRef.current = new ROSLIB.Ros({ url: ROSBRIDGE_URL });

    rosRef.current.on("connection", () =>
      console.log("Connected to rosbridge for", topic)
    );
    rosRef.current.on("error", (err) =>
      console.error("Rosbridge error:", err)
    );

    topicRef.current = new ROSLIB.Topic({
      ros: rosRef.current,
      name: topic,
      messageType: "sensor_msgs/CompressedImage",
    });

    topicRef.current.subscribe((msg) => {
      // Muestra en consola el formato recibido para debug
      console.log("Formato recibido:", msg.format);

      // Busca la palabra 'jpeg' en cualquier parte del formato
      if (msg && msg.format.toLowerCase().includes("jpeg")) {
        setImgSrc(`data:image/jpeg;base64,${msg.data}`);
      } else if (msg && msg.format.toLowerCase().includes("png")) {
        setImgSrc(`data:image/png;base64,${msg.data}`);
      } else {
        // Si no reconoce el formato, lo puedes loggear
        console.warn("Formato desconocido de imagen comprimida:", msg.format);
      }
    });


    return () => {
      if (topicRef.current) topicRef.current.unsubscribe();
      if (rosRef.current) rosRef.current.close();
    };
  }, [topic]);

  return (
    <Card
      elevation={3}
      sx={{
        width: 400,
        minHeight: 290,
        margin: 2,
        background: "#bad0ed",
        display: "flex",
        flexDirection: "column",
      }}
    >
      <CardHeader
        title={<Typography variant="h6">{title}</Typography>}
        sx={{ background: "#90caf9" }}
      />
      <CardContent
        sx={{
          display: "flex",
          alignItems: "center",
          justifyContent: "center",
          minHeight: 220,
          p: 0,
          background: "#e4e4e4",
        }}
      >
        {imgSrc ? (
          <CardMedia
            component="img"
            src={imgSrc}
            alt={title}
            sx={{
              width: "100%",
              height: "220px",
              objectFit: "contain",
              borderRadius: 1,
              background: "#e4e4e4",
            }}
          />
        ) : (
          <Box
            sx={{
              width: "100%",
              height: "220px",
              display: "flex",
              flexDirection: "column",
              alignItems: "center",
              justifyContent: "center",
              color: "#888",
              background: "#e4e4e4",
            }}
          >
            <Typography variant="body2">Waiting for image…</Typography>
          </Box>
        )}
      </CardContent>
    </Card>
  );
};

export default RosCameraView;
