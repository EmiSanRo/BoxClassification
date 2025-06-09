import React, { useState, useEffect } from "react";
import ProductTable from "./components/product_table";
import AddProductForm from "./components/add_product_form";
import DownloadCSVButton from "./components/downloadcsv";
import RosCameraView from "./components/camera_component";
import RosSendButton from "./components/RosSendButton";
import DetectedTable from './components/detectedproducts';
import {Typography} from "@mui/material";
import DetectedMessagesShow from "./components/detectedMessagesShow";
import {teal,deepPurple,red, pink, purple} from "@mui/material/colors";
import "./App.css";

const PRODUCTS_URL = "http://localhost:4000/products";
const DETECTED_URL = "http://localhost:4000/detectedproducts";

function App() {
  const [products, setProducts] = useState([]);
  const [detected, setDetected] = useState([]);

  useEffect(() => {
    fetch(PRODUCTS_URL)
      .then(res => res.json())
      .then(setProducts);

    // For detected, you might want polling if you want "live" updates:
    const fetchDetected = () => {
      fetch(DETECTED_URL)
        .then(res => res.json())
        .then(setDetected);
    };
    fetchDetected();
    const interval = setInterval(fetchDetected, 1000);
    return () => clearInterval(interval);
  }, []);

  const handleAdd = (product) => {
    fetch(PRODUCTS_URL, {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(product),
    })
      .then(res => res.json())
      .then(result => setProducts(result.data));
  };

  return (
    <div className="main-container">
      <div className="left-side-container">
        <RosCameraView
            title="1st camera"
            topic="/processed_image/compressed"
            style={{ flex: 1 }}
        />
        <DetectedMessagesShow
            topic="/detected_object_id"
            mt="std_msgs/Int16"
            label="Queued Box ID"
        />
        <RosCameraView
            title="2nd camera"
            topic="/aruco_image/compressed"
            style={{ flex: 1 }}
        />
        <RosSendButton
          topic="/motor2"
          mt="std_msgs/Int32"
          datamsg={{ data: 1 }} // Activate motor
          label="Activar expulsor"
        />
        <RosSendButton
          topic="/motor1"
          mt="std_msgs/Int32"
          datamsg={{ data: 1 }} // Activate motor
          label="Activar pluma"
        />
      </div>
      <div className="center-side-container">
        <ProductTable data={products} />
        <AddProductForm onAdd={handleAdd} />
        <DownloadCSVButton
          data={products}
          filename="products.csv"
          label="Download Products"
        />
      </div>
      <div className="right-side-container">
        <DetectedTable data={detected} />
        <DownloadCSVButton
          data={detected}
          filename="detected.csv"
          label="Download Detections"
        />
        <Typography variant="subtitle1">
          <span>Number of detected products:</span><span style={{ color: red[900] }}>{detected.length}</span>
        </Typography>
      </div>
    </div>
  );
}

export default App;
