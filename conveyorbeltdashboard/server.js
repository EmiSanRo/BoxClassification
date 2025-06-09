import express from "express"; //Manejar rutas y peticiones HTTP
import fs from "fs"; // Manejar archivos del sistema
import cors from "cors"; // Permitir peticiones desde otros dominios
import bodyParser from "body-parser"; // Parsear el cuerpo de las peticiones JSON
import ROSLIB from "roslib"; // Librería para interactuar con ROS2

const app = express(); // Crear instancia de Express
const PORT = 4000; // Puerto donde correrá el servidor

// Rutas de los archivos
const PRODUCT_FILE = "./product_data.json"; // Archivo para almacenar productos manualmente ingresados
const DETECTED_FILE = "./detectedproducts.json"; // Archivo para almacenar productos detectados automáticamente

app.use(cors()); 
app.use(bodyParser.json()); // Parsear el cuerpo de las peticiones como JSON

app.use((req, res, next) => {
  const oldSend = res.send;
  res.send = function (data) {
    console.log('Response size:', Buffer.byteLength(data), 'bytes');
    oldSend.apply(res, arguments);
  };
  next();
});


/* PRODUCTOS: Alta y consulta manual */

// Obtener lista de productos
app.get("/products", (req, res) => {
  let data = [];
  try {
    data = JSON.parse(fs.readFileSync(PRODUCT_FILE, "utf-8")); 
  } catch (e) {
    data = [];
  }
  res.json(data);
});

// Agregar producto nuevo (POST)
app.post("/products", (req, res) => {
  const newProduct = req.body;
  let data = [];
  try {
    data = JSON.parse(fs.readFileSync(PRODUCT_FILE, "utf-8"));
  } catch (e) {
    data = [];
  }
  data.push(newProduct);
  fs.writeFileSync(PRODUCT_FILE, JSON.stringify(data, null, 2));
  res.json({ success: true, data });
});

/* ==== DETECTADOS: Consulta automática + cruce con productos ==== */

// Obtener lista de detectados, incluyendo nombre de producto si existe
app.get("/detectedproducts", (req, res) => {
  let detected = [];
  let products = [];
  try {
    detected = JSON.parse(fs.readFileSync(DETECTED_FILE, "utf-8"));
  } catch (e) {
    detected = [];
  }
  try {
    products = JSON.parse(fs.readFileSync(PRODUCT_FILE, "utf-8"));
  } catch (e) {
    products = [];
  }
  // Si el aruco_id y box_type existen en productos, agrega el campo Product: Experimentando para crear informe de incidencias
    const detectedWithName = detected.map(det => {
    const match = products.find(
      prod =>
        String(prod.aruco_id) === String(det.aruco_id) &&
        String(prod.box_type) === String(det.box_type)
    );
    return {
      ...det,
      Product: match ? match.Product : undefined,
    };
  });
  res.json(detectedWithName);
});

/* ==== ROS2 SUSCRIPTOR ==== */

// Conexión rosbridge
const ros = new ROSLIB.Ros({ url: "ws://localhost:9090" });

ros.on("connection", () => {
  console.log("Conectado a rosbridge-server.");
});
ros.on("error", (error) => {
  console.error("Error de conexión a rosbridge-server:", error);
});
ros.on("close", () => {
  console.log("Conexión a rosbridge-server cerrada.");
});

// Suscríbete al tópico de productos detectados (string tipo CSV)
const detectedSubscriber = new ROSLIB.Topic({
  ros: ros,
  name: "/box_state", // Cambia por tu tópico real si es necesario
  messageType: "std_msgs/String",
});

detectedSubscriber.subscribe((msg) => {
  if (!msg.data) return;
  const [aruco_id, box_type, status] = msg.data.split(",");
  if (!(aruco_id && box_type && status)) {
    console.log("Mensaje recibido con formato inválido:", msg.data);
    return;
  }
  const nuevo = {
    aruco_id: Number(aruco_id),
    box_type: box_type,
    status: status,
  };

  // Leer datos actuales del archivo
  let data = [];
  try {
    data = JSON.parse(fs.readFileSync(DETECTED_FILE, "utf-8"));
  } catch (e) {
    data = [];
  }
  data.push(nuevo);
  fs.writeFileSync(DETECTED_FILE, JSON.stringify(data, null, 2));
  console.log("Detectado agregado:", nuevo);
});

app.listen(PORT, () => {
  console.log(`Servidor Node corriendo en http://localhost:${PORT}`);
});
