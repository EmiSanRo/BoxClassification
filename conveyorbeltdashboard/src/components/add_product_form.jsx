import React, { useState } from "react";
import { TextField, Button } from "@mui/material";

const AddProductForm = ({ onAdd }) => {
  const [newProduct, setNewProduct] = useState({
    aruco_id: "",
    box_type: "",
    Product: "",
  });

  const handleChange = (e) => {
    setNewProduct({ ...newProduct, [e.target.name]: e.target.value });
  };

  const handleSubmit = (e) => {
    e.preventDefault();
    if (!newProduct.aruco_id || !newProduct.box_type || !newProduct.Product) return;
    onAdd({ ...newProduct, aruco_id: parseInt(newProduct.aruco_id) });
    setNewProduct({ aruco_id: "", box_type: "", Product: "" });
  };

  return (
    <form
      onSubmit={handleSubmit}
      style={{ display: "flex", gap: "1rem", alignItems: "center", marginBottom: "1rem" }}
    >
      <TextField
        label="Aruco ID"
        name="aruco_id"
        value={newProduct.aruco_id}
        onChange={handleChange}
        type="number"
        size="small"
      />
      <TextField
        label="Box Type"
        name="box_type"
        value={newProduct.box_type}
        onChange={handleChange}
        size="small"
      />
      <TextField
        label="Product"
        name="Product"
        value={newProduct.Product}
        onChange={handleChange}
        size="small"
      />
      <Button variant="contained" type="submit">
        Add Product
      </Button>
    </form>
  );
};

export default AddProductForm;
