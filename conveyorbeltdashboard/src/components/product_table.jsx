import React from "react";
import {
  Table,
  TableBody,
  TableCell,
  TableContainer,
  TableHead,
  TableRow,
  Paper,
  Typography,
} from "@mui/material";

const ProductTable = ({ data }) => (
  <TableContainer component={Paper} sx={{ mb: 2 }}>
    <Table aria-label="product table">
      <TableHead>
        <TableRow>
          <TableCell>
            <Typography variant="subtitle1" fontWeight="bold">Aruco ID</Typography>
          </TableCell>
          <TableCell>
            <Typography variant="subtitle1" fontWeight="bold">Box Type</Typography>
          </TableCell>
          <TableCell>
            <Typography variant="subtitle1" fontWeight="bold">Product</Typography>
          </TableCell>
        </TableRow>
      </TableHead>
      <TableBody>
        {data.map((row) => (
          <TableRow key={row.aruco_id + row.box_type}>
            <TableCell>{row.aruco_id}</TableCell>
            <TableCell>{row.box_type}</TableCell>
            <TableCell>{row.Product}</TableCell>
          </TableRow>
        ))}
      </TableBody>
    </Table>
  </TableContainer>
);

export default ProductTable;
