import React from "react";
import {
  Table, TableBody, TableCell, TableContainer,
  TableHead, TableRow, Paper, Typography
} from "@mui/material";

const DetectedTable = ({ data }) => (
  <TableContainer component={Paper} sx={{ mb: 2 }}>
    <Table aria-label="detected table">
      <TableHead>
        <TableRow>
          <TableCell><Typography fontWeight="bold">Aruco ID</Typography></TableCell>
          <TableCell><Typography fontWeight="bold">Box Type</Typography></TableCell>
          <TableCell><Typography fontWeight="bold">Status</Typography></TableCell>
        </TableRow>
      </TableHead>
      <TableBody>
        {data.map((row, i) => (
          <TableRow key={row.aruco_id + row.box_type + i}>
            <TableCell>{row.aruco_id}</TableCell>
            <TableCell>{row.box_type}</TableCell>
            <TableCell>{row.status}</TableCell>
          </TableRow>
        ))}
      </TableBody>
    </Table>
  </TableContainer>
);

export default DetectedTable;

