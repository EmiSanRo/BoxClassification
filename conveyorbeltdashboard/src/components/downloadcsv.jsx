import React from "react";
import { Button, Typography } from "@mui/material";
import DownloadIcon from "@mui/icons-material/Download";
import { saveAs } from "file-saver";

// Utility: converts an array of objects to CSV string
function toCSV(data) {
  if (!data || !Array.isArray(data) || !data.length) return "";
  const headers = Object.keys(data[0]);
  const escape = (val) => `"${String(val ?? "").replace(/"/g, '""')}"`;
  const rows = [
    headers.join(","),
    ...data.map((row) => headers.map((field) => escape(row[field])).join(",")),
  ];
  return rows.join("\n");
}

const DownloadCSVButton = ({ data, filename = "data.csv", label = "Download CSV" }) => {
  const handleDownload = () => {
    const csv = toCSV(data);
    const blob = new Blob([csv], { type: "text/csv;charset=utf-8;" });
    saveAs(blob, filename);
  };

  return (
    <Button
      variant="contained"
      color="primary"
      startIcon={<DownloadIcon />}
      onClick={handleDownload}
      sx={{ mt: 2 }}
      disabled={!data || !Array.isArray(data) || !data.length}
    >
      <Typography variant="button">{label}</Typography>
    </Button>
  );
};

export default DownloadCSVButton;
