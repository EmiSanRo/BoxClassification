// components/Card.jsx
import React from "react";
//import './Styles/Card.css';

const Card = ({ title, image, description, children }) => {
  return (
    <div className="card">
      <div className="title-container">
        {title && <h2 className="card-title">{title}</h2>}
        {description && <div className="card-description">{description}</div>}
      </div>
      {image && (
        <img src={image} className="image" alt={title || "Card image"} />
      )}

      <div className="card-body">{children}</div>
    </div>
  );
};

export default Card;