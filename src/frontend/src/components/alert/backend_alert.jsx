import React from 'react';
import { Alert } from "@material-tailwind/react";

export function BackendAlert({ message }) {
  return (
    <Alert className="bg-red-500 text-white p-6 text-xl w-full max-w-lg mx-auto mt-4 shadow-lg rounded-lg">
      {message}
    </Alert>
  );
}
