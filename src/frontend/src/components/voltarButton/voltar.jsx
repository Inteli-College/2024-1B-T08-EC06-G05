function btVoltarCinza() {
    const onClick = () => {
        window.history.back();
    };
  
    return (
  
      <div className="flex items-center justify-center h-screen">
        <button
            onClick={onClick}
            className="bg-gray-300 text-black font-medium py-2 px-4 rounded focus:outline-none hover:bg-gray-400"
        >
            Voltar
        </button>
      </div>
    );
  }
  
  export default btVoltarCinza;
  
