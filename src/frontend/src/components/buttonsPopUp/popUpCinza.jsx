const BtPopUpCinza = ({ onClick, children }) => {
  function onClick () {
    console.log('Clicou no botão')
  }
    return (
      <>
        <button className="bg-gray-300 hover:bg-gray-400 text-gray-800 font-bold py-2 px-4 rounded inline-flex items-center" onClick={onClick}>{children}</button>
      </>
    )
  }
  
  export default BtPopUpCinza