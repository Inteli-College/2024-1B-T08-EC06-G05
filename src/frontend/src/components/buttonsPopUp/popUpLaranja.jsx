const BtPopUpLaranja = ({ onClick, children }) => {
    function onClick () {
      console.log('Clicou no botão laranja')
    }
      return (
        <>
          <button className="bg-orange-400 hover:bg-orange-300 text-white font-bold py-2 px-4 rounded inline-flex items-center" onClick={onClick}>{children}</button>
        </>
      )
    }

    export default BtPopUpLaranja