import curses

def main(stdscr):
    # Configurações iniciais da tela
    curses.curs_set(0)
    stdscr.nodelay(1)
    stdscr.timeout(100)  # Tempo de espera por entrada do teclado (100ms)

    # Posição inicial do cursor
    y, x = 3, 50

    while True:
        # Limpa a tela
        stdscr.clear()

        # Desenha o cursor na posição atual
        stdscr.addstr(y, x, "X")

        # Atualiza a tela
        stdscr.refresh()

        # Captura a entrada do teclado
        key = stdscr.getch()

        # Processa a entrada do teclado
        if key == ord('w'):
            y -= 1
        elif key == ord('s'):
            y += 1
        elif key == ord('a'):
            x -= 1
        elif key == ord('d'):
            x += 1
        elif key == ord('q'):
            break  # Sai do loop se a tecla 'q' for pressionada

if __name__ == "__main__":
    curses.wrapper(main)
