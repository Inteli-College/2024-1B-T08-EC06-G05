import os
import sys

# Função para capturar a entrada do teclado
def getch():
    if os.name == 'nt':  # Para Windows
        import msvcrt
        return msvcrt.getch().decode()
    else:  # Para sistemas baseados em POSIX (Linux, macOS, etc.)
        import tty
        import termios
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

def main():
    # Posição inicial do 'X'
    x, y = 0, 0

    while True:
        # Limpa a tela
        os.system('clear' if os.name == 'posix' else 'cls')

        # Desenha o 'X' na posição atual
        for i in range(y):
            print()
        for i in range(x):
            print(' ', end='')
        print('X')

        # Captura a entrada do teclado
        key = getch()

        # Verifica a entrada do teclado e move o 'X' de acordo
        if key == 'w':
            y = max(0, y - 1)
        elif key == 's':
            y += 1
        elif key == 'a':
            x = max(0, x - 1)
        elif key == 'd':
            x += 1
        elif key == 'q':
            break

if __name__ == "__main__":
    main()
