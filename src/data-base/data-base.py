import random
from datetime import datetime
from tinydb import TinyDB, Query

db = TinyDB('pipes.json')
Pipes = Query()
pipes_table = db.table('pipes')

def main():
    # Simulação de um dirty-grade aleatório para o exemplo
    dirty_grade = random.randint(1, 100)
    
    # Define o status com base no dirty-grade
    if dirty_grade < 30:
        status = True  # Limpo
    else:
        status = False  # Sujo
    
    # Obtém a data e hora atual
    current_datetime = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
    
    # Insere um registro no banco de dados com o status definido
    db.insert({'id': 1, 'status': status, 'id-boiler': 1, 'dirty-grade': dirty_grade, 'datetime': current_datetime})
    
    # Busca pelo registro inserido
    user = pipes_table.get(Pipes.id == 1)
    print(user)

if __name__ == '__main__':
    main()