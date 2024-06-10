import random
from datetime import datetime
from tinydb import TinyDB, Query

db = TinyDB('pipes.json')
Pipes = Query()
pipes_table = db.table('pipes')

i = 1

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

    status_dirt = OQUEPEGARDAIA

    dirt_percentage = OQUEPEGARDAIA

    id_boiler = OQUEPEGARDOFRONT
    
    try:
        
        db.insert({'id': i, 'status': dirt_percentage, 'id-boiler': id_boiler, 'dirty-grade': dirty_grade, 'datetime': current_datetime})
        i+=1
    except:
        print("Erro!")
    
    # Busca pelo registro inserido
    user = pipes_table.get(Pipes.id == 1)
    print(user)

if __name__ == '__main__':
    main()