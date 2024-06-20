from flask import Flask, request, jsonify
from flask_cors import CORS, cross_origin
from tinydb import TinyDB, Query
import os
import random
from datetime import datetime
import cv2
import base64 
import numpy as np
from ultralytics import YOLO

app = Flask(__name__)
CORS(app)  # Habilita CORS para todas as rotas

db_path = os.path.join(os.path.dirname(__file__), '../data-base/pipes.json')
db = TinyDB(db_path)
pipes_table = db.table('pipes')
Pipes = Query()
imgStringBase64 = None
model = YOLO('./best.pt')

updated_data = {
    'id': 0,
    'status': None,
    'id-reboiler': None,
    'datetime': None
}

# rota apenas para receber o ID do reboiler atual inserido pelo usuário no frontend
@app.route('/post_reboiler_id', methods=['POST', 'GET'])
def post_reboiler_id():
    data = request.get_json()
    reboiler_id = data.get('reboilerID')
    updated_data['id-reboiler'] = reboiler_id
    print(f"Id recebido para o reboiler atual: {updated_data['id-reboiler']}")
    return jsonify({"reboilerID": reboiler_id}), 200

# rota que recebe o frame atual capturado pelo usuário através do botão de IA
# o frame chega na forma de string, convertido em base 64
@app.route('/post_img_string', methods=['POST', 'GET'])
@cross_origin()  # Adiciona CORS para esta rota específica
def post_img_string():

    global imgStringBase64

    if request.method == 'POST':
        data = request.get_json()
        imgStringBase64 = data.get('currentFrame')
        print(f"imagem convertida")

        # converte a string da img em base64 p/ variável legível pro opencv
        decoded_data = base64.b64decode(imgStringBase64)
        np_data = np.fromstring(decoded_data,np.uint8)
        img = cv2.imdecode(np_data,cv2.IMREAD_COLOR)
        results = model(img)

        updated_data['id'] += 1
        updated_data['datetime'] = datetime.now().strftime('%Y-%m-%d %H:%M:%S')

        for result in results:
            
            updated_data['status'] = False

            # se mais de 0 sujeiras forem detectadas no frame
            if len(result.boxes.cls) > 0: 
                updated_data['status'] = True

            #img = result.plot()

        pipes_table.insert(updated_data)

        #nome_imagem = f"img{datetime.now().strftime('%Y-%m-%d %H:%M:%S')}.jpg"

        #cv2.imwrite(nome_imagem, img)

        return jsonify({"message": "Imagem recebida com sucesso"}), 200

    return jsonify({"imgStringBase64": imgStringBase64}), 200

@app.route('/pipes', methods=['GET'])
def get_pipes():
    pipes = pipes_table.all()
    return jsonify(pipes), 200

@app.route('/pipes', methods=['POST'])
def add_pipe():
    new_pipe = request.json
    pipes_table.insert(new_pipe)
    return jsonify(new_pipe), 201

@app.route('/pipes/<int:pipe_id>', methods=['GET'])
def get_pipe(pipe_id):
    pipe = pipes_table.get(Pipes.id == pipe_id)
    if pipe:
        return jsonify(pipe), 200
    return jsonify({"error": "Pipe not found"}), 404

@app.route('/pipes/<int:pipe_id>', methods=['POST', 'GET'])
def update_pipe(pipe_id):
    updated_id = request.args.get('id')
    updated_status = request.args.get('status')
    updated_id_reboiler = request.args.get('id-reboiler')
    updated_datetime = request.args.get('datetime')

    if updated_id:
        updated_data['id'] = int(updated_id)
    if updated_status:
        updated_data['status'] = updated_status.lower() == 'true'
    if updated_id_reboiler:
        updated_data['id-reboiler'] = int(updated_id_reboiler)
    if updated_datetime:
        updated_data['datetime'] = updated_datetime

    if updated_data:
        pipes_table.update(updated_data, Pipes.id == pipe_id)
    
    updated_pipe = pipes_table.get(Pipes.id == pipe_id)
    if updated_pipe:
        return jsonify(updated_pipe), 200
    return jsonify({"error": "Pipe not found"}), 404

@app.route('/pipes/<int:pipe_id>', methods=['DELETE'])
def delete_pipe(pipe_id):
    pipes_table.remove(Pipes.id == pipe_id)
    return jsonify({"message": f"Pipe {pipe_id} deleted"}), 200

@app.route('/pipes/simulate', methods=['POST'])
def simulate_pipe():
    dirty_grade = random.randint(1, 100)
    status = dirty_grade < 30
    current_datetime = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
    new_pipe = {
        'id': request.json.get('id', None),
        'status': status,
        'id-reboiler': request.json.get('id-reboiler', None),
        'datetime': current_datetime
    }
    pipes_table.insert(new_pipe)
    return jsonify(new_pipe), 201

if __name__ == '__main__':
    app.run(debug=True)
