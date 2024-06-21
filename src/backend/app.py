from flask import Flask, request, jsonify
from flask_cors import CORS
from tinydb import TinyDB, Query
import os
import random
from datetime import datetime
import cv2
import numpy as np
from ultralytics import YOLO

app = Flask(__name__)
CORS(app)  # Habilita CORS para todas as rotas

db_path = os.path.join(os.path.dirname(__file__), 'pipes.json')
db = TinyDB(db_path)
pipes_table = db.table('pipes')
Pipes = Query()

# Inicializa o modelo YOLO
model = YOLO("src/backend/home/grupo-05-t08/2024-1B-T08-EC06-G05/src/backend/runs/detect/train4/weights/best.pt")

updated_data = {
    'id': None,
    'status': None,
    'id-boiler': None,
    'dirty-grade': None,
    'datetime': None
}

@app.route('/post_reboiler_id', methods=['POST', 'GET'])
def post_reboiler_id():
    data = request.get_json()
    reboiler_id = data.get('reboilerID')
    updated_data['id-boiler'] = reboiler_id
    print(f"Id recebido para o reboiler atual: {updated_data['id-boiler']}")
    return jsonify({"reboilerID": reboiler_id}), 200

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
    updated_id_boiler = request.args.get('id-boiler')
    updated_dirty_grade = request.args.get('dirty-grade')
    updated_datetime = request.args.get('datetime')

    if updated_id:
        updated_data['id'] = int(updated_id)
    if updated_status:
        updated_data['status'] = updated_status.lower() == 'true'
    if updated_id_boiler:
        updated_data['id-boiler'] = int(updated_id_boiler)
    if updated_dirty_grade:
        updated_data['dirty-grade'] = int(updated_dirty_grade)
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
        'id-boiler': request.json.get('id-boiler', None),
        'dirty-grade': dirty_grade,
        'datetime': current_datetime
    }
    pipes_table.insert(new_pipe)
    return jsonify(new_pipe), 201

@app.route('/process_image', methods=['GET'])
def process_image():
    imagem = 2
    print("imagem")

    # if 'image' not in request.files:
    #    return jsonify({'error': 'No image provided'}), 400

    # file = request.files['image']
    # video = cv2.imdecode(np.frombuffer(file.read(), np.uint8), cv2.IMREAD_COLOR)

    # if image is None:
    #   return jsonify({'error': 'Unable to load image'}), 400

    # img = "src/data-base/imgs/img1.jpeg"
    # print (img)
    # # Processa a imagem com o modelo YOLO
    # results = model(img)
    # is_dirty = False

    # for result in results:
    #     img = result.plot()
    #     # Verifique se há detecção de sujeira
    #     for box in result.boxes:
    #         if box.cls == "sujeira":  
    #             is_dirty = True

    # # Define o diretório de saída para salvar as imagens processadas
    # output_dir = "src/data-base/imgs-results"
    # os.makedirs(output_dir, exist_ok=True)
    # output_path = os.path.join(output_dir, "processed_frame.png")
    # cv2.imwrite(output_path, video)

    # status = "sujo" if is_dirty else "limpo"
    # return jsonify({'status': status, 'image_path': output_path})


if __name__ == '__main__':
    app.run(debug=True)
