from flask import Flask, request, jsonify
from tinydb import TinyDB, Query
from tinydb.storages import JSONStorage
from tinydb.middlewares import CachingMiddleware
import os
import random
from datetime import datetime

app = Flask(__name__)

# Configuração do caminho para o arquivo JSON do TinyDB
db_path = os.path.join(os.path.dirname(__file__), 'pipes.json')
db = TinyDB(db_path)
pipes_table = db.table('pipes')
Pipes = Query()

@app.route('/pipes', methods=['GET'])
def get_pipes():
    pipes = pipes_table.all()
    return jsonify(pipes), 200

@app.route('/pipes/<int:pipe_id>', methods=['GET'])
def get_pipe(pipe_id):
    pipe = pipes_table.get(Pipes.id == pipe_id)
    if pipe:
        return jsonify(pipe), 200
    return jsonify({"error": "Pipe not found"}), 404

@app.route('/pipes', methods=['POST'])
def add_pipe():
    new_pipe = request.json
    pipes_table.insert(new_pipe)
    return jsonify(new_pipe), 201

@app.route('/pipes/<int:pipe_id>', methods=['PUT'])
def update_pipe(pipe_id):
    updated_data = request.json
    pipes_table.update(updated_data, Pipes.id == pipe_id)
    updated_pipe = pipes_table.get(Pipes.id == pipe_id)
    if updated_pipe:
        return jsonify(updated_pipe), 200
    return jsonify({"error": "Pipe not found"}), 404

@app.route('/pipes/<int:pipe_id>', methods=['DELETE'])
def delete_pipe(pipe_id):
    pipes_table.remove(Pipes.id == pipe_id)
    return jsonify({"message": "Pipe deleted"}), 200

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

if __name__ == '__main__':
    app.run(debug=True)
