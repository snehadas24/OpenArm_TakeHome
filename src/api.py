from flask import Flask, jsonify, send_file, abort
import os
import numpy as np

app = Flask(__name__)

BASE = os.path.join(os.path.dirname(__file__), "..", "data", "episodes")
BASE = os.path.abspath(BASE)

@app.route('/episodes', methods=['GET'])
def list_episodes():
    if not os.path.isdir(BASE):
        return jsonify([])
    files = sorted([f for f in os.listdir(BASE) if f.endswith('.npz')])
    meta = []
    for f in files:
        path = os.path.join(BASE, f)
        try:
            with np.load(path, allow_pickle=True) as d:
                n = len(d['ts']) if 'ts' in d else None
        except Exception:
            n = None
        meta.append({'file': f, 'samples': n})
    return jsonify(meta)

@app.route('/episodes/<name>', methods=['GET'])
def get_episode(name):
    path = os.path.join(BASE, name)
    if not os.path.isfile(path):
        abort(404)
    return send_file(path, as_attachment=True)

@app.route('/episodes/<name>/meta', methods=['GET'])
def episode_meta(name):
    path = os.path.join(BASE, name)
    if not os.path.isfile(path):
        abort(404)
    try:
        with np.load(path, allow_pickle=True) as d:
            meta = {k: v.shape for k, v in d.items()}
    except Exception as e:
        abort(500, str(e))
    return jsonify({k: (v.tolist() if hasattr(v, 'tolist') else str(v)) for k, v in meta.items()})

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5001)
