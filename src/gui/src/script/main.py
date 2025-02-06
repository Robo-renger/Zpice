import pyshine as ps
from flask import Flask, send_from_directory

app = Flask(__name__, static_folder="static")

@app.route('/static/<path:filename>')
def serve_static(filename):
    return send_from_directory('static', filename)

@app.route('/')
def index():
    return ps.stream_template('index.html')

if __name__ == "__main__":
    app.run(debug=True, port=8080)
