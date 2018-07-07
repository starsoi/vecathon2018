from flask import Flask, request
import sys
app = Flask(__name__)


new_distance, new_angle, new_fire = 0, 0, 0

@app.route('/')
def fireman():
    return 'ID = 1234; Distance = 100; Rotation = 100;fire = 1；'

@app.route('/update_cmd')
def update_car():
    global new_distance, new_angle, new_fire
    new_distance = int(request.args.get('distance', type=float))
    new_angle = request.args.get('angle', type=int)
    new_fire = request.args.get('fire', type=int)

    return ""


@app.route("/get")
def get_data():
    resp = '$'.join([str(new_distance), str(new_angle), str(new_fire)])
    resp = resp + "$"
    print(resp)
    response = app.response_class(status=200, mimetype='application/text', response=resp)
    return response


if __name__ == '__main__':

    app.run('192.168.96.1', port=8080, debug=True)