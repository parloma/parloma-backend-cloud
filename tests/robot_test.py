from server.models import Robot, LocalAddr
import uuid
from datetime import datetime, timedelta
from flask_jwt_extended import decode_token, get_jwt_identity
import pytest

def test_robot_exist(db):
    r = Robot(rid=str(uuid.uuid4()))
    r.save()
    assert r.first_logged is not None
    assert r.id is not None

def test_robot_get(robot):
    robot.global_ip = "192.168.0.32"
    robot.save()

    assert len(Robot.get_connected_by_ip("192.168.0.32")) == 1
    assert len(Robot.get_connected_by_ip("10.0.0.13")) == 0

    robot.last_logged = datetime.utcnow() - timedelta(minutes=10)
    robot.save()
    assert len(Robot.get_connected_by_ip("192.168.0.32")) == 0
    assert len(Robot.get_connected_by_ip("10.0.0.13")) == 0

def test_robot_locate_api(user, robot, client):
    token = user.create_token()
    robot.global_ip='127.0.0.1'
    robot.local_addr = [
       LocalAddr(ip='192.168.0.12', interface='eth0'),
       LocalAddr(ip='10.21.0.2', interface='wlan0')
    ]
    robot.save()

    res = client.get('/api/users/1.0/robots', headers={'Authorization': 'Bearer {}'.format(token)})
    assert res.status_code == 200
    assert len(res.json) == 1
    print(res.json)

    assert len(res.json[0]["ips"]) == len(robot.local_addr)

    robot.global_ip='217.0.0.2'
    robot.save()
    res = client.get('/api/users/1.0/robots', headers={'Authorization': 'Bearer {}'.format(token)})
    assert res.status_code == 200
    assert len(res.json) == 0

def test_robot_token(robot):
    token = robot.create_token()
    assert token is not None
    identity = decode_token(token)["identity"]
    assert identity['type'] == 'robot'
    assert identity['id'] == robot.id

def test_robot_load_from_token(robot):
    token = robot.create_token()
    identity = decode_token(token)["identity"]
    assert robot == Robot.from_identity(identity)

def test_robot_login(client):
    rid = str(uuid.uuid4())
    data = {
        'rid': rid
        }
    res = client.post('/api/robots/1.0/auth', json = data)
    assert res.status_code == 200
    assert 'access_token' in res.json
    print(res.json)
    token = res.json['access_token']

    robot = Robot.objects.get(rid = rid)
    assert robot is not None

@pytest.fixture
def robot_id():
    return str(uuid.uuid4())

@pytest.fixture
def robot_access_token(client, robot_id):
    rid = robot_id
    data = {
        'rid': rid
        }
    res = client.post('/api/robots/1.0/auth', json = data)
    return res.json['access_token']

def test_robot_no_login_error(client):
    data = {
        "ips": [
            {
                'interface': 'eth0',
                'ip': '101.232.12.2'
            }
        ]
    }

    res = client.post('/api/robots/1.0/poll', json = data)
    print(res.json)
    assert res.status_code == 401

def test_robot_poll(client, robot_access_token, robot_id):
    rid = robot_id

    data = {
        "ips": [
            {
                'interface': 'eth0',
                'ip': '101.232.12.2'
            }
        ]
    }

    headers = {'Authorization': 'Bearer {}'.format(robot_access_token)}
    res = client.post('/api/robots/1.0/poll', json = data, headers=headers)
    assert res.status_code == 200

    robot = Robot.objects.get(rid = rid)

    for addr, addr_data in zip(robot.local_addr, data['ips']):
        assert addr.interface == addr_data['interface']
        assert addr.ip == addr_data['ip']

def test_robot_save_ip_global(robot, client):
    robot.global_ip = ''
    robot.save()
    headers = {'Authorization': 'Bearer {}'.format(robot.create_token())}

    res = client.post('/api/robots/1.0/poll', json={'ips':[]}, headers=headers)
    assert res.status_code == 200
    robot = Robot.objects.get(id=robot.id)
    assert robot.global_ip == '127.0.0.1'

def test_robot_multiple_login(client):

    len_robots = len(Robot.objects)

    rid = str(uuid.uuid4())
    data = {
        'rid': rid
        }
    res = client.post('/api/robots/1.0/auth', json = data)
    assert res.status_code == 200
    assert len_robots < len(Robot.objects)
    len_robots = len(Robot.objects)
    res = client.post('/api/robots/1.0/auth', json = data)
    assert len_robots == len(Robot.objects)


def test_user_not_access_robot(client, user):
    res = client.post('/api/robots/1.0/poll', json = {'ips':[]}, headers = {'Authorization': 'Bearer {}'.format(user.create_token())})
    assert res.status_code == 401

def test_robot_not_access_user(client, robot):
    res = client.get('/api/users/1.0/robots', headers = {'Authorization': 'Bearer {}'.format(robot.create_token())})
    assert res.status_code == 401

def test_robot_dont_add_interface(client, robot):
    res = client.post('/api/robots/1.0/poll', json = {'ips':[{'interface': 'wlan0', 'ip': ''}]}, headers = {'Authorization': 'Bearer {}'.format(robot.create_token())})
    robot = Robot.objects.get(id = robot.id)
    len_interface = len(robot.local_addr)

    res = client.post('/api/robots/1.0/poll', json = {'ips':[{'interface': 'wlan0', 'ip': ''}]}, headers = {'Authorization': 'Bearer {}'.format(robot.create_token())})
    robot = Robot.objects.get(id = robot.id)
    assert len_interface == len(robot.local_addr)

def test_robot_poll_update_timestamp(client, robot):
    last_logged = robot.last_logged
    res = client.post('/api/robots/1.0/poll', json = {'ips':[]}, headers = {'Authorization': 'Bearer {}'.format(robot.create_token())})
    robot = Robot.objects.get(id = robot.id)
    assert last_logged < robot.last_logged
