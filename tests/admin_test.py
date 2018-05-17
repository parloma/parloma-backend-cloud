import pytest

from server.models import User, Robot, RegisterToken
from flask_jwt_extended import decode_token
from mongoengine import DoesNotExist
import uuid

@pytest.fixture
def admin(app, db):
    a = User(
        email="admin@admin.com", password="admin", 
        username="admin",
        activated=True)
    a.roles.append("admin")
    a.save()
    return a


def test_admin_role(admin):
    assert "admin" in admin.roles

def test_admin_access_admin_enpoint(admin, user, client):
    res = client.get('/api/admin/1.0/users', headers={'Authorization': 'Bearer {}'.format(admin.create_token())})
    assert res.status_code == 200
    res = client.get('/api/admin/1.0/users', headers={'Authorization': 'Bearer {}'.format(user.create_token())})
    assert res.status_code == 401


def test_admin_user_api(admin, client, user):
    user = User.objects
    luser = len(user)

    res = client.get('/api/admin/1.0/users', headers={'Authorization' : 'Bearer {}'.format(admin.create_token())} )
    assert len(res.json) == luser

    newuserinfo = {
        'email': 'new@user.com',
        'password': 'mypass',
        'username': 'username'
    }

    res = client.post('/api/admin/1.0/users', json=newuserinfo, headers={'Authorization' : 'Bearer {}'.format(admin.create_token())} )
    assert res.status_code == 200
    newuser = User.objects.get(id=res.json["id"])
    assert newuser.email == newuserinfo['email']
    assert newuser.check_password(newuserinfo['password'])
    assert newuser.activated == True

    res = client.get('/api/admin/1.0/users/{}'.format(newuser.id), json=newuserinfo, headers={'Authorization' : 'Bearer {}'.format(admin.create_token())} )
    assert res.status_code == 200
    assert res.json['email'] == newuser.email

    newuserinfo = {
        'password': 'newmypass'
    }

    res = client.patch('/api/admin/1.0/users/{}'.format(newuser.id), json=newuserinfo, headers={'Authorization' : 'Bearer {}'.format(admin.create_token())} )
    assert res.status_code == 200
    newuser = User.objects.get(id=res.json["id"])
    assert newuser.check_password(newuserinfo['password'])

    res = client.delete('/api/admin/1.0/users/{}'.format(newuser.id), headers={'Authorization' : 'Bearer {}'.format(admin.create_token())} )
    assert res.status_code == 200

    with pytest.raises(DoesNotExist):
        newuser = User.objects.get(id=res.json["id"])

def test_admin_auth(client, user): 
    res = client.options('/api/admin/1.0/users/{}'.format(user.id))
    assert res.status_code == 200
    assert client.delete('/api/admin/1.0/users/{}'.format(user.id)).status_code == 401
    res = client.get('/api/admin/1.0/users/{}'.format(user.id))
    assert res.status_code == 401

    assert client.patch('/api/admin/1.0/users/{}'.format(user.id), json={}).status_code == 401

    assert client.get('/api/admin/1.0/users').status_code == 401
    assert client.post('/api/admin/1.0/users', json={}).status_code == 401
    assert client.options('/api/admin/1.0/users').status_code == 200


def test_admin_robot_api(admin, client):
    for _ in range(5):
        Robot(rid=str(uuid.uuid4()), global_ip='12.34.32.23').save()
    robots = Robot.objects
    res = client.get('/api/admin/1.0/robots', headers={'Authorization': 'Bearer {}'.format(admin.create_token())} )
    assert res.status_code == 200
    assert len(res.json) == len(robots)

    rob_model = res.json[0]
    assert 'ips' in rob_model
    assert 'addr' in rob_model

def test_register_token_api(admin,client):
    tokens = RegisterToken.objects
    assert len(tokens) == 0

    data = {
        'token': 'TEST-18',
        'role': 'test'
    }
    res = client.post('/api/admin/1.0/tokens', json=data ,headers={'Authorization': 'Bearer {}'.format(admin.create_token())})
    assert res.status_code == 200
    print (RegisterToken.objects[0].token)
    token = RegisterToken.objects.get(token='TEST-18')
    assert token.role == 'test'

    res = client.get('/api/admin/1.0/tokens', headers={'Authorization': 'Bearer {}'.format(admin.create_token())})
    assert res.status_code == 200
    assert len(res.json) == 1
    assert res.json[0]['role'] == 'test'
    assert res.json[0]['token'] == 'TEST-18'
    assert 'id' in res.json[0]

    res = client.delete('/api/admin/1.0/tokens/{}'.format(token.id), headers={'Authorization': 'Bearer {}'.format(admin.create_token())})
    assert res.status_code == 200
    assert len(RegisterToken.objects) == 0
