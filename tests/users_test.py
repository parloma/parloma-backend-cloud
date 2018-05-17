import pytest
from server.models import User, Program, RegisterToken
from server import mail
from server import db
from itsdangerous import URLSafeTimedSerializer
from server.decorators import user_required
from faker import Faker
from random import randrange
from flask import render_template

def test_user_registration(client):
    email = 'newmail@gmail.com'
    data = {
        'email': email,
        'password': 'newpassword',
        'username': 'username'
    }
    with mail.record_messages() as outbox:
        res = client.post('/api/users/1.0/register', json=data)
        assert res.status_code == 200

        assert len(outbox) == 1
        msg = outbox[0]
        assert msg.subject == 'HotBlack Robotics - Complete Your Registration'
        
    user = User.objects.get(email=email)
    assert user.activated == False

def test_user_registration_with_token(client):
    RegisterToken(token='AVOGADRO-18', role='premium').save()
    email = 'newmailtoken@gmail.com'
    data = {
        'email': email,
        'password': 'newpassword',
        'username': 'username',
        'token' : 'AVOGADRO-18'
    }
    res = client.post('/api/users/1.0/register', json=data)
    assert res.status_code == 200
        
    user = User.objects.get(email=email)
    assert 'premium' in user.roles

def test_user_registration_with_non_existing_token(client):
    email = 'newmailtoken@gmail.com'
    data = {
        'email': 'useremail@userm.com',
        'password': 'newpassword',
        'username': 'username2',
        'token' : 'NONEXIST-18'
    }
    res = client.post('/api/users/1.0/register', json=data)
    assert res.status_code == 200
    
def test_generate_activation_token_user(client, user, app):
    s = URLSafeTimedSerializer(app.config['SECRET_KEY'])
    token = user.generate_activation_token()
    data = s.loads(token)
    assert data['type'] == 'activate'
    assert data['id'] == user.id


def test_user_activation(client, app):
    user = User(email='tetmail@test.com', password='testpasso', username="uname")
    user.save()

    assert user.activated == False
    with app.app_context():
        data = {
            'activation_token': user.generate_activation_token()
        }
    res = client.post('/api/users/1.0/activate', json=data)
    assert res.status_code == 200
    assert 'access_token' in res.json
    
    user = User.objects.get(email='tetmail@test.com') 
    assert user.activated == True
    print(res.json['access_token'])
    with app.app_context():
        assert user == User.login(res.json['access_token'])

def test_user_not_activated_try_access(client, app):
    user = User(
        email='qwew@test.com', 
        password='testpasso',
        username="testname"
        )
    user.save()

    assert user.activated == False

    data = {
        "email": 'qwew@test.com',
        "password": 'testpasso'
    }
    res = client.post('/api/users/1.0/auth', json=data)
    assert res.status_code == 401

    @app.route('/protected')
    @user_required
    def protected_resource():
        return ''

    res = client.get('/protected')
    assert res.status_code == 401


@pytest.mark.skip()
def test_user_register_already_exists(client, user):
    user.save()
    User.ensure_indexes()

    data = {
        'email': user.email,
        'password': 'test'
    }
    res = client.post('/api/users/1.0/register', json=data)
    assert res.status_code == 401

def test_login(user, email, password):
    assert user == User.login(email, password)
    assert User.login(email, 'errors') is None

    token = user.create_token()

    assert user == User.login(token)

def test_user_auth(client, email, password, user):
    data = {
        'dd': email,
        'password': password
    }
    res = client.post('/api/users/1.0/auth', json=data)
    assert res.status_code == 400

    data = {
        'email': email,
        'password': password
    }
    res = client.post('/api/users/1.0/auth', json=data)
    assert res.status_code == 200
    assert res.json['access_token'] is not None
    assert res.json['user_data'] is not None

    data = {
        'email': email,
        'password': 'error'
    }
    res = client.post('/api/users/1.0/auth', json=data)

    assert res.status_code == 400
    assert res.content_type == 'application/json'

def test_user_access_protected_endpoint(user, client):
    token = user.create_token()
    res = client.get('/api/users/1.0/')
    assert res.status_code == 401

    p = Program(name="test")
    user.programs.append(p)
    user.save()

    assert p.code == render_template('node.py')

    res = client.get('/api/users/1.0/', headers={'Authorization': 'Bearer {}'.format(token)})
    assert res.status_code == 200
    assert res.json['email'] == user.email

def test_user_program(user, client):

    user.programs = []
    user.programs.append(Program(name="test1"))
    user.programs.append(Program(name="test2"))
    user.save()
    assert len(user.programs) == 2
    token = user.create_token()
    res = client.get('/api/users/1.0/', headers={'Authorization': 'Bearer {}'.format(token)})
    assert res.status_code == 200
    assert res.json['$programs'] == '/api/users/1.0/programs'

    res = client.get(res.json['$programs'], headers={'Authorization': 'Bearer {}'.format(token)})
    assert res.status_code == 200
    assert len(res.json) == len(user.programs)
    for program in res.json:
        assert 'id' in program
        assert 'code' not in program

def test_user_programs(user, client):
    user.programs = []
    user.programs.append(Program(name="test1"))
    user.programs.append(Program(name="test2"))
    user.save()
    assert len(user.programs) == 2

    res = client.get("/api/users/1.0/programs", headers={'Authorization': 'Bearer {}'.format(user.create_token())})
    for program in user.programs:
        res = client.get('/api/users/1.0/programs/{}'.format(program.id), headers={'Authorization': 'Bearer {}'.format(user.create_token())})
        assert res.status_code == 200
        assert res.json['name'] in ['test1', 'test2']
        assert 'code' in res.json
        res = client.get('/api/users/1.0/programs/123', headers={'Authorization': 'Bearer {}'.format(user.create_token())})
        assert res.status_code == 404

def test_program_api(user, client):
    user.programs = []
    user.save()

    data = {
        'name': 'test',
        'code': 'hey',
        'language': 'python'
    }

    res = client.post("/api/users/1.0/programs", json=data, headers={'Authorization': 'Bearer {}'.format(user.create_token())})
    assert res.status_code == 200
    user = User.objects.get(id=user.id)
    assert len(user.programs) == 1
    assert user.programs[0].name == data['name']
    assert user.programs[0].code == data['code']
    assert user.programs[0].language == data['language']
    assert user.programs[0].created_at is not None
    assert user.programs[0].last_edit is not None
    assert user.programs[0].id == res.json['id']

    id = res.json['id']
    
    last_edit_old = user.programs[0].last_edit
    res = client.patch("/api/users/1.0/programs/{}".format(id), json=data, headers={'Authorization': 'Bearer {}'.format(user.create_token())})
    assert res.status_code == 200
    user = User.objects.get(id=user.id)
    assert user.programs[0].code == data['code']
    assert user.programs[0].last_edit != last_edit_old
    
    
    res = client.delete("/api/users/1.0/programs/{}".format(id), headers={'Authorization': 'Bearer {}'.format(user.create_token())})
    user = User.objects.get(id=user.id)
    assert res.status_code == 200
    assert len(user.programs) == 0

def test_multiple_user(client, app):
    fake = Faker()
    
    for _ in range(2):
        u = User(
            email=fake.company_email(), 
            password=fake.password(), 
            username = fake.name(),
            activated=True)
        u.save()

        for _ in range(randrange(1, 10)):
            p = Program(name=fake.name(), code=fake.text(), language="python")
            u.programs.append(p)

        u.save()

    for user in User.objects:
        with app.app_context():
            token = user.create_token()
        res = client.get('/api/users/1.0/programs', headers={'Authorization': 'Bearer {}'.format(token)})
        assert res.status_code == 200
        assert len(user.programs) == len(res.json)

    user1 = User.objects[0]
    user2 = User.objects[1]
    
    with app.app_context():
        token = user1.create_token()

    res = client.get('/api/users/1.0/programs/{}'.format(user2.programs[0].id), headers={'Authorization': 'Bearer {}'.format(token)})
    assert res.status_code == 404
