from server.models import Program, User
from datetime import datetime, timedelta

from flask_jwt_extended import decode_token, create_access_token
from freezegun import freeze_time
from jwt import ExpiredSignatureError
import pytest

from mongoengine.errors import NotUniqueError

def test_app_running(client):
    res = client.get('/')
    assert res.status_code is not None

@pytest.mark.skip("Bug in MongoMock")
def test_multiple_email(app, db):
    with pytest.raises(NotUniqueError):
        user1 = User(email="test@test.com", password="", activared=True)
        user1.save()
        user2 = User(email="test@test.com", password="asd", activared=True)
        user2.save()

def test_user_db(user):
    assert user is not None
    assert user.password_hash is not None

def test_user_compute_password_hash(user, password):
    assert user.password_hash != password

def test_user_check_password(user, password):
    assert user.check_password('error') == False
    assert user.check_password(password) == True

def test_user_programs(user):
    p1 = Program(name="test", code="for i in range()\n    test", language="python")
    user.programs.append(p1)
    user.save()
    p = Program(name="test 3", code="for i in range()\n    test", language="python")
    user.programs.append(p)
    user.save()

    assert p1.id != p.id
    assert len(user.programs) == 2
    assert user.programs[0].created_at > datetime.utcnow() - timedelta(seconds=5)
    assert user.programs[0].created_at < datetime.utcnow()

def test_user_change_password(user, password):
    newpass = 'nuova password'
    user.password = newpass
    user.save()

    assert user.check_password(password) == False
    assert user.check_password(newpass) == True

def test_token_expiration(app):
    with app.app_context():
        with freeze_time("2012-01-14"):
            token = create_access_token(identity={})
        with freeze_time("2012-01-15"):
            try:
                decode_token(token)
            except ExpiredSignatureError as e:
                pytest.fail('DID RAISE {}'.format(e))

        with freeze_time("2012-01-18"):
            with pytest.raises(ExpiredSignatureError):
                decode_token(token)
