from . import db
from datetime import datetime, timedelta

from werkzeug.security import generate_password_hash, check_password_hash
from flask_jwt_extended import create_access_token, decode_token, get_jwt_identity
from flask import abort, current_app, render_template
from flask_restplus import fields
from mongoengine.queryset.visitor import Q
from itsdangerous import URLSafeTimedSerializer
import json

def datetime_from_bson(field):
    return datetime.fromtimestamp(field["$date"]/1e3)

class LoginDocument(db.Document):
    meta = {
        'abstract': True,
    }
    def create_token(self, **args):
        identity = {
            'type': type(self).__name__.lower(),
            'id': self.id,
            **args
        }
        return create_access_token(identity=identity)

    @classmethod
    def current_logged(cls):
        return cls.from_identity(get_jwt_identity())

    @classmethod
    def from_identity(cls,identity):
        try:
            if identity['type'] == cls.__name__.lower():
                try:
                    return cls.objects(id = identity['id'])[0]
                except:
                    return None
            else:
                return None
        except:
            return None

def default_node():
    return '''from dotbot_ros import DotbotNode
import rospy
from std_msgs.msg import String
import time

class Node(DotbotNode):
    node_name='node'
    
    def setup(self):
        self.publisher = rospy.Publisher('/chatter', String)
        self.cnt = 0

    def loop(self):
        msg = String()
        msg.data = "Hey, cnt = {}".format(self.cnt)
        self.publisher.publish(msg)
        self.cnt += 1
        rospy.loginfo(msg.data)
        time.sleep(1)'''

class Program(db.EmbeddedDocument):
    code = db.StringField(default=default_node)
    name = db.StringField(max_length=30)
    language = db.StringField(max_length=30)
    created_at = db.DateTimeField(default=datetime.utcnow)
    last_edit = db.DateTimeField(default=datetime.utcnow)
    id = db.SequenceField(primary_key=True)

    info_model = {
        'name': fields.String,
        'language': fields.String,
        'created_at': fields.DateTime,
        'last_edit': fields.DateTime,
        'id': fields.Integer
    }

    model = {
        **info_model,
        'code': fields.String,
    }

class RegisterToken(db.Document):
    id = db.SequenceField(primary_key=True)
    token = db.StringField(required=True, unique=True, max_length=30)
    role = db.StringField(required=True, max_length=30)

    
class User(LoginDocument):
    id = db.SequenceField(primary_key=True)
    username = db.StringField(required=True, unique=True)
    email = db.EmailField(required=True, unique=True)
    password_hash = db.StringField(required=True)
    programs = db.EmbeddedDocumentListField(Program)
    roles = db.ListField(db.StringField())
    activated = db.BooleanField(default=False)

    def __init__(self, **kargs):
        if 'password' in kargs:
            password = kargs['password']
            kargs.pop('password', None)
            kargs['password_hash'] = generate_password_hash(password)
            
        super(User, self).__init__(**kargs)

    @property
    def password(self):
        raise(ValueError)

    @password.setter
    def password(self, password):
        self.password_hash = generate_password_hash(password)

    def __str__(self):
        return '<u:{}>'.format(self.email, self.password_hash)

    def check_password(self, password):
        return check_password_hash(self.password_hash, password)

    def generate_activation_token(self):
        s = URLSafeTimedSerializer(current_app.config['SECRET_KEY'])
        return s.dumps({
            'type': 'activate',
            'id': self.id
        })

    @staticmethod
    def activate(token):
        s = URLSafeTimedSerializer(current_app.config['SECRET_KEY'])
        try:
            data = s.loads(token)
        except:
            abort(404)
        if data['type'] == 'activate':
            user = User.objects.get(id=data['id'])
            user.activated = True
            user.save()
            return user
        abort(404)
            
    def get_data(self):
        return {
            'username': self.username,
            'roles': self.roles
        }

    @staticmethod
    def login(email_or_token, password=None):
        if password:
            email = email_or_token
            try:
                u = User.objects(email=email)[0]
            except:
                return None
            if u.check_password(password):
                return u
            else:
                return None
        else:
            token = email_or_token
            identity = decode_token(token)["identity"]
            return User.from_identity(identity)

class LocalAddr(db.EmbeddedDocument):
    ip = db.StringField()
    interface = db.StringField(max_length=30)


class Robot(LoginDocument):
    id = db.SequenceField(primary_key=True)
    rid = db.StringField(unique=True)
    first_logged = db.DateTimeField(default=datetime.utcnow)
    last_logged = db.DateTimeField(default=datetime.utcnow)
    local_addr = db.EmbeddedDocumentListField(LocalAddr)
    global_ip = db.StringField()

    model = {
        
    }

    extended_model = {
        **model,
        'addr': fields.String(attribute='global_ip')
    }

    @staticmethod
    def get_connected_by_ip(ip):
        q1 = Q(last_logged__gte=datetime.utcnow() - timedelta(minutes=1))
        q2 = Q(global_ip = ip)
        robots = Robot.objects(q1 & q2)
        return robots
