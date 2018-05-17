import os
from datetime import timedelta

class Config(object):
    RESTPLUS_VALIDATE = True

    JWT_ACCESS_TOKEN_EXPIRES = timedelta(days=3)
    MAIL_DEFAULT_SENDER = 'no-reply@parloma.com'
    MAIN_SITE_URL = 'crf.parloma.com'

    MONGODB_SETTINGS = {
        'db': os.environ.get('MONGODB_NAME') or 'default',
        'host': os.environ.get('MONGODB_URL') or 'mongomock://test'
    }


class TestConfig(Config):
    SECRET_KEY = 'test-super-secret'
    TESTING = True
    MONGODB_SETTINGS = {
        'db': 'test',
        'host': 'mongomock://test'
    }

class DockerConfig(Config):
    MAIL_SERVER = 'smtpout.secureserver.net'
    MAIL_PORT = 465
    MAIL_USE_TLS = False
    MAIL_USE_SSL = True
    MAIL_USERNAME = os.environ.get('MAIL_USERNAME', 'info@test.com')
    MAIL_PASSWORD = os.environ.get('MAIL_PASSWORD', 'test')

    SECRET_KEY = os.environ.get('SECRET_KEY', 'test-super-secret')
    
    MONGODB_SETTINGS = {
        'db':  os.environ.get('MONGO_DB'),
        'host': os.environ.get('MONGO_URL')
    }


configs = {
    'default': Config,
    'testing': TestConfig,
    'docker': DockerConfig
}
