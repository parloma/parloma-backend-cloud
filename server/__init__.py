from flask import Flask
from flask_mongoengine import MongoEngine
from flask_jwt_extended import JWTManager
from .configs import configs
from flask_cors import CORS
from flask_mail import Mail

app = Flask(__name__)
db = MongoEngine()
mail = Mail()

def create_app(confname='default'):
    app = Flask(__name__)
    app.config.from_object(configs[confname])

    db.init_app(app)
    mail.init_app(app)

    JWTManager(app)
    CORS(app)

    from .apis import api
    api.init_app(app)

    return app
