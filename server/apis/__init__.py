from flask_restplus import Api, fields
from ..models import User, Robot
from flask_jwt_extended import exceptions as jwt_extended_exception

api = Api(
    title='HBR Server API',
    version='1.0',
    description='API for build HBR Server',
    ui=False
)

login_model = api.model('Login', {
    'email': fields.String(required=True, description='User email'),
    'password': fields.String(required=True, description='User password')
})

register_model = api.model('Login', {
    **login_model,
    'username': fields.String(required=True),
    'token': fields.String(required=False)
})

user_activate_model = api.model('User Activate', {
    'activation_token': fields.String(required=True, description='User activation token')
})

robot_interface_model = api.model("RobotInterface", {
            'interface': fields.String(required=True),
            'ip': fields.String(required=True)
        })

robot_polling_model = api.model("RobotPolling", {
    'ips': fields.List(fields.Nested(robot_interface_model), attribute='local_addr')
})

robot_model = api.model("Robot", {
    **robot_polling_model
})

robot_extended_model = api.model("RobotExtended",{
    **robot_model,
    'addr': fields.String(attribute='global_ip')
})

user_model = api.model("UserMode", {
        'email': fields.String,
        'id': fields.Integer(required=False),
        'username': fields.String(required=False),
        '$programs': fields.String(attribute=lambda p: '/api/users/1.0/programs')
    })

register_token_model = api.model("RegisterToken", {
    'id': fields.Integer(required=False),
    'role': fields.String,
    'token': fields.String,
})

from .users import api as api_users
api.add_namespace(api_users, path="/api/users/1.0")

from .admin import api as api_admin
api.add_namespace(api_admin, path="/api/admin/1.0")

from .robots import api as api_robots
api.add_namespace(api_robots, path="/api/robots/1.0")

@api.errorhandler(jwt_extended_exception.NoAuthorizationError)
def handle_auth_error(e):
    return {'message': str(e)}, 401

