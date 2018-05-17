from flask_jwt_extended import jwt_required, exceptions
from functools import wraps
from .models import User, Robot
from flask import request

from flask_jwt_extended.exceptions import  UserClaimsVerificationError
from flask_jwt_extended.utils import verify_token_claims
from flask_jwt_extended.view_decorators import _decode_jwt_from_request, _load_user


EXEMPT_METHODS = ['OPTIONS']

def model_required(cls):
    def wrapper_decorator(f):
        @wraps(f)
        @jwt_required
        def wrapper(*args, **kargs):
            if request.method in EXEMPT_METHODS:
                return f(*args, **kargs)
            model = cls.current_logged()
            if model is not None:
                return f(*args, **kargs)
            raise(exceptions.NoAuthorizationError)
        return wrapper
    return wrapper_decorator

robot_required = model_required(Robot)

def user_required(f):
    @wraps(f)
    @model_required(User)
    def wrapper(*args, **kargs):
        if not User.current_logged().activated:
            raise(exceptions.NoAuthorizationError)
        else:
            return f(*args, **kargs)
    return wrapper

def admin_required(f):
    @wraps(f)
    @user_required
    def wrapper(*args, **kargs):
        if request.method in EXEMPT_METHODS:
            return f(*args, **kargs)
        admin = User.current_logged()
        if 'admin' in admin.roles:
            return f(*args, **kargs)
        raise(exceptions.NoAuthorizationError)
    return wrapper