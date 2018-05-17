from flask import request
from flask_restplus import Namespace, Resource
from ..models import User, Robot, RegisterToken
from ..decorators import admin_required

from . import user_model, robot_extended_model, register_token_model

api = Namespace('admin')


@api.route('/users')
class UsersResource(Resource):
    @admin_required
    @api.marshal_with(user_model, as_list=True)
    def get(self):
        return [u for u in User.objects]

    @admin_required
    @api.expect(user_model)
    @api.marshal_with(user_model)
    def post(self):
        user = User(**request.json)
        user.activated = True
        user.save()
        return user

@api.route('/users/<int:id>')
class UserResource(Resource):
    @admin_required
    @api.marshal_with(user_model)
    def get(self, id):
        return User.objects.get_or_404(id=id)

    @admin_required
    @api.expect(user_model)
    @api.marshal_with(user_model)
    def patch(self, id):
        user = User.objects.get_or_404(id=id)
        for key, value in request.json.items():
            try:
                setattr(user, key, value)
            except AttributeError:
                pass
        user.save()
        return user

    @admin_required
    @api.marshal_with(user_model)
    def delete(self, id):
        user = User.objects.get_or_404(id=id)
        user.delete()
        return user

@api.route('/robots')
class RobotsResource(Resource):
    @admin_required
    @api.marshal_with(robot_extended_model, as_list=True)
    def get(self):
        return [r for r in Robot.objects]

@api.route('/tokens')
class AccessTokensResource(Resource):
    @admin_required
    @api.expect(register_token_model)
    @api.marshal_with(register_token_model)
    def post(self):
        token = RegisterToken(**request.json).save()
        return token

    @admin_required
    @api.marshal_with(register_token_model, as_list=True)
    def get(self):
        return [token for token in RegisterToken.objects]

@api.route('/tokens/<int:id>')
class AccessTokenResource(Resource):
    @admin_required
    @api.marshal_with(register_token_model)
    def delete(self, id):
        token = RegisterToken.objects.get_or_404(id=id)
        token.delete()
        return token



