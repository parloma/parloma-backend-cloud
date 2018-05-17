from flask import request, render_template, current_app
from flask_restplus import Api, Resource, fields, Namespace
from ..models import User, Program, Robot, RegisterToken
from flask_jwt_extended import exceptions as jwt_extended_exception
from ..decorators import user_required
from datetime import datetime
from .. import mail
from flask_mail import Message
from mongoengine.errors import NotUniqueError, DoesNotExist

from . import user_model, user_activate_model

api = Namespace('users', description='Users Operation')

program_model = api.model('Program', Program.model)
program_info_model = api.model('Program Info', Program.info_model)

from . import login_model, robot_model, register_model

@api.errorhandler(jwt_extended_exception.NoAuthorizationError)
def handle_auth_error(e):
    return {'message': str(e)}, 401


@api.route('/auth')
class AuthResource(Resource):
    @api.expect(login_model)
    def post(self):
        data = request.json
        user = User.login(data['email'], data['password'])
        if user is None:
            return {'msg': 'login error'}, 400
        if not user.activated:
            return {'error': 'confirm mail required'}, 401
        
        return {
            'access_token': user.create_token(),
            'user_data': user.get_data()
            }


@api.route('/register')
class RegisterResource(Resource):
    @api.expect(register_model)
    def post(self):
        data = request.json
        roles = []
        if 'token' in request.json:
            token = request.json['token']
            request.json.pop('token', None)
            try:
                token = RegisterToken.objects.get(token=token)
                if token:
                    roles.append(token.role)
            except DoesNotExist:
                pass
        try:
            user = User(**request.json, roles=roles)
            user.save()
        except NotUniqueError:
            return {'error': 'email already registered'}, 401
        msg = Message('HotBlack Robotics - Complete Your Registration', 
                recipients=[user.email],
                cc=['info@hotblackrobotics.com']
            )
        msg.body = render_template('mail_registration.txt', mainsite=current_app.config["MAIN_SITE_URL"], token=user.generate_activation_token())
        mail.send(msg)
        return {}, 200

@api.route('/activate')
class ConfirmUserResource(Resource):
    @api.expect(user_activate_model)
    def post(self):
        user = User.activate(request.json['activation_token'])
        return {
            'access_token': user.create_token()
        }, 200

@api.route('/')
class MainResource(Resource):
    @user_required
    @api.marshal_with(user_model)
    def get(self):
        return User.current_logged()


@api.route('/programs')
class ProgramsResource(Resource):
    @user_required
    @api.marshal_with(program_info_model, as_list=True)
    def get(self):
        user = User.current_logged()
        return user.programs

    @api.marshal_with(program_model)
    @api.expect(program_model)
    @user_required
    def post(self):
        user = User.current_logged()
        program = Program(**request.json)
        user.programs.append(program)
        user.save()
        return program


@api.route('/programs/<int:id>')
class ProgramResource(Resource):
    @user_required
    @api.marshal_with(program_model)
    def get(self, id):
        user = User.current_logged()
        try:
            return user.programs.get(id=id)
        except:
            return {}, 404

    @user_required
    @api.expect(program_model)
    @api.marshal_with(program_model)
    def patch(self, id):
        user = User.current_logged()
        program = user.programs.get(id=id)
        if program is None:
            return {}, 404
        for key, value in request.json.items():
            try:
                setattr(program, key, value)
            except AttributeError:
                pass
        program.last_edit = datetime.utcnow()
        user.save()
        return program

    @user_required
    @api.marshal_with(program_model)
    def delete(self, id):
        user = User.current_logged()
        program = user.programs.get(id=id)
        if program is None:
            return {}, 404
        user.programs.remove(program)
        user.save()
        return program


@api.route('/robots')
class GetRobotResource(Resource):
    @user_required
    @api.marshal_with(robot_model)
    def get(self):
        # robots = [r for r in Robot.get_connected_by_ip(request.remote_addr)]
        robots = [r for r in Robot]
        return robots