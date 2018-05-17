from flask_restplus import Resource, fields, Namespace
from flask import request
from ..models import Robot, LocalAddr
from ..decorators import robot_required
from flask_jwt_extended import exceptions as jwt_extended_exception
from datetime import datetime
api = Namespace('robots')

from . import robot_polling_model

auth_model = api.model('Auth', {
    'rid': fields.String(required=True, description='Robot RID')
})


@api.route('/auth')
class AuthResource(Resource):
    @api.expect(auth_model)
    def post(self):
        rid = request.json['rid']
        try:
            robot = Robot.objects.get(rid=rid)
        except:
            robot = Robot(rid=rid)
            robot.save()
        return {'access_token': robot.create_token()}


@api.route('/poll')
class PollResource(Resource):
    @robot_required
    @api.expect(robot_polling_model)
    def post(self):
        robot = Robot.current_logged()
        robot.local_addr = []
        for address in request.json['ips']:
            la = LocalAddr(interface=address['interface'], ip = address['ip'])
            robot.local_addr.append(la)
        robot.global_ip = request.remote_addr
        robot.last_logged = datetime.utcnow()
        robot.save()
        return {}
