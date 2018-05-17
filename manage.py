#!/usr/bin/env python

import os
from flask_script import Manager
from server import create_app

app = create_app(os.environ.get('CONFIG') or 'default')
manager = Manager(app)

@manager.command
def deploy():
    '''run deploy'''
    from server.models import User
    try:
        admin = User(email="admin@admin.com", password="admin", username="admin")
        admin.activated = True
        admin.roles = ['admin']
        admin.save()
        print("Admin Created")
    except:
        print("Admin Already Exists")

if __name__ == '__main__':
    manager.run()
