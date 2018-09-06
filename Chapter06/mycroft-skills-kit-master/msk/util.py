# Copyright (c) 2018 Mycroft AI, Inc.
#
# This file is part of Mycroft Light
# (see https://github.com/MatthewScholefield/mycroft-light).
#
# Licensed to the Apache Software Foundation (ASF) under one
# or more contributor license agreements.  See the NOTICE file
# distributed with this work for additional information
# regarding copyright ownership.  The ASF licenses this file
# to you under the Apache License, Version 2.0 (the
# "License"); you may not use this file except in compliance
# with the License.  You may obtain a copy of the License at
#
#   http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing,
# software distributed under the License is distributed on an
# "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY
# KIND, either express or implied.  See the License for the
# specific language governing permissions and limitations
# under the License.
import atexit

import os
from contextlib import contextmanager
from difflib import SequenceMatcher
from functools import wraps
from getpass import getpass
from github import Github, GithubException
from github.Repository import Repository
from msm import SkillEntry
from os import chmod
from os.path import join
from tempfile import mkstemp
from typing import Optional

from msk import __version__
from msk.exceptions import PRModified, MskException

ASKPASS = '''#!/usr/bin/env python3
import sys
print(
    "{password}"
    if sys.argv[1] == "Password for 'https://{username}@github.com': " else
    "{username}"
)'''

skills_kit_footer = '<sub>Created with [mycroft-skills-kit]({}) v{}</sub>'.format(
    'https://github.com/mycroftai/mycroft-skills-kit', __version__
)


def register_git_injector(username, password):
    """Generate a script that writes the password to the git command line tool"""
    fd, tmp_path = mkstemp()
    atexit.register(lambda: os.remove(tmp_path))

    with os.fdopen(fd, 'w') as f:
        f.write(ASKPASS.format(username=username, password=password or ''))

    chmod(tmp_path, 0o700)
    os.environ['GIT_ASKPASS'] = tmp_path


def ask_for_github_credentials(use_token=False) -> Github:
    print('=== GitHub Credentials ===')
    while True:
        if use_token:
            username = getpass('Token: ')
            password = None
        else:
            username = input('Username: ')
            password = getpass('Password: ')
        github = Github(username, password)
        try:
            _ = github.get_user().login
            register_git_injector(username, password)
            return github
        except GithubException:
            print('Login incorrect. Retry:')


def skill_repo_name(url: str):
    return '{}/{}'.format(SkillEntry.extract_author(url), SkillEntry.extract_repo_name(url))


def ask_input(message: str, validator=lambda x: True, on_fail='Invalid entry'):
    while True:
        resp = input(message + ' ').strip()
        try:
            if validator(resp):
                return resp
        except Exception:
            pass
        o = on_fail(resp) if callable(on_fail) else on_fail
        if isinstance(o, str):
            print(o)


def ask_choice(message: str, choices: list, allow_empty=False, on_empty=None) -> Optional[str]:
    if not choices:
        if allow_empty:
            print(on_empty)
            return None
        else:
            raise MskException(on_empty or 'Error with "{}"'.format(message))

    print()
    print(message)
    print('\n'.join(
        '{}. {}'.format(i + 1, choice)
        for i, choice in enumerate(choices)
    ))
    print()

    def find_match(x):
        if not x and allow_empty:
            return ...
        try:
            return choices[int(x) - 1]
        except (ValueError, IndexError):
            pass

        def calc_conf(y):
            return SequenceMatcher(a=x, b=y).ratio()
        best_choice = max(choices, key=calc_conf)
        best_conf = calc_conf(best_choice)
        if best_conf > 0.8:
            return best_choice
        raise ValueError

    resp = find_match(ask_input(
        '>', find_match, 'Please enter one of the options.'
    ))
    return None if resp is ... else resp


def ask_input_lines(message: str, bullet: str = '>') -> list:
    print(message)
    lines = []
    while len(lines) < 1 or lines[-1]:
        lines.append(ask_input(bullet))
    return lines[:-1]


def ask_yes_no(message: str, default: Optional[bool]) -> bool:
    resp = ask_input(message, lambda x: (not x and default is not None) or x in 'yYnN')
    return {'n': False, 'y': True, '': default}[resp.lower()]


def create_or_edit_pr(title: str, body: str, skills_repo: Repository,
                      user, branch: str):
    base = skills_repo.default_branch
    head = '{}:{}'.format(user.login, branch)
    pulls = list(skills_repo.get_pulls(base=base, head=head))
    if pulls:
        pull = pulls[0]
        if 'mycroft-skills-kit' in pull.body:
            pull.edit(title, body)
        else:
            raise PRModified('Not updating description since it was not autogenerated')
        return pull
    else:
        return skills_repo.create_pull(title, body, base=base, head=head)


def to_camel(snake):
    """time_skill -> TimeSkill"""
    return snake.title().replace('_', '')


def to_snake(camel):
    """TimeSkill -> time_skill"""
    if not camel:
        return camel
    return ''.join('_' + x if 'A' <= x <= 'Z' else x for x in camel).lower()[camel[0].isupper():]


@contextmanager
def print_error(exception):
    try:
        yield
    except exception as e:
        print('{}: {}'.format(exception.__name__, e))


def read_file(*path):
    with open(join(*path)) as f:
        return f.read()


def read_lines(*path):
    with open(join(*path)) as f:
        return [i for i in (i.strip() for i in f.readlines()) if i]


def serialized(func):
    """Write a serializer by yielding each line of output"""

    @wraps(func)
    def wrapper(*args, **kwargs):
        return '\n'.join(
            ' '.join(parts) if isinstance(parts, tuple) else parts
            for parts in func(*args, **kwargs)
        )

    return wrapper
