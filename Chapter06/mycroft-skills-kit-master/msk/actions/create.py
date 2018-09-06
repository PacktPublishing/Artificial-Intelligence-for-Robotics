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

import re
from argparse import ArgumentParser
from git import Git
from github import GithubException
from github.Repository import Repository
from os import makedirs
from os.path import join, exists, isdir
from shutil import rmtree
from subprocess import call
from typing import Callable, Optional

from msk.console_action import ConsoleAction
from msk.exceptions import GithubRepoExists
from msk.lazy import Lazy
from msk.util import ask_input, to_camel, ask_yes_no, ask_input_lines, \
    print_error

readme_template = '''## {title_name}
{short_description}

## Description
{long_description}

## Examples
{examples}

{credits}
'''

credits_template = '''## Credits
{author}

'''

init_template = '''from mycroft import MycroftSkill, intent_file_handler


class {class_name}(MycroftSkill):
    def __init__(self):
        MycroftSkill.__init__(self)

    @intent_file_handler('{intent_name}.intent')
    def handle_{handler_name}(self, message):
{handler_code}


def create_skill():
    return {class_name}()

'''

gitignore_template = '''*.pyc
settings.json

'''

settingsmeta_template = '''{{
    "name": "{capital_desc}",
    "skillMetadata": {{
        "sections": [
            {{
                "name": "Options << Name of section",
                "fields": [
                    {{
                        "name": "internal_python_variable_name",
                        "type": "text",
                        "label": "Setting Friendly Display Name",
                        "value": "",
                        "placeholder": "demo prompt in the input box"
                    }}
                ]
            }},
            {{
                "name": "Login << Name of another section",
                "fields": [
                    {{
                        "type": "label",
                        "label": "Just a little bit of extra info for the user to understand following settings"
                    }},
                    {{
                        "name": "username",
                        "type": "text",
                        "label": "Username",
                        "value": ""
                    }},
                    {{
                        "name": "password",
                        "type": "password",
                        "label": "Password",
                        "value": ""
                    }}
                ]
            }}
        ]
    }}
}}'''


class CreateAction(ConsoleAction):
    def __init__(self, args, name: str = None):
        if name:
            self.name = name

    @staticmethod
    def register(parser: ArgumentParser):
        pass

    @Lazy
    def name(self) -> str:
        name_to_skill = {skill.name: skill for skill in self.msm.list()}
        while True:
            name = ask_input(
                'Enter a short unique skill name (ie. "siren alarm" or "pizza orderer"):',
                lambda x: re.match(r'^[a-zA-Z \-]+$', x), 'Please use only letter and spaces.'
            ).strip(' -').lower().replace(' ', '-')
            skill = name_to_skill.get(name, name_to_skill.get('{}-skill'.format(name)))
            if skill:
                print('The skill {} {}already exists'.format(
                    skill.name, 'by {} '.format(skill.author) * bool(skill.author)
                ))
                if ask_yes_no('Remove it? (y/N)', False):
                    rmtree(skill.path)
                else:
                    continue
            class_name = '{}Skill'.format(to_camel(name.replace('-', '_')))
            repo_name = '{}-skill'.format(name)
            print()
            print('Class name:', class_name)
            print('Repo name:', repo_name)
            print()
            alright = ask_yes_no('Looks good? (Y/n)', True)
            if alright:
                return name

    path = Lazy(lambda s: join(s.msm.skills_dir, s.name + '-skill'))
    git = Lazy(lambda s: Git(s.path))
    short_description = Lazy(lambda s: ask_input(
        'Enter a one line description for your skill (ie. Orders fresh pizzas from the store):',
    ).capitalize())
    author = Lazy(lambda s: ask_input('Enter author:'))
    intent_lines = Lazy(lambda s: [
        i.capitalize() for i in ask_input_lines(
            'Enter some example phrases to trigger your skill:', '-'
        )
    ])
    dialog_lines = Lazy(lambda s: [
        i.capitalize() for i in ask_input_lines(
            'Enter what your skill should say to respond:', '-'
        )
    ])
    intent_entities = Lazy(lambda s: set(re.findall(
        r'(?<={)[a-z_A-Z]*(?=})', '\n'.join(i for i in s.intent_lines)
    )))
    dialog_entities = Lazy(lambda s: set(re.findall(
        r'(?<={)[a-z_A-Z]*(?=})', '\n'.join(s.dialog_lines)
    )))
    long_description = Lazy(lambda s: '\n\n'.join(
        ask_input_lines('Enter a long description:', '>')
    ).strip().capitalize())
    readme = Lazy(lambda s: readme_template.format(
        title_name=s.name.replace('-', ' ').title(),
        short_description=s.short_description,
        long_description=s.long_description,
        examples=''.join(' - "{}"\n'.format(i) for i in s.intent_lines),
        credits=credits_template.format(author=s.author)
    ))
    init_file = Lazy(lambda s: init_template.format(
        class_name=to_camel(s.name.replace('-', '_')),
        handler_name=s.intent_name.replace('.', '_'),
        handler_code='\n'.join(
            ' ' * 8 * bool(i) + i
            for i in [
                "{ent} = message.data.get('{ent}')".format(ent=entity)
                for entity in sorted(s.intent_entities)
            ] + [
                "{ent} = ''".format(ent=entity)
                for entity in sorted(s.dialog_entities - s.intent_entities)
            ] + [''] * bool(
                s.dialog_entities | s.intent_entities
            ) + "self.speak_dialog('{intent}'{args})".format(
                intent=s.intent_name, args=", data={{\n{}\n}}".format(',\n'.join(
                    "    '{ent}': {ent}".format(ent=entity)
                    for entity in s.dialog_entities | s.intent_entities
                )) * bool(s.dialog_entities | s.intent_entities)
            ).split('\n')
        ),
        intent_name=s.intent_name
    ))
    intent_name = Lazy(lambda s: '.'.join(reversed(s.name.split('-'))))

    def add_vocab(self):
        makedirs(join(self.path, 'vocab', self.lang))
        with open(join(self.path, 'vocab', self.lang, self.intent_name + '.intent'), 'w') as f:
            f.write('\n'.join(self.intent_lines + ['']))

    def add_dialog(self):
        makedirs(join(self.path, 'dialog', self.lang))
        with open(join(self.path, 'dialog', self.lang, self.intent_name + '.dialog'), 'w') as f:
            f.write('\n'.join(self.dialog_lines + ['']))

    def initialize_template(self, files: set = None):
        git = Git(self.path)

        skill_template = [
            ('', lambda: makedirs(self.path)),
            ('vocab', self.add_vocab),
            ('dialog', self.add_dialog),
            ('__init__.py', lambda: self.init_file),
            ('README.md', lambda: self.readme),
            ('.gitignore', lambda: gitignore_template),
            ('settingsmeta.json', lambda: settingsmeta_template.format(
                capital_desc=self.name.replace('-', ' ').capitalize()
            )),
            ('.git', lambda: git.init())
        ]

        def cleanup():
            rmtree(self.path)

        if not isdir(self.path):
            atexit.register(cleanup)
        for file, handler in skill_template:
            if files and file not in files:
                continue
            if not exists(join(self.path, file)):
                result = handler()
                if isinstance(result, str) and not exists(join(self.path, file)):
                    with open(join(self.path, file), 'w') as f:
                        f.write(result)
        atexit.unregister(cleanup)

    def commit_changes(self):
        if self.git.rev_parse('HEAD', with_exceptions=False) == 'HEAD':
            self.git.add('.')
            self.git.commit(message='Initial commit')

    def create_github_repo(self, get_repo_name: Callable = None) -> Optional[Repository]:
        if 'origin' not in Git(self.path).remote().split('\n'):
            if ask_yes_no('Would you like to create a GitHub repo for it? (Y/n)', True):
                repo_name = (get_repo_name and get_repo_name()) or (self.name + '-skill')
                try:
                    repo = self.user.create_repo(repo_name, self.short_description)
                except GithubException as e:
                    if e.status == 422:
                        raise GithubRepoExists(repo_name) from e
                    raise
                self.git.remote('add', 'origin', repo.html_url)
                call(['git', 'push', '-u', 'origin', 'master'], cwd=self.git.working_dir)
                print('Created GitHub repo:', repo.html_url)
                return repo
        return None

    def perform(self):
        self.initialize_template()
        self.commit_changes()
        with print_error(GithubRepoExists):
            self.create_github_repo()
        print('Created skill at:', self.path)
