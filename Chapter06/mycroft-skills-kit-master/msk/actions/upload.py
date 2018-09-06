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
import os
import shutil
from argparse import ArgumentParser
from git import Git
from msm import SkillEntry
from os import listdir
from os.path import join, abspath, expanduser, basename

from msk.actions.create import CreateAction
from msk.console_action import ConsoleAction
from msk.exceptions import MskException
from msk.lazy import Lazy
from msk.repo_action import SkillData
from msk.util import skills_kit_footer, \
    create_or_edit_pr, ask_yes_no, skill_repo_name, read_file, ask_choice

body_template = '''
## Info

This PR adds the new skill, [{skill_name}]({skill_url}), to the skills repo.

## Description

{description}

''' + skills_kit_footer


class UploadAction(ConsoleAction):
    def __init__(self, args):
        folder = abspath(expanduser(args.skill_folder))
        self.entry = SkillEntry.from_folder(folder)
        skills_dir = abspath(expanduser(self.msm.skills_dir))
        if join(skills_dir, basename(folder)) != folder:
            raise MskException('Skill folder, {}, not directly within skills directory, {}.'.format(
                args.skill_folder, self.msm.skills_dir
            ))

    git = Lazy(lambda s: Git(s.entry.path))  # type: Git

    @staticmethod
    def register(parser: ArgumentParser):
        parser.add_argument('skill_folder')

    def perform(self):
        for i in listdir(self.entry.path):
            if i.lower() == 'readme.md' and i != 'README.md':
                shutil.move(join(self.entry.path, i), join(self.entry.path, 'README.md'))

        creator = CreateAction(None, self.entry.name.replace('-skill', ''))
        creator.path = self.entry.path
        creator.initialize_template({'.git', '.gitignore', 'README.md'})
        self.git.add('README.md')
        creator.commit_changes()
        skill_repo = creator.create_github_repo(lambda: input('Repo name:'))
        if skill_repo:
            self.entry.url = skill_repo.html_url
            self.entry.author = self.user.login
        else:
            skill_repo = self.github.get_repo(skill_repo_name(self.entry.url))

        if not skill_repo.permissions.push:
            print('Warning: You do not have write permissions to the provided skill repo.')
            if ask_yes_no('Create a fork and use that instead? (Y/n)', True):
                skill_repo = self.user.create_fork(skill_repo)
                print('Created fork:', skill_repo.html_url)
                self.git.remote('rename', 'origin', 'upstream')
                self.git.remote('add', 'origin', skill_repo.html_url)

        self.entry.name = input('Enter a unique skill name (ie. npr-news or grocery-list): ')

        readme_file = {i.lower(): i for i in os.listdir(self.entry.path)}['readme.md']
        readme = read_file(self.entry.path, readme_file)

        last_section = None
        sections = {last_section: ''}
        for line in readme.split('\n'):
            line = line.strip()
            if line.startswith('#'):
                last_section = line.strip('# ').lower()
                sections[last_section] = ''
            else:
                sections[last_section] += '\n' + line
        del sections[None]

        if 'description' in sections:
            description = sections['description']
        else:
            description = ask_choice(
                'Which section contains the description?', list(sections),
                on_empty='Please create a description section in the README'
            )

        branch = SkillData(self.entry).add_to_repo()
        self.repo.push_to_fork(branch)

        pull = create_or_edit_pr(
            title='Add {}'.format(self.entry.name), body=body_template.format(
                description=description, skill_name=self.entry.name, skill_url=skill_repo.html_url
            ), user=self.user, branch=branch, skills_repo=self.repo.hub
        )

        print('Created pull request: ', pull.html_url)
