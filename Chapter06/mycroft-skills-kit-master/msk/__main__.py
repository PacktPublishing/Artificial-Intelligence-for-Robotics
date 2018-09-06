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
import sys

from argparse import ArgumentParser
from msm import MycroftSkillsManager, SkillRepo

from msk.actions.create import CreateAction
from msk.actions.create_test import CreateTestAction
from msk.actions.upgrade import UpgradeAction
from msk.actions.upload import UploadAction
from msk.exceptions import MskException
from msk.global_context import GlobalContext

console_actions = {
    'upgrade': UpgradeAction,
    'upload': UploadAction,
    'create': CreateAction,
    'create-test': CreateTestAction
}


def main():
    parser = ArgumentParser()
    parser.add_argument('-l', '--lang', default='en-us')
    parser.add_argument('-u', '--repo-url', help='Url of GitHub repo to upload skills to')
    parser.add_argument('-b', '--repo-branch', help='Branch of skills repo to upload to')
    parser.add_argument('-s', '--skills-dir', help='Directory to look for skills in')
    parser.add_argument('-c', '--repo-cache', help='Location to store local skills repo clone')
    parser.add_argument('-t', '--use-token', action='store_true')

    subparsers = parser.add_subparsers(dest='action')
    subparsers.required = True
    for action, cls in console_actions.items():
        cls.register(subparsers.add_parser(action))

    args = parser.parse_args(sys.argv[1:])

    context = GlobalContext()
    context.lang = args.lang
    context.msm = MycroftSkillsManager(
        skills_dir=args.skills_dir, repo=SkillRepo(url=args.repo_url, branch=args.repo_branch)
    )
    context.use_token = args.use_token

    try:
        return console_actions[args.action](args).perform()
    except MskException as e:
        print('{}: {}'.format(e.__class__.__name__, str(e)))
    except (KeyboardInterrupt, EOFError):
        pass


if __name__ == '__main__':
    main()
