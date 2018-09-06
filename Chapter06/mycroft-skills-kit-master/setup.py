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
from setuptools import setup

setup(
    name='msk',
    version='0.3.10',  # Also update in msk/__init__.py
    packages=['msk', 'msk.actions'],
    install_requires=['GitPython', 'typing', 'msm>=0.5.13', 'pygithub'],
    url='https://github.com/MycroftAI/mycroft-skills-kit',
    license='Apache-2.0',
    author='Mycroft AI',
    author_email='support@mycroft.ai',
    maintainer='Matthew Scholefield',
    maintainer_email='matthew331199@gmail.com',
    description='Mycroft Skills Kit',
    entry_points={
        'console_scripts': {
            'msk=msk.__main__:main'
        }
    }
)
