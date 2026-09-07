# Copyright 2026 ROBOTIS AI CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Tests for the smooth keyboard controller math."""

import math

import pytest

from antbot_teleop.mapping_keyboard import motion_for_key
from antbot_teleop.teleop_smooth import step_towards


def test_step_towards_increases_without_overshoot():
    assert step_towards(0.0, 1.0, 0.2) == 0.2
    assert step_towards(0.9, 1.0, 0.2) == 1.0


def test_step_towards_decreases_without_overshoot():
    assert step_towards(1.0, 0.0, 0.2) == 0.8
    assert step_towards(0.1, 0.0, 0.2) == 0.0


@pytest.mark.parametrize('key,expected', [
    ('w', (1.0, 0.0, 0.0)),
    ('d', (0.0, -1.0, 0.0)),
    ('x', (-1.0, 0.0, 0.0)),
    ('a', (0.0, 1.0, 0.0)),
    ('r', (0.0, 0.0, -2.0)),
    ('t', (0.0, 0.0, 2.0)),
])
def test_omnidirectional_cardinal_and_rotation_bindings(key, expected):
    assert motion_for_key(key, 1.0, 2.0) == pytest.approx(expected)


@pytest.mark.parametrize('key,sign_x,sign_y', [
    ('q', 1.0, 1.0),
    ('e', 1.0, -1.0),
    ('c', -1.0, -1.0),
    ('z', -1.0, 1.0),
])
def test_diagonal_bindings_keep_constant_linear_speed(key, sign_x, sign_y):
    vx, vy, wz = motion_for_key(key, 1.0, 2.0)
    assert math.hypot(vx, vy) == pytest.approx(1.0)
    assert math.copysign(1.0, vx) == sign_x
    assert math.copysign(1.0, vy) == sign_y
    assert wz == 0.0
