/*
 * Copyright (c) 2020-2024 Advanced Robotics at the University of Washington <robomstr@uw.edu>
 *
 * This file is part of aruw-mcb.
 *
 * aruw-mcb is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * aruw-mcb is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with aruw-mcb.  If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef ABSTRACT_STATE_GENERATOR_HPP_
#define ABSTRACT_STATE_GENERATOR_HPP_

namespace auto_nav::state_generators
{
/**
 * An interface for a state of the path following state machine. Each state is responsible for
 * determining the next state and the setpoint to follow.
 */
class AbstractStateGenerator
{
public:
    virtual constexpr ~AbstractStateGenerator() {}

    virtual int Id() const = 0;
};
}  // namespace aruwsrc::algorithms::auto_nav::state_generators

#endif  // ABSTRACT_STATE_GENERATOR_HPP_