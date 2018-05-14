/* Copyright (C) 2013-2016, The Regents of The University of Michigan.
All rights reserved.

This software was developed in the APRIL Robotics Lab under the
direction of Edwin Olson, ebolson@umich.edu. This software may be
available under alternative licensing terms; contact the address above.

This library is free software; you can redistribute it and/or
modify it under the terms of the GNU Lesser General Public
License as published by the Free Software Foundation; either
version 2.1 of the License, or (at your option) any later version.

This library is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public
License along with this library; if not, see <http://www.gnu.org/licenses/>.
*/

#include <stdio.h>

#include "utest/utest.h"

int main()
{
    TEST_ASSERT_EQUAL_INT(1, 1);
    // TEST_ASSERT_EQUAL_INT(1, 2);
    TEST_ASSERT_INT_WITHIN(5, 10, 12);
    // TEST_ASSERT_INT_WITHIN(5, 10, 20);
    // TEST_ASSERT_INT_WITHIN(5, 10, 1);
    int expected[] = {1,2,3,4};
    int actual1[] = {1,2,3,5};
    // TEST_ASSERT_EQUAL_INT_ARRAY(expected,actual1,4);
    int actual2[] = {1,2,3,5};
    TEST_ASSERT_EQUAL_INT_ARRAY(expected,actual2,3);
    // TEST_ASSERT_EQUAL_INT_ARRAY(expected,actual2,4);
    return 0;
}
