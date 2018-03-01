#
# The MIT License (MIT)
#
# Copyright (c) 2017-2018 Zubax Robotics
#
# Permission is hereby granted, free of charge, to any person obtaining a copy of
# this software and associated documentation files (the "Software"), to deal in
# the Software without restriction, including without limitation the rights to
# use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
# the Software, and to permit persons to whom the Software is furnished to do so,
# subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
# FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
# COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
# IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
# CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
#
# Author: Pavel Kirienko <pavel.kirienko@zubax.com>
#

import datetime
import enum


class StringRepresentable:
    def __str__(self):
        def do_repr(x) -> str:
            if isinstance(x, (bytes, bytearray)):
                return x.hex()
            elif isinstance(x, datetime.datetime):
                return x.isoformat()
            elif isinstance(x, enum.Enum):
                return str(x)
            else:
                return repr(x)

        fields = ', '.join(('%s=%s' % (k, do_repr(v)))
                           for k, v in vars(self).items())

        return '%s(%s)' % (type(self).__name__, fields)

    __repr__ = __str__
