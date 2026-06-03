import sys

import pytest
from test_messages.msg import Foo


def test_foo():
    foo = Foo()
    assert foo.bar.baz.value == 0
    foo.bar.baz.value = 42
    assert foo.bar.baz.value == 42
    with pytest.raises(AttributeError):
        # This should raise an error because baz is not a member of Foo
        foo.baz.value = 42


if __name__ == '__main__':
    sys.exit(pytest.main())
