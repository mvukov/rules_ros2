import sys

import em

em.invoke([
    '-o', sys.argv[1],
    '-D', 'rcutils_module_path=\"\"',  # No need to extend the Python path.
    sys.argv[2],
])
