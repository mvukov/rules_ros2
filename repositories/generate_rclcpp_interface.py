import sys

import em

em.invoke([
    '-D',
    f'interface_name=\"{sys.argv[1]}\"',
    '-o',
    sys.argv[2],
    sys.argv[3],
])
